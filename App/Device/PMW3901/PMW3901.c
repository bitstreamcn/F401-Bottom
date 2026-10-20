/*
 * PMW3901.c
 *
 *  Created on: Jul 4, 2025
 *      Author: bitstream
 */
#include "main.h"
#include "PMW3901.h"
#include "spi.h"
#include <stdbool.h>
#include <stdint.h>

extern float GetAlt(void);

volatile uint8_t pmw_irq = 0;

void PMWMotionHandler()
{
	//pmw3901mb_interrupt_irq_handler(1.0f);
	pmw_irq++;
}

// 在任务中读取数据
void PMW_Task()
{
	if (pmw_irq > 0)
	{
		pmw_irq = 0;
		float alt = GetAlt();
		if (alt == 0)
		{
			alt = 1.0f;
		}
		if (alt > 4)
		{
			alt = 4;
		}
		pmw3901mb_interrupt_irq_handler(alt);
	}
}

static pmw3901mb_handle_t gs_handle;                                                           /**< pmw3901mb handle */
static void (*gs_irq)(pmw3901mb_motion_t *motion, float delta_x, float delta_y) = NULL;        /**< irq */

/**
 * @brief     interrupt irq
 * @param[in] height_m height(m)
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 * @note      none
 */
uint8_t pmw3901mb_interrupt_irq_handler(float height_m)
{
    uint8_t res;
    float delta_x;
    float delta_y;
    pmw3901mb_motion_t motion;

    /* burst read */
    res = pmw3901mb_burst_read(&gs_handle, &motion);
    if (res != 0)
    {
        pmw3901mb_interface_debug_print("pmw3901mb: burst read failed.\n");
        (void)pmw3901mb_set_motion(&gs_handle, 0x00);

        return 1;
    }

    /* check the result */
    if (motion.is_valid == 1)
    {
        /* convert the delta x */
        res = pmw3901mb_delta_raw_to_delta_cm(&gs_handle, motion.delta_x, height_m, (float *)&delta_x);
        if (res != 0)
        {
            pmw3901mb_interface_debug_print("pmw3901mb: delta raw to delta cm failed.\n");
            (void)pmw3901mb_set_motion(&gs_handle, 0x00);

            return 1;
        }

        /* convert the delta y */
        res = pmw3901mb_delta_raw_to_delta_cm(&gs_handle, motion.delta_y, height_m, (float *)&delta_y);
        if (res != 0)
        {
            pmw3901mb_interface_debug_print("pmw3901mb: delta raw to delta cm failed.\n");
            (void)pmw3901mb_set_motion(&gs_handle, 0x00);

            return 1;
        }

        /* run the callback */
        if (gs_irq != NULL)
        {
            gs_irq(&motion, delta_x, delta_y);
        }
    }

    /* clear the interrupt flag */
    res = pmw3901mb_set_motion(&gs_handle, 0x00);
    if (res != 0)
    {
        pmw3901mb_interface_debug_print("pmw3901mb: set motion failed.\n");

        return 1;
    }

    return 0;
}

void setLed(bool ledOn)
{
	uint8_t val = 0x14;
	pmw3901mb_interface_spi_write(0x7f, &val, 1);
	val = ledOn ? 0x1c : 0x00;
	pmw3901mb_interface_spi_write(0x6f, &val, 1);
	val = 0x00;
	pmw3901mb_interface_spi_write(0x7f, &val, 1);
}

/**
 * @brief     interrupt example init
 * @param[in] *callback pointer to a callback function
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t pmw3901mb_interrupt_init(void (*callback)(pmw3901mb_motion_t *motion, float delta_x, float delta_y))
{
    uint8_t res;

    /* link interface function */
    DRIVER_PMW3901MB_LINK_INIT(&gs_handle, pmw3901mb_handle_t);
    DRIVER_PMW3901MB_LINK_SPI_INIT(&gs_handle, pmw3901mb_interface_spi_init);
    DRIVER_PMW3901MB_LINK_SPI_DEINIT(&gs_handle, pmw3901mb_interface_spi_deinit);
    DRIVER_PMW3901MB_LINK_SPI_READ(&gs_handle, pmw3901mb_interface_spi_read);
    DRIVER_PMW3901MB_LINK_SPI_WRITE(&gs_handle, pmw3901mb_interface_spi_write);
    DRIVER_PMW3901MB_LINK_RESET_GPIO_INIT(&gs_handle, pmw3901mb_interface_reset_gpio_init);
    DRIVER_PMW3901MB_LINK_RESET_GPIO_DEINIT(&gs_handle, pmw3901mb_interface_reset_gpio_deinit);
    DRIVER_PMW3901MB_LINK_RESET_GPIO_WRITE(&gs_handle, pmw3901mb_interface_reset_gpio_write);
    DRIVER_PMW3901MB_LINK_DELAY_MS(&gs_handle, pmw3901mb_interface_delay_ms);
    DRIVER_PMW3901MB_LINK_DEBUG_PRINT(&gs_handle, pmw3901mb_interface_debug_print);

    /* init pmw3901mb */
    res = pmw3901mb_init(&gs_handle);
    if (res != 0)
    {
        pmw3901mb_interface_debug_print("pmw3901mb: init failed.\n");

        return 1;
    }

    /* chip power up */
    res = pmw3901mb_power_up(&gs_handle);
    if (res != 0)
    {
        pmw3901mb_interface_debug_print("pmw3901mb: power up failed.\n");
        (void)pmw3901mb_deinit(&gs_handle);

        return 1;
    }

    /* set optimum performance */
    res = pmw3901mb_set_optimum_performance(&gs_handle);
    if (res != 0)
    {
        pmw3901mb_interface_debug_print("pmw3901mb: set optimum performance failed.\n");
        (void)pmw3901mb_deinit(&gs_handle);

        return 1;
    }

    //打开LED
    HAL_GPIO_WritePin(PMW_LED_GPIO_Port, PMW_LED_Pin, GPIO_PIN_SET);
    setLed(true);

    /* set callback */
    gs_irq = callback;

    return 0;
}

/**
 * @brief  interrupt example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t pmw3901mb_interrupt_deinit(void)
{
    gs_irq = NULL;

    if (pmw3901mb_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}





