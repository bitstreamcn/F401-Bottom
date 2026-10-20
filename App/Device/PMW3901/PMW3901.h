/*
 * PMW3901.h
 *
 *  Created on: Jul 4, 2025
 *      Author: bitstream
 */

#ifndef PMW3901_PMW3901_H_
#define PMW3901_PMW3901_H_

#include <stdbool.h>

#include "interface/driver_pmw3901mb_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @addtogroup pmw3901mb_example_driver
 * @{
 */

/**
 * @brief     interrupt irq
 * @param[in] height_m height(m)
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 * @note      none
 */
uint8_t pmw3901mb_interrupt_irq_handler(float height_m);

/**
 * @brief     interrupt example init
 * @param[in] *callback pointer to a callback function
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t pmw3901mb_interrupt_init(void (*callback)(pmw3901mb_motion_t *motion, float delta_x, float delta_y));

/**
 * @brief  interrupt example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t pmw3901mb_interrupt_deinit(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif


void PMW_Task();

void PMWMotionHandler();


#endif /* PMW3901_PMW3901_H_ */
