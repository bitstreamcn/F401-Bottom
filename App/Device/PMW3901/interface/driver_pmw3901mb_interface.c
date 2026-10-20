/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_pmw3901mb_interface_template.c
 * @brief     driver pmw3901mb interface template source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-01-08
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/01/08  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */
#include "main.h"
#include "driver_pmw3901mb_interface.h"
#include "spi.h"
#include "log.h"
#include "common.h"
#include <stdio.h>
#include <string.h>
#include "tx_api.h"

#define MS_TO_TICKS(ms)  (((ms) * TX_TIMER_TICKS_PER_SECOND) / 1000)

// 读取寄存器
HAL_StatusTypeDef PMW3901_Read_Reg(uint8_t reg, uint8_t * data, uint16_t len) {
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET); // 拉低片选

	uint8_t tx_data[64];
	uint8_t rx_data[64];

	if (len > (sizeof(rx_data) - 1))
	{
		LogE("PMW3901_Read_Reg len Err: %x", len);
		return HAL_ERROR;
	}

	tx_data[0] = reg;

	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, len + 1, 2); // 超时1ms
	delay_us(10);

	if (HAL_OK != ret)
	{
	  LogE("PMW3901_Read_Reg Err: %x", ret);
	  return ret;
	}
	memcpy(data, &rx_data[1], len);

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET); // 释放片选
	return HAL_OK;
}

// 写入寄存器
HAL_StatusTypeDef PMW3901_Write_Reg(uint8_t reg, const uint8_t * data, uint16_t len) {
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	uint8_t tx_data[64];
	uint8_t rx_data[64];

	if (len > (sizeof(tx_data) - 1))
	{
		LogE("PMW3901_Write_Reg len Err: %x", len);
		return HAL_ERROR;
	}

	tx_data[0] = reg;
	memcpy(&tx_data[1], data, len);

	delay_us(10);
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, len+1, 2); // 超时1ms
	delay_us(10);

	if (HAL_OK != ret)
	{
	  LogE("PMW3901_Write_Reg Err: %x", ret);
	  return ret;
	}

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	return HAL_OK;
}






/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t pmw3901mb_interface_spi_init(void)
{
    return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t pmw3901mb_interface_spi_deinit(void)
{
    return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t pmw3901mb_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef ret = PMW3901_Read_Reg(reg, buf, len);

	return (ret == HAL_OK) ? 0 : -1;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t pmw3901mb_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef ret = PMW3901_Write_Reg(reg, buf, len);
	 return (ret == HAL_OK) ? 0 : -1;
}

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t pmw3901mb_interface_reset_gpio_init(void)
{
    return 0;
}

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t pmw3901mb_interface_reset_gpio_deinit(void)
{
    return 0;
}

/**
 * @brief     interface reset gpio write
 * @param[in] data written data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t pmw3901mb_interface_reset_gpio_write(uint8_t data)
{
	HAL_GPIO_WritePin(PMW_NREST_GPIO_Port, PMW_NREST_Pin, data ? GPIO_PIN_SET : GPIO_PIN_SET);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void pmw3901mb_interface_delay_ms(uint32_t ms)
{
	//delay(ms);
	tx_thread_sleep(ms);
	//HAL_Delay(ms);
}

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void pmw3901mb_interface_debug_print(const char *const fmt, ...)
{
	char buffer[128];
    va_list ap;
    va_start(ap, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, ap);
	va_end(ap);
    LogE(buffer);
}
