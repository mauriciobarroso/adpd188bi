/**
  ******************************************************************************
  * @file           : adpd188.c
  * @author         : Mauricio Barroso Benavides
  * @date           : Aug 4, 2023
  * @brief          : todo: write brief
  ******************************************************************************
  * @attention
  *
  * MIT License
  *
  * Copyright (c) 2023 Mauricio Barroso Benavides
  *
  * Permission is hereby granted, free of charge, to any person obtaining a copy
  * of this software and associated documentation files (the "Software"), to
  * deal in the Software without restriction, including without limitation the
  * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
  * sell copies of the Software, and to permit persons to whom the Software is
  * furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  * 
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adpd188.h"
#include "esp_err.h"
#include "esp_log.h"

/* Private macros ------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "adpd188";

/* Private function prototypes -----------------------------------------------*/
static int8_t i2c_read(uint8_t reg_addr, uint16_t *reg_data, void *intf);
static int8_t i2c_write(uint8_t reg_addr, const uint16_t reg_data, void *intf);
static bool set_n_bits(uint16_t *target, uint16_t bits_val_val, uint8_t n,
		uint8_t index);
static bool get_n_bits(uint16_t target, uint16_t *bits_val_val,  uint8_t n,
		uint8_t index);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a ADPD188 instance.
 */
esp_err_t adpd188_init(adpd188_t *const me, i2c_bus_t *i2c_bus, uint8_t dev_addr) {
	/* Print initializing message */
	ESP_LOGI(TAG, "Initializing instance...");

	esp_err_t ret = ESP_OK;

	/* Add device to I2C bus */
	if ( i2c_bus != NULL) {
		ret = i2c_bus_add_dev(i2c_bus, dev_addr, "adpd188", NULL, NULL);

		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to add device to I2C bus");
			return ret;
		}

		/* Reference the new device created to instance structure */
		me->i2c_dev = &i2c_bus->devs.dev[i2c_bus->devs.num - 1]; /* todo: write function to get the dev from name */
	}

	/* Print successful initialization message */
	ESP_LOGI(TAG, "Instance initialized successfully");

	/* Initialize interrupt GPIO */

	/*  */

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to set the working mode of the ADPD188
 */
esp_err_t adpd188_set_mode(adpd188_t *const me, adpd188_mode_e mode) {
	esp_err_t ret = ESP_OK;

	i2c_write(ADPD188_REG_MODE, (mode & 0x3), me->i2c_dev);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform software reset
 */
esp_err_t adpd188_soft_reset(adpd188_t *const me) {
	esp_err_t ret = ESP_OK;

	i2c_write(ADPD188_REG_SW_RESET, 0x0001, me->i2c_dev);

	vTaskDelay(pdMS_TO_TICKS(100));

	/* Return ESP_OK */
	return ret;
}

/* Private function definitions ----------------------------------------------*/
static int8_t i2c_read(uint8_t reg_addr, uint16_t *reg_data, void *intf) {
	int8_t ret = 0;
	i2c_bus_dev_t *dev = (i2c_bus_dev_t *)intf;

	uint8_t buf[2] = {0};

	ret =  dev->read(reg_addr ? &reg_addr : NULL, reg_addr ? 1 : 0, buf, 2, dev);

	if (ret < 0) {
		return ret;
	}

	*reg_data = (uint16_t)((buf[0] << 8) | buf[1]);

	/* Return for success */
	return ret;
}

static int8_t i2c_write(uint8_t reg_addr, const uint16_t reg_data, void *intf) {
	int8_t ret = 0;
	i2c_bus_dev_t *dev = (i2c_bus_dev_t *)intf;

	uint8_t buf[2] = {(uint8_t)((reg_data & 0xFF00) >> 8),
			(uint8_t)(reg_data & 0x00FF)};

	ret = dev->write(&reg_addr, 1, buf, 2, dev);

	if (ret < 0) {
		return ret;
	}

	/* Return for success */
	return ret;
}

static bool set_n_bits(uint16_t *target, uint16_t bits_val, uint8_t n,
		uint8_t index) {
	/* Check for a valid index value */
	if (index > (16 - n) || n < 1) {
		printf("Index out of bounds or less than 1\r\n");
		return false;
	}

	/* Create the mask */
	uint16_t mask = 0;

	if (n == 16) {
		mask = ~mask;
	}
	else {
		mask = (0x1 << n) - 1;
	}

	/* Write the bits_val */
	*target = (*target & ~(mask << index)) | ((bits_val & mask) << index);

	return true;
}

/**
 * @brief Function to read n bits_val for an uint32_t variable in a specific index
 */
static bool get_n_bits(uint16_t target, uint16_t *bits_val,  uint8_t n, uint8_t index) {
	/* Check for a valid index value */
	if (index > (16 - n) || n < 1) {
		printf("Index out of bounds or less than 1\r\n");
		return false;
	}

	/* Create the mask */
	uint16_t mask = 0;

	if (n == 16) {
		mask = ~mask;
	}
	else {
		mask = (0x1 << n) - 1;
	}

	/* Read the bits_val */
	*bits_val = (target >> index) & mask;

	return true;
}

/***************************** END OF FILE ************************************/
