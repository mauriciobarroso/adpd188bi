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

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a ADPD188 instance.
 */
esp_err_t adpd188_init(adpd188_t *const me, i2c_bus_t *i2c_bus, uint8_t dev_addr) {
	/* Print initializing message */
	ESP_LOGI(TAG, "Initializing instance...");

	esp_err_t ret = ESP_OK;

	/* Add devit to bus */
	ret = i2c_bus_add_dev(i2c_bus, dev_addr, TAG, NULL, NULL);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to add device to I2C bus");
		return ret;
	}

	/* Reference the new device created to instance structure */
	me->i2c_dev = &i2c_bus->devs.dev[i2c_bus->devs.num - 1]; /* todo: write function to get the dev from name */

	/* Print successful initialization message */
	ESP_LOGI(TAG, "Instance initialized successfully");

	/* Return ESP_OK */
	return ret;
}

/* Private function definitions ----------------------------------------------*/

/***************************** END OF FILE ************************************/
