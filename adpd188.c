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
static int8_t adpd188_read_data(uint8_t reg_addr, uint16_t *reg_data,
		void *intf);
static int8_t adpd188_write_data(uint8_t reg_addr, const uint16_t reg_data,
		void *intf);
static bool set_n_bits(uint16_t *target, uint8_t n, uint16_t val,
		uint8_t index);
static bool get_n_bits(uint16_t target, uint8_t n, uint16_t *val,
		uint8_t index);

static void print_binary(uint16_t val);
static void print_test(uint16_t bits_val);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a ADPD188 instance.
 */
esp_err_t adpd188_init(adpd188_t *const me, i2c_bus_t *i2c_bus,
		uint8_t dev_addr, int int_gpio) {
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
	me->int_gpio = int_gpio;

	gpio_config_t gpio_conf = {
			.pin_bit_mask = (1ULL << me->int_gpio),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
	};

	ret = gpio_config(&gpio_conf);

	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure int GPIO");
		return ret;
	}

	/*  */
	adpd188_soft_reset(me);
	adpd188_set_mode(me, ADPD188_MODE_IDLE);

	uint16_t devid;

	adpd188_read_data(ADPD188_REG_DEVID, &devid, me->i2c_dev);
	printf("ID: 0x%.2X\r\n", (uint16_t)(devid  & 0xFF));
	printf("Rev: 0x%.2X\r\n", (uint16_t)((devid >> 8) & 0xFF));

  adpd188_write_data(ADPD188_REG_SLOT_EN, 0x30A9, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_FSAMPLE, 0x0200, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_PD_LED_SELECT, 0x011D, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_NUM_AVG, 0x0000, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_INT_SEQ_A, 0x0009, me->i2c_dev);

  adpd188_write_data(ADPD188_REG_SLOTA_CH1_OFFSET, 0x0000, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_CH2_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_CH3_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_CH4_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_INT_SEQ_B, 0x0009, me->i2c_dev);

  adpd188_write_data(ADPD188_REG_SLOTB_CH1_OFFSET, 0x0000, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_CH2_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_CH3_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_CH4_OFFSET, 0x3FFF, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_ILED3_COARSE, 0x3539, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_ILED1_COARSE, 0x3536, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_ILED2_COARSE, 0x1530, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_ILED_FINE, 0x630C, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_LED_PULSE, 0x0320, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_NUM_PULSES, 0x040E, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_LED_PULSE, 0x0320, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_NUM_PULSES, 0x040E, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_AFE_WINDOW, 0x22F0, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_AFE_WINDOW, 0x22F0, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_AFE_PWR_CFG1, 0x31C6, me->i2c_dev);

  adpd188_write_data(ADPD188_REG_SLOTA_TIA_CFG, 0x1C34, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTA_AFE_CFG, 0xADA5, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_TIA_CFG, 0x1C34, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_SLOTB_AFE_CFG, 0xADA5, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_MATH, 0x0544, me->i2c_dev);
  adpd188_write_data(ADPD188_REG_AFE_PWR_CFG2, 0x0AA0, me->i2c_dev);

  adpd188_write_data(ADPD188_REG_DATA_ACCESS_CTL, 0x0007, me->i2c_dev);

  adpd188_set_mode(me, ADPD188_MODE_PROGRAM);

  adpd188_set_bit(me, ADPD188_REG_SAMPLE_CLK, 7, 1);
  adpd188_set_bit(me, ADPD188_REG_DATA_ACCESS_CTL, 0, 1);
  adpd188_set_bit(me, ADPD188_REG_INT_MASK, 5, 0);
  adpd188_set_bit(me, ADPD188_REG_INT_MASK, 6, 1);
  adpd188_set_bit(me, ADPD188_REG_INT_MASK, 8, 1);
  adpd188_set_bit(me, ADPD188_REG_GPIO_DRV, 0, 1);
  adpd188_set_bit(me, ADPD188_REG_GPIO_DRV, 1, 1);
  adpd188_set_bit(me, ADPD188_REG_GPIO_DRV, 2, 1);

  adpd188_write_data(ADPD188_REG_SLOT_EN, 0x3001, me->i2c_dev);

  adpd188_set_mode(me, ADPD188_MODE_NORMAL);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to set the working mode of the ADPD188
 */
esp_err_t adpd188_set_mode(adpd188_t *const me, adpd188_mode_e mode) {
	esp_err_t ret = ESP_OK;

	adpd188_set_bit_mask(me, ADPD188_REG_MODE, 2, (uint16_t)mode);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to set the working mode of the ADPD188
 */
esp_err_t adpd188_get_mode(adpd188_t *const me, adpd188_mode_e *mode) {
	esp_err_t ret = ESP_OK;

	adpd188_read_data(ADPD188_REG_MODE, (uint16_t *)&mode, me->i2c_dev);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform software reset
 */
esp_err_t adpd188_soft_reset(adpd188_t *const me) {
	esp_err_t ret = ESP_OK;

	adpd188_write_data(ADPD188_REG_SW_RESET, 0x0001, me->i2c_dev);

	vTaskDelay(pdMS_TO_TICKS(100));

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_get_int(adpd188_t *const me, uint8_t *fifo, uint8_t *slot_a,
		uint8_t *slot_b) {
	esp_err_t ret = ESP_OK;
	uint16_t reg_val = 0;

	adpd188_read_data(ADPD188_REG_STATUS, &reg_val, me->i2c_dev);
	print_test(reg_val);

  *fifo = (reg_val >> 8) & 0xFF;
  *slot_a = (reg_val >> 5) & 0x01;
  *slot_b = (reg_val >> 6) & 0x01;

  adpd188_write_data(ADPD188_REG_STATUS, 0xFFFF, me->i2c_dev);
  adpd188_read_data(ADPD188_REG_STATUS, &reg_val, me->i2c_dev);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_read_sens_data(adpd188_t *const me, uint8_t slot, uint8_t ch,
		uint16_t *data) {
	esp_err_t ret = ESP_OK;

	/**/
	adpd188_set_bit(me, ADPD188_REG_DATA_ACCESS_CTL, 1 + slot, 1);
	adpd188_read_data(ADPD188_REG_SLOTA_CH1 + slot + ch, data, me->i2c_dev);
	adpd188_set_bit(me, ADPD188_REG_DATA_ACCESS_CTL, 1 + slot, 0);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_set_bit(adpd188_t *const me, uint8_t reg_addr, uint8_t bit_num,
		bool bit_val) {
	esp_err_t ret = ESP_OK;
	uint16_t reg_data = 0x0;

	/* Read data register, set the specified bit number and write the new value */
	adpd188_read_data(reg_addr, &reg_data, me->i2c_dev);
	set_n_bits(&reg_data, 1, bit_val, bit_num);
	adpd188_write_data(reg_addr, reg_data, me->i2c_dev);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_set_bit_mask(adpd188_t *const me, uint8_t reg_addr,
		uint8_t bits_num,	uint16_t bits_val) {
	esp_err_t ret = ESP_OK;
	uint16_t reg_data = 0x0;

	/* Read data register, set the specified bit number and write the new value */
	adpd188_read_data(reg_addr, &reg_data, me->i2c_dev);
	set_n_bits(&reg_data, bits_num, bits_val, 0);
	adpd188_write_data(reg_addr, reg_data, me->i2c_dev);

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_calibration(adpd188_t *const me, uint16_t threshold) {
	esp_err_t ret = ESP_OK;

	me->threshold_value = threshold;

	adpd188_read_data(ADPD188_REG_SLOT_EN, &me->threshold_value , me->i2c_dev);

	if ((me->threshold_value  & 0x0001) == 0x0001) {
		me->enabled_slot = ADPD188_SLOT_A;
	}
	else if ((me->threshold_value  & 0x0020) == 0x0020) {
		me->enabled_slot = ADPD188_SLOT_B;
	}

	/* Read data for calibration */
	uint32_t sum = 0;

	for (uint8_t i = 0; i < 64; i++) {
		if (!adpd188_get_int_gpio(me)) {
			uint16_t reg_data = 0x0;
			adpd188_read_sens_data(me, me->enabled_slot, ADPD188_CH_1, &reg_data);
			sum += reg_data;
		}
	}

	me->calib_value = sum >> 6;

	/* Return ESP_OK*/
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
uint16_t adpd188_get_calib(adpd188_t *const me) {
	return me->calib_value;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
esp_err_t adpd188_check_smoke(adpd188_t *const me, adpd188_smoke_e *smoke) {
	esp_err_t ret = ESP_OK;

	/**/
	if (!adpd188_get_int_gpio(me)) {
		uint16_t reg_data = 0x0;
		adpd188_read_sens_data(me, me->enabled_slot, ADPD188_CH_1, &reg_data);

		if (reg_data > (me->calib_value + me->threshold_value)){
			*smoke = ADPD188_SMOKE_DETECTED;
		}
		else {
			*smoke = ADPD188_SMOKE_NOT_DETECTED;
		}
	}

	*smoke = ADPD188_SMOKE_ERROR;

	/* Return ESP_OK */
	return ret;
}

/**
 * @brief Function to perform a software reset todo: write correctly
 */
int adpd188_get_int_gpio(adpd188_t *const me) {
	return gpio_get_level(me->int_gpio);
}

/* Private function definitions ----------------------------------------------*/
static int8_t adpd188_read_data(uint8_t reg_addr, uint16_t *reg_data, void *intf) {
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

static int8_t adpd188_write_data(uint8_t reg_addr, const uint16_t reg_data, void *intf) {
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

static bool set_n_bits(uint16_t *target, uint8_t n, uint16_t val,
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
	*target = (*target & ~(mask << index)) | ((val & mask) << index);

	return true;
}

/**
 * @brief Function to read n bits_val for an uint32_t variable in a specific index
 */
static bool get_n_bits(uint16_t target, uint8_t n, uint16_t *val,
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

	/* Read the bits_val */
	*val = (target >> index) & mask;

	return true;
}

static void print_binary(uint16_t val) {
	for (int i = 15; i >= 0; i--) {
        if (i > 9) {
            printf(" ");
        }

		printf("%d ", val & (0x1 << i) ? 1 : 0);
	}
	printf("\r\n");
}

static void print_test(uint16_t bits_val) {
    print_binary(bits_val);

    for (int i = 15; i >= 0; i--) {
        printf("%d ", i);
    }

    printf("\n");
}

/***************************** END OF FILE ************************************/
