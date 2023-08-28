/**
  ******************************************************************************
  * @file           : adpd188.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADPD188_H_
#define ADPD188_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include "i2c_bus.h"

/* Exported Macros -----------------------------------------------------------*/
/* ADPD188 I2C device address */
#define ADPD188_I2C_ADDR							0x64

/* ADPD188 registers */
#define ADPD188_REG_STATUS						0x00
#define ADPD188_REG_INT_MASK					0x01
#define ADPD188_REG_GPIO_DRV					0x02
#define ADPD188_REG_BG_STATUS					0x04
#define ADPD188_REG_FIFO_THR					0x06
#define ADPD188_REG_DEVID							0x08
#define ADPD188_REG_I2CS_ID						0x09
#define ADPD188_REG_CLK_RATIO					0x0A
#define ADPD188_REG_GPIO_CTRL					0x0B
#define ADPD188_REG_SLAVE_ADDR_KEY		0x0D
#define ADPD188_REG_SW_RESET					0x0F
#define ADPD188_REG_MODE							0x10
#define ADPD188_REG_SLOT_EN						0x11
#define ADPD188_REG_FSAMPLE						0x12
#define ADPD188_REG_PD_LED_SELECT			0x14
#define ADPD188_REG_NUM_AVG						0x15
#define ADPD188_REG_BIG_MEAS_A				0x16
#define ADPD188_REG_INT_SEQ_A					0x17
#define ADPD188_REG_SLOTA_CH1_OFFSET	0x18
#define ADPD188_REG_SLOTA_CH2_OFFSET	0x19
#define ADPD188_REG_SLOTA_CH3_OFFSET	0x1A
#define ADPD188_REG_SLOTA_CH4_OFFSET	0x1B
#define ADPD188_REG_BIG_MEAS_B				0x1C
#define ADPD188_REG_INT_SEQ_B					0x1D
#define ADPD188_REG_SLOTB_CH1_OFFSET	0x1E
#define ADPD188_REG_SLOTB_CH2_OFFSET	0x1F
#define ADPD188_REG_SLOTB_CH3_OFFSET	0x20
#define ADPD188_REG_SLOTB_CH4_OFFSET	0x21
#define ADPD188_REG_ILED3_COARSE			0x22
#define ADPD188_REG_ILED1_COARSE			0x23
#define ADPD188_REG_ILED2_COARSE			0x24
#define ADPD188_REG_ILED_FINE					0x25
#define ADPD188_REG_SLOTA_LED_PULSE		0x30
#define ADPD188_REG_SLOTA_NUM_PULSES	0x31
#define ADPD188_REG_LED_DISABLE				0x34
#define ADPD188_REG_SLOTB_LED_PULSE		0x35
#define ADPD188_REG_SLOTB_NUM_PULSES	0x36
#define ADPD188_REG_ALT_PWR_DN				0x37
#define ADPD188_REG_EXT_SYNC_STARTUP	0x38
#define ADPD188_REG_SLOTA_AFE_WINDOW	0x39
#define ADPD188_REG_SLOTB_AFE_WINDOW	0x3B
#define ADPD188_REG_AFE_PWR_CFG1			0x3C
#define ADPD188_REG_SLOTA_FLOAT_LED		0x3E
#define ADPD188_REG_SLOTB_FLOAT_LED		0x3F
#define ADPD188_REG_SLOTA_TIA_CFG			0x42
#define ADPD188_REG_SLOTA_AFE_CFG			0x43
#define ADPD188_REG_SLOTB_TIA_CFG			0x44
#define ADPD188_REG_SLOTB_AFE_CFG			0x45
#define ADPD188_REG_SAMPLE_CLK				0x4B
#define ADPD188_REG_CLK32M_ADJUST			0x4D
#define ADPD188_REG_EXT_SYNC_SEL			0x4F
#define ADPD188_REG_CLK32M_CAL_EN			0x50
#define ADPD188_REG_AFE_PWR_CFG2			0x54
#define ADPD188_REG_TIA_INDEP_GAIN		0x55
#define ADPD188_REG_MATH							0x58
#define ADPD188_REG_FLT_CONFIG_B			0x59
#define ADPD188_REG_FLT_LED_FIRE			0x5A
#define ADPD188_REG_FLT_CONFIG_A			0x5E
#define ADPD188_REG_DATA_ACCESS_CTL		0x5F
#define ADPD188_REG_FIFO_ACCESS				0x60
#define ADPD188_REG_SLOTA_CH1					0x64
#define ADPD188_REG_SLOTA_CH2					0x65
#define ADPD188_REG_SLOTA_CH3					0x66
#define ADPD188_REG_SLOTA_CH4					0x67
#define ADPD188_REG_SLOTB_CH1					0x68
#define ADPD188_REG_SLOTB_CH2					0x69
#define ADPD188_REG_SLOTB_CH3					0x6A
#define ADPD188_REG_SLOTB_CH4					0x6B
#define ADPD188_REG_A_CH1_LOW					0x70
#define ADPD188_REG_A_CH2_LOW					0x71
#define ADPD188_REG_A_CH3_LOW					0x72
#define ADPD188_REG_A_CH4_LOW					0x73
#define ADPD188_REG_A_CH1_HIGH				0x74
#define ADPD188_REG_A_CH2_HIGH				0x75
#define ADPD188_REG_A_CH3_HIGH				0x76
#define ADPD188_REG_A_CH4_HIGH				0x77
#define ADPD188_REG_B_CH1_LOW					0x78
#define ADPD188_REG_B_CH2_LOW					0x79
#define ADPD188_REG_B_CH3_LOW					0x7A
#define ADPD188_REG_B_CH4_LOW					0x7B
#define ADPD188_REG_B_CH1_HIGH				0x7C
#define ADPD188_REG_B_CH2_HIGH				0x7D
#define ADPD188_REG_B_CH3_HIGH				0x7E
#define ADPD188_REG_B_CH4_HIGH				0x7F

/*  */
#define ADPD188_READ_CMD							0
#define ADPD188_WRITE_CMD							1

/* Exported typedef ----------------------------------------------------------*/
/**
 * @brief Smoke 2 description setting.
 */
typedef enum {
	ADPD188_SLOT_A = 0,
	ADPD188_SLOT_B = 4
} adpd188_slot_e;

/**
 * @brief Smoke 2 description setting to select channel.
 */
typedef enum {
	ADPD188_CH_1 = 0,
	ADPD188_CH_2,
	ADPD188_CH_3,
	ADPD188_CH_4
} adpd188_channel_e;

/**
 * @brief Smoke 2 description setting to select mode.
 */
typedef enum {
	ADPD188_MODE_IDLE = 0,
	ADPD188_MODE_PROGRAM,
	ADPD188_MODE_NORMAL
} adpd188_mode_e;

/**
 * @brief Smoke 2 description setting to select mode.
 */
typedef enum {
	ADPD188_SMOKE_ERROR = 0x0,
	ADPD188_SMOKE_DETECTED = 0xCC,
	ADPD188_SMOKE_NOT_DETECTED = 0xDD
} adpd188_smoke_e;

typedef struct {
	i2c_bus_dev_t *i2c_dev;
	int int_gpio;
	uint16_t calib_value;
	uint16_t threshold_value;
	uint8_t enabled_slot;
} adpd188_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a ADPD188 instance
 *
 * @param me       : Pointer to a adpd188_t instance
 * @param i2c_bus  : Pointer to a structure with the data to initialize the
 * 								   I2C device
 * @param dev_addr : I2C device address
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_init(adpd188_t *const me, i2c_bus_t *i2c_bus,
		uint8_t dev_addr, int int_gpio);

/**
 * @brief Function to set the working mode of the ADPD188
 *
 * @param me   : Pointer to a adpd188_t instance
 * @param mode : Working mode: IDLE, PROGRAM and NORMAL.
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_set_mode(adpd188_t *const me, adpd188_mode_e mode);

/**
 * @brief Function to set the working mode of the ADPD188
 *
 * @param me   : Pointer to a adpd188_t instance
 * @param mode : Working mode: IDLE, PROGRAM and NORMAL.
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_get_mode(adpd188_t *const me, adpd188_mode_e *mode);

/**
 * @brief Function to perform a software reset
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_soft_reset(adpd188_t *const me);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_get_int(adpd188_t *const me, uint8_t *fifo, uint8_t *slot_a,
		uint8_t *slot_b);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_read_sens_data(adpd188_t *const me, uint8_t slot, uint8_t ch,
		uint16_t *data);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_set_bit(adpd188_t *const me, uint8_t reg_addr, uint8_t bit_num,
		bool bit_val);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_set_bit_mask(adpd188_t *const me, uint8_t reg_addr,
		uint8_t bits_num,	uint16_t bits_val);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_calibration(adpd188_t *const me, uint16_t threshold);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
uint16_t adpd188_get_calib(adpd188_t *const me);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
esp_err_t adpd188_check_smoke(adpd188_t *const me, adpd188_smoke_e *smoke);

/**
 * @brief Function to perform a software reset todo: write correctly
 *
 * @param me : Pointer to a adpd188_t instance
 *
 * @return ESP_OK on success
 */
int adpd188_get_int_gpio(adpd188_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* ADPD188_H_ */

/***************************** END OF FILE ************************************/
