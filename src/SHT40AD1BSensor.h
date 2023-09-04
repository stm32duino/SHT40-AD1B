/**
  ******************************************************************************
  * @file    SHT40AD1BSensor.h
  * @author  STMicroelectronics
  * @brief   SHT40AD1B header driver file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2014-2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SHT40AD1B_H
#define SHT40AD1B_H
#define SHT40AD1B_I2C_ADDRESS  0x89U

/* Includes ------------------------------------------------------------------*/
#include "Wire.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

/* Typedefs ------------------------------------------------------------------*/
typedef enum {
  SHT40AD1B_STATUS_OK = 0,
  SHT40AD1B_STATUS_ERROR
} SHT40AD1BStatusTypeDef;

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an SHT40AD1B Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class SHT40AD1BSensor {
  public:
    SHT40AD1BSensor(TwoWire *i2c);

    SHT40AD1BStatusTypeDef GetHumidity(float *hum_value);
    SHT40AD1BStatusTypeDef GetTemperature(float *tmp_value);

  private:
    uint8_t crc_calculate(const uint8_t *data, uint16_t count);
    uint8_t crc_check(const uint8_t *data, uint16_t count, uint8_t crc);
    int32_t data_get(float_t *buffer);
    /* Helper classes. */
    TwoWire *dev_i2c;
    /* Configuration */
    uint8_t address;
};


#endif
