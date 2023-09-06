/**
 ******************************************************************************
  * @file    SHT40AD1BSensor.cpp
  * @author  STMicroelectronics
  * @brief   SHT40AD1B driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "SHT40AD1BSensor.h"
/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
SHT40AD1BSensor::SHT40AD1BSensor(TwoWire *i2c) : dev_i2c(i2c)
{
  address = SHT40AD1B_I2C_ADDRESS;
}

/**
  * @brief  Get the SHT40AD1B humidity value
  * @param  hum_value pointer where the humidity value is written
  * @retval 0 in case of success, an error code otherwise
  */
SHT40AD1BStatusTypeDef SHT40AD1BSensor::GetHumidity(float *hum_value)
{
  float data[2];  /* humidity, temperature */
  if (data_get(data) != 0) {
    return SHT40AD1B_STATUS_ERROR;
  }

  *hum_value = data[0];
  return SHT40AD1B_STATUS_OK;
}

/**
  * @brief  Get the SHT40AD1B temperature value
  * @param  temp_value pointer where the temperature value is written
  * @retval 0 in case of success, an error code otherwise
  */
SHT40AD1BStatusTypeDef SHT40AD1BSensor::GetTemperature(float *temp_value)
{
  float data[2];  /* humidity, temperature */
  if (data_get(data) != 0) {
    return SHT40AD1B_STATUS_ERROR;
  }

  *temp_value = data[1];

  return SHT40AD1B_STATUS_OK;
}


/**
  * @brief  Calculate CRC
  *
  * @param  data   data stream bytes
  * @param  count  number of data bytes
  * @retval        CRC check sum of data stream
  *
  */
uint8_t SHT40AD1BSensor::crc_calculate(const uint8_t *data, uint16_t count)
{
  const uint8_t crc8_polynomial = 0x31;
  uint8_t crc = 0xFF;

  /* Calculate 8-bit checksum for given polynomial */
  for (uint16_t index = 0; index < count; index++) {
    crc ^= data[index];
    for (uint8_t crc_bit = 8U; crc_bit > 0U; crc_bit--) {
      crc = ((crc & 0x80U) != 0U) ? ((crc << 1) ^ crc8_polynomial) : (crc << 1);
    }
  }

  return crc;
}

/**
  * @brief  Check CRC
  *
  * @param  data   data stream bytes
  * @param  count  number of data bytes
  * @param  crc    CRC check sum of data stream
  * @retval        0 if CRC is OK else 1
  *
  */
uint8_t SHT40AD1BSensor::crc_check(const uint8_t *data, uint16_t count, uint8_t crc)
{
  return (crc_calculate(data, count) == crc) ? 0U : 1U;
}


/**
  * @brief  Humidity and Temperature output value [get]
  *
  * @param  buffer  buffer to store humidity and temperature values pair
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t SHT40AD1BSensor::data_get(float *buffer)
{
  uint8_t command = 0xFD;  // TODO: Replace value 0xFD with enum value
  uint8_t data[6] = {0};

  dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
  dev_i2c->write(command);
  dev_i2c->endTransmission();

  /* Wait 10 ms */
  delay(10);

  dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), 6);

  int i = 0;
  while (dev_i2c->available()) {
    data[i] = dev_i2c->read();
    i++;
  }

  uint16_t temp_value_raw = (data[0] * 0x100U) + data[1];
  uint8_t temp_value_crc  = data[2];
  uint16_t hum_value_raw  = (data[3] * 0x100U) + data[4];
  uint8_t hum_value_crc   = data[5];

  /* Check CRC for temperature value */
  if (crc_check(&data[0], 2, temp_value_crc) != 0U) {
    return 1;
  }

  /* Check CRC for humidity value */
  if (crc_check(&data[3], 2, hum_value_crc) != 0U) {
    return 1;
  }

  float temp_value = -45.0f + (175.0f * (float)temp_value_raw / (float)0xFFFF);
  float hum_value  =  -6.0f + (125.0f * (float)hum_value_raw  / (float)0xFFFF);

  hum_value = (hum_value > 100.0f) ? 100.0f
              : (hum_value <   0.0f) ?   0.0f
              :                        hum_value;

  buffer[0] = hum_value;
  buffer[1] = temp_value;
  return 0;
}
