/*
   @file    SHT40AD1B_DataLog_Terminal.ino
   @author  STMicroelectronics
   @brief   Example of DataLog Terminal
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

#include <SHT40AD1BSensor.h>
SHT40AD1BSensor sensor(&Wire);
float tmp,hum;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  Serial.println("SHT40AD1B DataLog Terminal");  
}

void loop() {
  Serial.print("[TMP]: ");  
  sensor.GetTemperature(&tmp);
  Serial.print(tmp);
  Serial.print(" C [HUM]: ");  
  sensor.GetHumidity(&hum);
  Serial.print(hum);
  Serial.println(" %");  
}
