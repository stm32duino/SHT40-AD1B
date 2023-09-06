# SHT40-AD1B
Arduino library to support the SHT40-AD1B digital humidity and temperature sensor

## API

This sensor uses I2C to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    SHT40AD1BSensor sensor(&Wire);

The access to the sensor values is done as explained below:  

  Read temperature and humidity.

    float tmp,hum;
    sensor.GetTemperature(&tmp);
    sensor.GetHumidity(&hum);

## Examples

* SHT40AD1B_DataLog_Terminal: This application shows how to get data from SHT40-AD1B humidity and temperature and print them on terminal.

## Documentation

You can find the source files at  
https://github.com/stm32duino/SHT40-AD1B
