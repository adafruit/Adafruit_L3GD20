/*************************************************** 
  This is an example for the Adafruit Triple-Axis Gyro sensor

  Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032

  These sensors use I2C or SPI to communicate, 2 pins (I2C) 
  or 4 pins (SPI) are required to interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h> 
#include "Adafruit_STM_Gyro.h"
#include <Adafruit_Sensor.h>

// Comment this next line to use SPI
#define USE_I2C

#ifdef USE_I2C
  // The default constructor uses I2C
  Adafruit_STM_Gyro gyro;
#else
  // To use SPI, you have to define the pins
  #define GYRO_CS 4 // labeled CS
  #define GYRO_DO 5 // labeled SA0
  #define GYRO_DI 6  // labeled SDA
  #define GYRO_CLK 7 // labeled SCL
  Adafruit_STM_Gyro gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif

void setup() 
{
  Serial.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
   if (!gyro.begin(gyro.GYRO_RANGE_250DPS))
  //if (!gyro.begin(gyro.GYRO_RANGE_500DPS))
  //if (!gyro.begin(gyro.GYRO_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the gyro. Check your wiring!");
    while (1);
  }
}

void loop() 
{
  sensors_event_t event;
  gyro.getEvent(&event);
  Serial.print("X: "); Serial.print((int)event.gyro.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)event.gyro.y);   Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)event.gyro.z); Serial.print(" ");
  delay(100);
}
