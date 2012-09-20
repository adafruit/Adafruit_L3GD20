#include <Wire.h>
#include <Adafruit_L3GD20.h>

Adafruit_L3GD20 gyro;

void setup() 
{
  Serial.begin(9600);
  
  // Make sure I2C is properly setup
  Wire.begin();

  // Try to initialise and warn if we couldn't detect the chip
  // if (!gyro.init(gyro.L3DS20_RANGE_250DPS))
  // if (!gyro.init(gyro.L3DS20_RANGE_500DPS))
  if (!gyro.init(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
}

void loop() 
{
  gyro.read();
  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.print(" ");
  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.print(" ");
  Serial.print("Z: "); Serial.println((int)gyro.data.z); Serial.print(" ");
  delay(100);
}
