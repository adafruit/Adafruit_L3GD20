/***************************************************
  This is a library for the L3GD20 GYROSCOPE

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

#include "Adafruit_STM_Gyro.h"
#include <Adafruit_Sensor.h>

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

Adafruit_STM_Gyro::Adafruit_STM_Gyro(gyro_type my_type, int8_t cs, int8_t miso, int8_t mosi, int8_t clk) {
  this->type = my_type;
  _cs = cs;
  _miso = miso;
  _mosi = mosi;
  _clk = clk;
}

Adafruit_STM_Gyro::Adafruit_STM_Gyro(gyro_type my_type) {
  // use i2c
  this->type = my_type;
  _cs = _mosi = _miso = _clk = -1;
}

bool Adafruit_STM_Gyro::begin(gyroRange_t rng)
{
  if (_cs == -1) {
    Wire.begin();
  } else {
    pinMode(_cs, OUTPUT);
    pinMode(_clk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    digitalWrite(_cs, HIGH);
  }

  range = rng;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
   
  byte whoami = read8(GYRO_REGISTER_WHO_AM_I); 
  if (whoami != this->type.deviceID)
  {
    return false;
  }

  /* Set CTRL_REG1 (0x20)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   7-6  DR1/0     Output data rate                                   00
   5-4  BW1/0     Bandwidth selection                                00
     3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
     2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
     1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
     0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

  /* Switch to normal mode and enable all three channels */
  write8(GYRO_REGISTER_CTRL_REG1, 0x0F);
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG2 (0x21)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
   5-4  HPM1/0    High-pass filter mode selection                    00
   3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG3 (0x22)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
     6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
     5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
     4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
     3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
     2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
     1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
     0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG4 (0x23)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
     6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
   5-4  FS1/0     Full scale selection                               00
                                  00 = 250 dps
                                  01 = 500 dps
                                  10 = 2000 dps
                                  11 = 2000 dps
     0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

  /* Adjust resolution if requested */
  switch(range)
  {
    case GYRO_RANGE_250DPS:
      write8(GYRO_REGISTER_CTRL_REG4, 0x00);
      break;
    case GYRO_RANGE_500DPS:
      write8(GYRO_REGISTER_CTRL_REG4, 0x10);
      break;
    case GYRO_RANGE_2000DPS:
      write8(GYRO_REGISTER_CTRL_REG4, 0x20);
      break;
  }
  /* ------------------------------------------------------------------ */

  /* Set CTRL_REG5 (0x24)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     7  BOOT      Reboot memory content (0=normal, 1=reboot)          0
     6  FIFO_EN   FIFO enable (0=FIFO disable, 1=enable)              0
     4  HPen      High-pass filter enable (0=disable,1=enable)        0
   3-2  INT1_SEL  INT1 Selection config                              00
   1-0  OUT_SEL   Out selection config                               00 */

  /* Nothing to do ... keep default values */
  /* ------------------------------------------------------------------ */

  return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_STM_Gyro::getEvent(sensors_event_t* event)
{
  /* Clear and prepare the event */
  memset(event, 0, sizeof(sensors_event_t)); 
  event->version   = sizeof(sensors_event_t);
  event->sensor_id = 0; // _sensorID;
  event->type      = SENSOR_TYPE_GYROSCOPE;
  event->timestamp = millis();

  uint8_t xhi, xlo, ylo, yhi, zlo, zhi;

  if (_cs == -1) {
    Wire.beginTransmission(this->type.I2CAddress);
    // Make sure to set address auto-increment bit
    Wire.write(GYRO_REGISTER_OUT_X_L | 0x80);
    Wire.endTransmission();
    Wire.requestFrom(this->type.I2CAddress, (byte)6);
    
    // Wait around until enough data is available
    while (Wire.available() < 6);
    
    xlo = Wire.read();
    xhi = Wire.read();
    ylo = Wire.read();
    yhi = Wire.read();
    zlo = Wire.read();
    zhi = Wire.read();

  } else {
    digitalWrite(_clk, HIGH);
    digitalWrite(_cs, LOW);

    SPIxfer(GYRO_REGISTER_OUT_X_L | 0x80 | 0x40); // SPI read, autoincrement
    delay(10);
    xlo = SPIxfer(0xFF);
    xhi = SPIxfer(0xFF);
    ylo = SPIxfer(0xFF);
    yhi = SPIxfer(0xFF);
    zlo = SPIxfer(0xFF);
    zhi = SPIxfer(0xFF);

    digitalWrite(_cs, HIGH);
  }
  // Shift values to create properly formed integer (low byte first)
  event->gyro.x = (xlo | (xhi << 8));
  event->gyro.y = (ylo | (yhi << 8));
  event->gyro.z = (zlo | (zhi << 8));
  
  // Compensate values depending on the resolution
  switch(range)
  {
    case GYRO_RANGE_250DPS:
      event->gyro.x *= GYRO_SENSITIVITY_250DPS;
      event->gyro.y *= GYRO_SENSITIVITY_250DPS;
      event->gyro.z *= GYRO_SENSITIVITY_250DPS;
      break;
    case GYRO_RANGE_500DPS:
      event->gyro.x *= GYRO_SENSITIVITY_500DPS;
      event->gyro.y *= GYRO_SENSITIVITY_500DPS;
      event->gyro.z *= GYRO_SENSITIVITY_500DPS;
      break;
    case GYRO_RANGE_2000DPS:
      event->gyro.x *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.y *= GYRO_SENSITIVITY_2000DPS;
      event->gyro.z *= GYRO_SENSITIVITY_2000DPS;
      break;
  }
}
    
void  Adafruit_STM_Gyro::getSensor(sensor_t* sensor)
{
  
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "SensorName", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = this->type.deviceID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = this->type.minDelay;
  sensor->max_value   = (float)this->range;              
  sensor->min_value   = this->range * -1.0;
  sensor->resolution  = this->type.resolution;             
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_STM_Gyro::write8(gyroRegisters_t reg, byte value)
{
  if (_cs == -1) {
    // use i2c
    Wire.beginTransmission(this->type.I2CAddress);
    Wire.write((byte)reg);
    Wire.write(value);
    Wire.endTransmission();
  } else {
    digitalWrite(_clk, HIGH);
    digitalWrite(_cs, LOW);

    SPIxfer(reg);
    SPIxfer(value);

    digitalWrite(_cs, HIGH);
  }
}

byte Adafruit_STM_Gyro::read8(gyroRegisters_t reg)
{
  byte value;

  if (_cs == -1) {
    // use i2c
    Wire.beginTransmission(this->type.I2CAddress);
    Wire.write((byte)reg);
    Wire.endTransmission();
    Wire.requestFrom(this->type.I2CAddress, (byte)1);
    value = Wire.read();
    Wire.endTransmission();
  } else {
    digitalWrite(_clk, HIGH);
    digitalWrite(_cs, LOW);

    SPIxfer((uint8_t)reg | 0x80); // set READ bit
    value = SPIxfer(0xFF);

    digitalWrite(_cs, HIGH);
  }

  return value;
}

uint8_t Adafruit_STM_Gyro::SPIxfer(uint8_t x) {
  uint8_t value = 0;

  for (int i=7; i>=0; i--) {
    digitalWrite(_clk, LOW);
    if (x & (1<<i)) {
      digitalWrite(_mosi, HIGH);
    } else {
      digitalWrite(_mosi, LOW);
      }
    digitalWrite(_clk, HIGH);
    if (digitalRead(_miso))
      value |= (1<<i);
  }

  return value;
}
