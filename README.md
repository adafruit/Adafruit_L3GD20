This is a library for the Adafruit Triple-Axis Gyro sensor

Designed specifically to work with the Adafruit L3GD20 Breakout 
  ----> https://www.adafruit.com/products/1032

These sensors use I2C or SPI to communicate, 2 pins (I2C) or 4 pins (SPI) 
are required to interface.

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Check out the links above for our tutorials and wiring diagrams 

Written by Kevin Townsend for Adafruit Industries.  
BSD license, all text above must be included in any redistribution

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder Adafruit_L3GD20. Check that the Adafruit_L3GD20 folder contains Adafruit_L3GD20.cpp and Adafruit_L3GD20.h

Place the Adafruit_L3GD20 library folder your (arduinosketchfolder)/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

--------------------------------------------------------------------------------
2013-05-10 Mark Ruys added support for L3G4200D. The L3G4200D is register
compatible with a L3GD20. To use a L3G4200D, initialize the gyroscoop with address L3DS20_ADDRESS_L3G4200D (L3DS20_ prefix to comply to library naming):

    gyro.begin(gyro.L3DS20_RANGE_250DPS, gyro.L3DS20_ADDRESS_L3G4200D)

