/*!
 * @file SCD30.cpp
 *
 * @mainpage Adafruit SCD30 gas sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's SCD30 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit SCD30 breakout: http://www.adafruit.com/products/3709
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * @section author Author
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 * BSD license, all text here must be included in any redistribution.
 *
 */

/*
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
*/

#include "SCD30.h"

#define I2C_DEBUG

/**************************************************************************/
/*! 
    @brief  Instantiates a new SCD30 class
*/
/**************************************************************************/
SCD30::SCD30() {
}

/**************************************************************************/
/*! 
    @brief  Setups the hardware and detects a valid SCD30. Initializes I2C
    then reads the serialnumber and checks that we are talking to an SCD30
    @param  theWire Optional pointer to I2C interface, otherwise use Wire
    @returns True if SCD30 found on I2C, False if something went wrong!
*/
/**************************************************************************/
boolean SCD30::begin(TwoWire *theWire) {
  _i2caddr = SCD30_I2CADDR_DEFAULT;
  if (theWire == NULL) {
    _i2c = &Wire;
  } else {
    _i2c = theWire;
  }

  _i2c->begin();

  return true;
}



/**************************************************************************/
/*! 
    @brief  I2C low level interfacing
*/
/**************************************************************************/


boolean SCD30::readWordFromCommand(uint8_t command[], uint8_t commandLength, 
                                  uint16_t delayms = 0, uint16_t *readdata, uint8_t readlen)
{
  uint8_t data;

  _i2c->beginTransmission(_i2caddr);

#ifdef I2C_DEBUG
  Serial.print("\t\t-> ");
#endif

  for (uint8_t i=0; i<commandLength; i++) {
    _i2c->write(command[i]);
#ifdef I2C_DEBUG
    Serial.print("0x"); Serial.print(command[i], HEX); Serial.print(", ");
#endif
  }
#ifdef I2C_DEBUG
  Serial.println();
#endif
  _i2c->endTransmission();
  
  if (readlen == 0) 
    return true;

  uint8_t replylen = readlen * (SCD30_WORD_LEN +1);
  if (_i2c->requestFrom(_i2caddr, replylen) != replylen) 
    return false;
  uint8_t replybuffer[replylen];
#ifdef I2C_DEBUG
  Serial.print("\t\t<- ");
#endif  
  for (uint8_t i=0; i<replylen; i++) {
    replybuffer[i] = _i2c->read();
#ifdef I2C_DEBUG
    Serial.print("0x"); Serial.print(replybuffer[i], HEX); Serial.print(", ");
#endif
  }

#ifdef I2C_DEBUG
  Serial.println();
#endif

  for (uint8_t i=0; i<readlen; i++) {
    uint8_t crc = generateCRC(replybuffer+i*3, 2);
#ifdef I2C_DEBUG
    Serial.print("\t\tCRC calced: 0x"); Serial.print(crc, HEX);
    Serial.print(" vs. 0x"); Serial.println(replybuffer[i * 3 + 2], HEX);
#endif
    if (crc != replybuffer[i * 3 + 2])
      return false;
    // success! store it
    readdata[i] = replybuffer[i*3];
    readdata[i] <<= 8;
    readdata[i] |= replybuffer[i*3 + 1];
#ifdef I2C_DEBUG
    Serial.print("\t\tRead: 0x"); Serial.println(readdata[i], HEX);
#endif
  }
  return true;
}

uint8_t SCD30::generateCRC(uint8_t *data, uint8_t datalen) {
  // calculates 8-Bit checksum with given polynomial
  uint8_t crc = SCD30_CRC8_INIT;

  for (uint8_t i=0; i<datalen; i++) {
    crc ^= data[i];
    for (uint8_t b=0; b<8; b++) {
      if (crc & 0x80)
	crc = (crc << 1) ^ SCD30_CRC8_POLYNOMIAL;
      else
	crc <<= 1;
    }
  }
  return crc;
}

boolean SCD30::readMeasurement() {
  // TODO: add dataready check
  // TOREMEMBER: readWordFromCommand already handled byteswap at word level. Only apply wordswap for conv. 
  uint8_t command[2] = {0x03, 0x00};
  uint16_t resp16[6];
  if (readWordFromCommand(command, sizeof(command), 0, (uint16_t*)resp16, 6)) {
    uint16_t tempArray[] = {resp16[3], resp16[2]}; 
    uint16_t rhArray[] = {resp16[5], resp16[4]};
    uint16_t co2Array[] = {resp16[1], resp16[0]};
    float *t, *rh, *co2;
    t = (float*)tempArray;
    rh = (float*)rhArray;
    co2 = (float*)co2Array;
    fTemp = *t;
    fRH = *rh;
    fCO2 = *co2;
    return true;
  }
  else return false;
}
