/*!
 * @file SCD30.cpp
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

//#define I2C_DEBUG

/**************************************************************************/
/*!
    @brief  Instantiates a new SCD30 class
*/
/**************************************************************************/
SCD30::SCD30() {}

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
                                   uint16_t delayms, uint16_t *readdata,
                                   uint8_t readlen) {
  uint8_t data;

  _i2c->beginTransmission(_i2caddr);

#ifdef I2C_DEBUG
  Serial.print("\t\t-> ");
#endif

  for (uint8_t i = 0; i < commandLength; i++) {
    _i2c->write(command[i]);
#ifdef I2C_DEBUG
    Serial.print("0x");
    Serial.print(command[i], HEX);
    Serial.print(", ");
#endif
  }
#ifdef I2C_DEBUG
  Serial.println();
#endif
  _i2c->endTransmission();

  if (readlen == 0 || readdata == nullptr)
    return true;

  uint8_t replylen = readlen * (SCD30_WORD_LEN + 1);
  if (_i2c->requestFrom(_i2caddr, replylen) != replylen)
    return false;
  uint8_t replybuffer[replylen];
#ifdef I2C_DEBUG
  Serial.print("\t\t<- ");
#endif
  for (uint8_t i = 0; i < replylen; i++) {
    replybuffer[i] = _i2c->read();
#ifdef I2C_DEBUG
    Serial.print("0x");
    Serial.print(replybuffer[i], HEX);
    Serial.print(", ");
#endif
  }

#ifdef I2C_DEBUG
  Serial.println();
#endif

  for (uint8_t i = 0; i < readlen; i++) {
    uint8_t crc = generateCRC(replybuffer + i * 3, 2);
#ifdef I2C_DEBUG
    Serial.print("\t\tCRC calced: 0x");
    Serial.print(crc, HEX);
    Serial.print(" vs. 0x");
    Serial.println(replybuffer[i * 3 + 2], HEX);
#endif
    if (crc != replybuffer[i * 3 + 2])
      return false;
    // success! store it
    readdata[i] = replybuffer[i * 3];
    readdata[i] <<= 8;
    readdata[i] |= replybuffer[i * 3 + 1];
#ifdef I2C_DEBUG
    Serial.print("\t\tRead: 0x");
    Serial.println(readdata[i], HEX);
#endif
  }
  return true;
}

/**
 * @brief
 *
 * @param data
 * @param datalen
 * @return uint8_t
 */
uint8_t SCD30::generateCRC(uint8_t *data, uint8_t datalen) {
  // calculates 8-Bit checksum with given polynomial
  uint8_t crc = SCD30_CRC8_INIT;

  for (uint8_t i = 0; i < datalen; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ SCD30_CRC8_POLYNOMIAL;
      else
        crc <<= 1;
    }
  }
  return crc;
}

/**
 * @brief
 *
 * @return boolean
 */
boolean SCD30::readMeasurement() {
  // TODO: add dataready check
  // TOREMEMBER: readWordFromCommand already handled byteswap at word level.
  // Only apply wordswap for conv.
  uint8_t command[2] = {0x03, 0x00};
  uint16_t resp16[6];
  if (readWordFromCommand(command, sizeof(command), 0, (uint16_t *)resp16, 6)) {
    uint16_t tempArray[] = {resp16[3], resp16[2]};
    uint16_t rhArray[] = {resp16[5], resp16[4]};
    uint16_t co2Array[] = {resp16[1], resp16[0]};
    float *t, *rh, *co2;
    t = (float *)tempArray;
    rh = (float *)rhArray;
    co2 = (float *)co2Array;
    fTemp = *t;
    fRH = *rh;
    fCO2 = *co2;
    return true;
  } else
    return false;
}

/**
 * @brief
 *
 * @param ambientPressure
 * @return boolean
 */
boolean SCD30::trigSingleMeasurement(uint16_t ambientPressure) {
  uint8_t command[2] = {0x00, 0x06};
  if (ambientPressure < 700 || ambientPressure > 1200) {
    ambientPressure = 0;
  }
  return setParamByCommand(command, ambientPressure);
}

/**
 * @brief
 *
 * @param ambientPressure
 * @return boolean
 */
boolean SCD30::trigContinuousMeasurement(uint16_t ambientPressure) {
  uint8_t command[2] = {0x00, 0x10};
  if (ambientPressure < 700 || ambientPressure > 1200) {
    ambientPressure = 0;
  }
  return setParamByCommand(command, ambientPressure);
}

/**
 * @brief
 *
 * @return boolean
 */
boolean SCD30::stopContinuousMeasurement() {
  uint8_t command[2] = {0x00, 0x06};
  return setParamByCommand(command, 0, 0);
}

/**
 * @brief
 *
 * @param uInterval
 * @return boolean
 */
boolean SCD30::setMeasurementInterval(uint16_t uInterval) {
  uint8_t command[2] = {0x46, 0x00};
  if (uInterval < 2 || uInterval > 1200) {
    uInterval = 2;
  }
  return setParamByCommand(command, uInterval);
}

/**
 * @brief
 *
 * @return boolean
 */
boolean SCD30::getDataReadyStatus() {
  uint8_t command[2] = {0x02, 0x02};
  uint16_t resp16[1];
  if (readWordFromCommand(command, sizeof(command), 0, (uint16_t *)resp16, 1)) {
    bReady = (resp16[0] == 1);
    return true;
  } else
    return false;
}

/**
 * @brief
 *
 * @param tempOffset
 * @return boolean
 */
boolean SCD30::setTemperatureOffset(uint16_t tempOffset) {
  uint8_t command[2] = {0x54, 0x03};
  return setParamByCommand(command, tempOffset);
}

/**
 * @brief
 *
 * @param altitude
 * @return boolean
 */
boolean SCD30::setAltitudeOffset(uint16_t altitude) {
  uint8_t command[2] = {0x51, 0x02};
  return setParamByCommand(command, altitude);
}

/**
 * @brief
 *
 * @param enable
 * @return boolean
 */
boolean SCD30::setASC(boolean enable) {
  uint8_t command[2] = {0x53, 0x06};
  uint16_t enableCmd = enable == true ? 1 : 0;
  return setParamByCommand(command, enableCmd);
}

/**
 * @brief
 *
 * @param co2baseline
 * @return boolean
 */
boolean SCD30::setFRCValue(uint16_t co2baseline) {
  uint8_t command[2] = {0x52, 0x04};

  if (co2baseline < 400 || co2baseline > 2000) {
    return false;
  }
  return setParamByCommand(command, co2baseline);
}

/**
 * @brief
 *
 * @param command
 * @param param
 * @param param_length
 * @return boolean
 */
boolean SCD30::setParamByCommand(uint8_t command[], uint16_t param,
                                 uint8_t param_length) {
  uint8_t __command[5];
  __command[0] = command[0];
  __command[1] = command[1];
  if (param_length == 0) {
    _i2c->beginTransmission(_i2caddr);
    if (_i2c->write(__command, SCD30_WORD_LEN) != SCD30_WORD_LEN)
      return false;
    _i2c->endTransmission();
    return true;
  } else {
    __command[2] = (param >> 8) & 0xFF;
    __command[3] = param & 0xFF;
    __command[4] = generateCRC(&__command[2], SCD30_WORD_LEN);
    _i2c->beginTransmission(_i2caddr);
    if (_i2c->write(__command, sizeof(__command)) != sizeof(__command))
      return false;
    _i2c->endTransmission();
    return true;
  }
}