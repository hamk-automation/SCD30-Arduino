/*

*/

#include "Arduino.h"
#include <Wire.h>

// the i2c address
#define SCD30_I2CADDR_DEFAULT 0x61     ///< SCD30 has only one I2C address

// commands and constants
#define SCD30_CRC8_POLYNOMIAL  0x31    ///< Seed for Scd30's CRC polynomial
#define SCD30_CRC8_INIT        0xFF    ///< Init value for CRC
#define SCD30_WORD_LEN         2       ///< 2 bytes per word

/**
 * 
 * 
 * 
 * 
 * 
 * 
 * */
class SCD30 {
 public:
  SCD30();
  boolean begin(TwoWire *theWire = NULL);
  float fTemp, fRH, fCO2;
  boolean bReady;

  boolean trigSingleMeasurement(uint16_t ambientPressure = 0);
  boolean trigContinuousMeasurement(uint16_t ambientPressure = 0);
  boolean stopContinuousMeasurement();
  boolean setMeasurementInterval(uint16_t uInterval = 2);
  boolean getDataReadyStatus();
  boolean readMeasurement();
  boolean setTemperatureOffset(uint16_t tempOffset);
  boolean setAltitudeOffset(uint16_t altitude);

  boolean setASC(boolean enable = true);
  boolean setFRCValue(uint16_t co2baseline);

 private:
  TwoWire *_i2c;
  uint8_t _i2caddr;

  void write(uint8_t address, uint8_t *data, uint8_t n);
  void read(uint8_t address, uint8_t *data, uint8_t n);
  boolean readWordFromCommand(uint8_t command[], uint8_t commandLength, uint16_t delayms = 0, uint16_t *readdata = NULL, uint8_t readlen = 0);
  uint8_t generateCRC(uint8_t data[], uint8_t datalen);
  boolean setParamByCommand(uint8_t command[], uint16_t param, uint8_t param_length = 1);
};
