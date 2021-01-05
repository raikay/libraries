#if defined(ARDUINO) && ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define HTU3X_ADDR 0x44  //Unshifted 7-bit I2C address for the sensor

#define ERROR_I2C_TIMEOUT 	2
#define ERROR_BAD_CRC		    3

#define CMD_MEAS_PERI_2_H   0x2236
#define CMD_MEAS_CLOCKSTR_H 0x2C06

class HTU3X {

public:
  HTU3X(uint8_t addr = HTU3X_ADDR);

  //Public Functions
  void begin(TwoWire &wirePort = Wire); //If user doesn't specificy then Wire will be used
  byte readTempAndHumi(float *temp, float *humi);

  //Public Variables

private:
  //Private Functions
  TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware

  byte checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor);
  byte readValue(uint16_t *res);
  
  uint8_t m_addr;

  //Private Variables

};
