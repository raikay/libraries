
#include "HTU3X.h"

HTU3X::HTU3X(uint8_t addr)
{
  //Set initial values for private vars
  m_addr = addr;
}

//Begin
/*******************************************************************************************/
//Start I2C communication
void HTU3X::begin(TwoWire &wirePort)
{
  _i2cPort = &wirePort; //Grab which port the user wants us to use
  
  _i2cPort->begin();
}

#define MAX_WAIT 100
#define DELAY_INTERVAL 10
#define MAX_COUNTER (MAX_WAIT/DELAY_INTERVAL)

//read 2-byte value with CRC from the HTU3X
byte HTU3X::readValue(uint16_t *res)
{

  byte msb, lsb, checksum;

  msb = _i2cPort->read();
  lsb = _i2cPort->read();
  checksum = _i2cPort->read();

  uint16_t rawValue = ((uint16_t) msb << 8) | (uint16_t) lsb;

  if (checkCRC(rawValue, checksum) != 0) 
    return (ERROR_BAD_CRC); //Error out

  *res = rawValue;

  return true;
}

byte HTU3X::readTempAndHumi(float *temp, float *humi)
{
    //Request a humidity reading
  _i2cPort->beginTransmission(m_addr);
  uint16_t cmd = CMD_MEAS_CLOCKSTR_H;
  _i2cPort->write(cmd>>8); //Measure value (prefer no hold!)
  _i2cPort->write(cmd);
  _i2cPort->endTransmission();
  
  delay(500);

  _i2cPort->requestFrom(m_addr,6);
  
  uint16_t rawTemp, rawHumi;
  readValue(&rawTemp);
  readValue(&rawHumi);

  _i2cPort->endTransmission();

  *temp = 175.0f * (float)rawTemp / 65535.0f - 45.0f; 
  *humi = 100.0f * (float)rawHumi / 65535.0f; 

}


#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

byte HTU3X::checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  uint8_t bit;        
  uint8_t crc = 0xFF;
  uint8_t byteCtr;   
   uint8_t data[2] = {message_from_sensor>>8, (uint8_t)message_from_sensor};
  for(byteCtr = 0; byteCtr < 2; byteCtr++)
  {
          crc ^= (data[byteCtr]);
          for(bit = 8; bit > 0; --bit)
          {
                  if(crc & 0x80)
                          crc = (crc << 1) ^ 0x131;
                  else           
                          crc = (crc << 1);
          }
  }
   
  if(crc != check_value_from_sensor)
          return 1;
  else                                
          return 0; 
}