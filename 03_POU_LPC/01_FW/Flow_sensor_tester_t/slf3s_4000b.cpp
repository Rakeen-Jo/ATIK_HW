#include <Arduino.h>
#include <Wire.h>
#include "slf3s_4000b.h"

const float    SLF3S_SCALE_FACTOR_FLOW = 500.0;
const float    SLF3S_SCALE_FACTOR_TEMP = 200.0;
const uint8_t  SLF3S_I2C_ADDRESS = 0x08;

const uint8_t  CMD_READ_INFO_LENGTH = 2;
const uint8_t  CMD_READ_INFO_MSB[CMD_READ_INFO_LENGTH] = { 0x36, 0x7C };
const uint8_t  CMD_READ_INFO_LSB[CMD_READ_INFO_LENGTH] = { 0xE1, 0x02 };

const uint8_t  CMD_START_MEASUREMENT_LENGTH = 2;
const uint8_t  CMD_START_MEASUREMENT[CMD_START_MEASUREMENT_LENGTH] = { 0x36, 0x08 };
const uint8_t  INITIAL_MEASURE_DELAY = 50;

const uint8_t  INFO_DATA_LENGTH = 18;
const uint8_t  DATA_LENGTH = 9;

const uint8_t  SOFT_RESET_I2C_ADDRESS = 0x0;
const uint8_t  CMD_SOFT_RESET_LENGTH = 1;
const uint8_t  CMD_SOFT_RESET[CMD_SOFT_RESET_LENGTH] = { 0x06 };
const uint8_t  SOFT_RESET_MAX_TRIES = 10;

const uint8_t  CHIP_RESET_DELAY = 100;
const uint16_t CHIP_RESET_RETRY_DELAY = 500;

uint32_t  product_num;
uint64_t  serial_num;

uint16_t  value_flow;
uint16_t  value_temp;
uint16_t  sig_flag;

uint8_t init_i2c()
{
  Wire.begin();
  
  if (soft_reset_slf3s() != TRUE)
    return FALSE;

  return TRUE;
}

uint8_t read_slf3s()
{
  uint8_t data[DATA_LENGTH] = {0};

  if (i2c_read(SLF3S_I2C_ADDRESS, data, DATA_LENGTH) != TRUE)
    return FALSE;
  
  if (validate_crc(data, 3) != TRUE)
    return 2;

  value_flow = convert_and_scale(data[0], data[1], SLF3S_SCALE_FACTOR_FLOW);
  value_temp = convert_and_scale(data[3], data[4], SLF3S_SCALE_FACTOR_TEMP);
  sig_flag   = (uint16_t)((data[6] << 8) | data[7]);

  return TRUE; 
}

float convert_and_scale(uint8_t msb, uint8_t lsb, float scale_factor)
{
  return (float)((uint16_t)((msb << 8) | lsb)) / scale_factor;
}

uint8_t soft_reset_slf3s()
{
  uint8_t count =0;

  while (i2c_write(SOFT_RESET_I2C_ADDRESS, CMD_SOFT_RESET, CMD_SOFT_RESET_LENGTH) != TRUE)
  {
    Serial.println("Error proceed soft reset, retrying...");
    delay(CHIP_RESET_RETRY_DELAY);
    ++count;

    if (count > SOFT_RESET_MAX_TRIES)
      return FALSE;
  }

  delay(CHIP_RESET_DELAY);

  return TRUE;
}

uint8_t read_info_slf3s()
{
  uint8_t data[INFO_DATA_LENGTH] = {0};

  if (i2c_write(SLF3S_I2C_ADDRESS, CMD_READ_INFO_MSB, CMD_READ_INFO_LENGTH) != TRUE)
    return FALSE;
  else
  {
    if (i2c_write(SLF3S_I2C_ADDRESS, CMD_READ_INFO_LSB, CMD_READ_INFO_LENGTH) != TRUE)
      return FALSE;    
  }

  if (i2c_read(SLF3S_I2C_ADDRESS, data, INFO_DATA_LENGTH) != TRUE)
    return 2;

  if (validate_crc(data, 6) != TRUE)
    return 3;

  product_num = (uint32_t) ((data[0] << 24) | (data[1] << 16) | (data[3] << 8 ) | data[4]);
  serial_num  = (uint64_t) ((data[6] << 56) | (data[7] << 48) | (data[9] << 40) | (data[10] << 32) | (data[12] << 24) | (data[13] << 16) | (data[15] << 8) | data[16]);

  return TRUE;
}

uint8_t start_meas_slf3s()
{
  if (i2c_write(SLF3S_I2C_ADDRESS, CMD_START_MEASUREMENT, CMD_START_MEASUREMENT_LENGTH) != TRUE)
    return FALSE;
  
  delay(INITIAL_MEASURE_DELAY);
  return TRUE;
}

uint8_t i2c_read(uint8_t addr, uint8_t* data, uint8_t byte)
{
  Wire.requestFrom(addr, byte);

  if (Wire.available() != byte)
    return FALSE;

  for (int i = 0; i < byte; i ++)
    data[i] = Wire.read();

  return TRUE;
}

uint8_t i2c_write(uint8_t addr, const uint8_t* data, uint8_t byte)
{
  Wire.beginTransmission(addr);

  for (int i = 0; i < byte; i ++)
  {
    if (Wire.write(data[i]) != TRUE)
      return FALSE;
  }

  if (Wire.endTransmission() != 0)
    return FALSE;

  return TRUE;
}

uint8_t validate_crc(uint8_t* data, uint8_t line)
{
  // the data coming from the sensor is in the following format:
  // [MSB0][LSB0][CRC0][MSB1][LSB1][CRC1]...[MSBx][LSBx][CRCx]
  // in this function, we verify 'word_count' MSB+LSB pairs

  static const uint8_t DATA_SIZE = 2; //
  static const uint8_t STEP_SIZE = DATA_SIZE + 1; // 2 data bytes + crc

  for (int i = 0; i < line; ++i) 
  {
    uint8_t pos = i * STEP_SIZE;

    if (crc8(data + pos, DATA_SIZE) != data[pos + DATA_SIZE])
      return FALSE;
  }

  return TRUE;
}

uint8_t crc8(const uint8_t* data, uint8_t len)
{
  // adapted from SHT21 sample code from http://www.sensirion.com/en/products/humidity-temperature/download-center/
  uint8_t crc = 0xff;
  uint8_t byteCtr;

  for (byteCtr = 0; byteCtr < len; ++byteCtr) 
  {
    crc ^= (data[byteCtr]);

    for (uint8_t bit = 8; bit > 0; --bit) 
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc = (crc << 1);
    }
  }

  return crc;
}