#ifndef SLF3S_4000B_H
#define SLF3S_4000B_H

#define TRUE  1
#define FALSE 0

uint8_t init_slf3s();
uint8_t read_slf3s();
uint8_t read_info_slf3s();
float convert_and_scale(uint8_t msb, uint8_t lsb, float scale_factor);
uint8_t soft_reset_slf3s();
uint8_t start_meas_slf3s();
uint8_t i2c_read(uint8_t addr, uint8_t* data, uint8_t byte);
uint8_t i2c_write(uint8_t addr, const uint8_t* data, uint8_t byte);
uint8_t validate_crc(uint8_t* data, uint8_t line);
uint8_t crc8(const uint8_t* data, uint8_t len);

#endif