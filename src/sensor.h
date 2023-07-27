#ifndef SENSOR_H_
#define SENSOR_H_

#include <zephyr/drivers/i2c.h>
#include "vl6180_registers.h"

int check_i2c_device( const struct i2c_dt_spec* dev);
int tca9548a_set_channel(const struct i2c_dt_spec *spec, uint8_t channel);
uint8_t vl6180_read8(const struct i2c_dt_spec* dev, uint16_t reg_addr);
uint16_t vl6180_read16(const struct i2c_dt_spec* dev, uint16_t reg_addr);
int vl6180_write8(const struct i2c_dt_spec* dev, uint16_t reg_addr, uint8_t dat);
int vl6180_write16(const struct i2c_dt_spec* dev, uint16_t reg_addr, uint16_t dat);
int loadSettings(const struct i2c_dt_spec* dev);
float vl6180_readLux(const struct i2c_dt_spec* dev, uint8_t gain);
uint8_t vl6180_readRange(const struct i2c_dt_spec* dev);
#endif