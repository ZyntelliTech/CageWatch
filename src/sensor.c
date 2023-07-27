#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include "sensor.h"

int check_i2c_device( const struct i2c_dt_spec* dev)
{
	if (i2c_is_ready_dt( dev )){
		printk("%02X I2C is ready\n", dev->addr);

		return 0;
	}
	else{
		printk("%02X I2C is failed\n", dev->addr);

		return -EFAULT;
	}
}

int tca9548a_set_channel(const struct i2c_dt_spec *spec, uint8_t channel)
{
	uint8_t tx_buf = (1 << channel);

	return i2c_write(spec->bus, &tx_buf, 1, spec->addr);
}

uint8_t vl6180_read8(const struct i2c_dt_spec* dev, uint16_t reg_addr)
{
	uint8_t buf[2];
	buf[0] = (reg_addr >> 8) & 0xff;
	buf[1] = (reg_addr & 0xff);
	i2c_write_read(dev->bus, VL6180X_DEFAULT_I2C_ADDR, buf, 2, buf, 1);

	return buf[0];
}

uint16_t vl6180_read16(const struct i2c_dt_spec* dev, uint16_t reg_addr)
{
	uint8_t buf[2];
	uint16_t ret;
	buf[0] = (reg_addr >> 8) & 0xff;
	buf[1] = (reg_addr & 0xff);
	i2c_write_read(dev->bus, VL6180X_DEFAULT_I2C_ADDR, buf, 2, buf, 2);

	ret = buf[0];
	ret = (ret << 8) + buf[1];
	return ret;
}

int vl6180_write8(const struct i2c_dt_spec* dev, uint16_t reg_addr, uint8_t dat)
{
	uint8_t buf[3];
	buf[0] = (reg_addr >> 8) & 0xff;
	buf[1] = reg_addr & 0xff;
	buf[2] = dat;
	return i2c_write(dev->bus, buf, 3, VL6180X_DEFAULT_I2C_ADDR );
}

int vl6180_write16(const struct i2c_dt_spec* dev, uint16_t reg_addr, uint16_t dat)
{
	uint8_t buf[4];
	buf[0] = (reg_addr >> 8) & 0xff;
	buf[1] = reg_addr & 0xff;
	buf[2] = (dat >> 8) & 0xff;
	buf[3] = dat & 0xff;

	return i2c_write(dev->bus, buf, 4, VL6180X_DEFAULT_I2C_ADDR );
}

int loadSettings(const struct i2c_dt_spec* dev)
{
	// load settings!

  // private settings from page 24 of app note
  vl6180_write8(dev, 0x0207, 0x01);
  vl6180_write8(dev, 0x0208, 0x01);
  vl6180_write8(dev, 0x0096, 0x00);
  vl6180_write8(dev, 0x0097, 0xfd);
  vl6180_write8(dev, 0x00e3, 0x00);
  vl6180_write8(dev, 0x00e4, 0x04);
  vl6180_write8(dev, 0x00e5, 0x02);
  vl6180_write8(dev, 0x00e6, 0x01);
  vl6180_write8(dev, 0x00e7, 0x03);
  vl6180_write8(dev, 0x00f5, 0x02);
  vl6180_write8(dev, 0x00d9, 0x05);
  vl6180_write8(dev, 0x00db, 0xce);
  vl6180_write8(dev, 0x00dc, 0x03);
  vl6180_write8(dev, 0x00dd, 0xf8);
  vl6180_write8(dev, 0x009f, 0x00);
  vl6180_write8(dev, 0x00a3, 0x3c);
  vl6180_write8(dev, 0x00b7, 0x00);
  vl6180_write8(dev, 0x00bb, 0x3c);
  vl6180_write8(dev, 0x00b2, 0x09);
  vl6180_write8(dev, 0x00ca, 0x09);
  vl6180_write8(dev, 0x0198, 0x01);
  vl6180_write8(dev, 0x01b0, 0x17);
  vl6180_write8(dev, 0x01ad, 0x00);
  vl6180_write8(dev, 0x00ff, 0x05);
  vl6180_write8(dev, 0x0100, 0x05);
  vl6180_write8(dev, 0x0199, 0x05);
  vl6180_write8(dev, 0x01a6, 0x1b);
  vl6180_write8(dev, 0x01ac, 0x3e);
  vl6180_write8(dev, 0x01a7, 0x1f);
  vl6180_write8(dev, 0x0030, 0x00);

  // Recommended : Public registers - See data sheet for more detail
  vl6180_write8(dev, 0x0011, 0x10); // Enables polling for 'New Sample ready'
                        // when measurement completes
  vl6180_write8(dev, 0x010a, 0x30); // Set the averaging sample period
                        // (compromise between lower noise and
                        // increased execution time)
  vl6180_write8(dev, 0x003f, 0x46); // Sets the light and dark gain (upper
                        // nibble). Dark gain should not be
                        // changed.
  vl6180_write8(dev, 0x0031, 0xFF); // sets the # of range measurements after
                        // which auto calibration of system is
                        // performed
  vl6180_write8(dev, 0x0041, 0x63); // Set ALS integration time to 100ms
  vl6180_write8(dev, 0x002e, 0x01); // perform a single temperature calibration
                        // of the ranging sensor

  // Optional: Public registers - See data sheet for more detail
  vl6180_write8(dev, SYSRANGE__INTERMEASUREMENT_PERIOD,
         0x09);         // Set default ranging inter-measurement
                        // period to 100ms
  vl6180_write8(dev, 0x003e, 0x31); // Set default ALS inter-measurement period
                        // to 500ms
  vl6180_write8(dev, 0x0014, 0x24); // Configures interrupt on 'New Sample
                        // Ready threshold event'
}

float vl6180_readLux(const struct i2c_dt_spec* dev, uint8_t gain)
{
  uint8_t reg;

  reg = vl6180_read8(dev, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG);
  reg &= ~0x38;
  reg |= (0x4 << 3); // IRQ on ALS ready
  vl6180_write8(dev, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg);

  // 100 ms integration period
  vl6180_write8(dev, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0);
  vl6180_write8(dev, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100);

  // analog gain
  if (gain > VL6180X_ALS_GAIN_40) {
    gain = VL6180X_ALS_GAIN_40;
  }
  vl6180_write8(dev, VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain);

  // start ALS
  vl6180_write8(dev, VL6180X_REG_SYSALS_START, 0x1);

  // Poll until "New Sample Ready threshold event" is set
  while (4 != ((vl6180_read8(dev, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7))
    ;

  // read lux!
  float lux = vl6180_read16(dev, VL6180X_REG_RESULT_ALS_VAL);

  // clear interrupt
  vl6180_write8(dev, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  lux *= 0.32; // calibrated count/lux
  switch (gain) {
  case VL6180X_ALS_GAIN_1:
    break;
  case VL6180X_ALS_GAIN_1_25:
    lux /= 1.25;
    break;
  case VL6180X_ALS_GAIN_1_67:
    lux /= 1.67;
    break;
  case VL6180X_ALS_GAIN_2_5:
    lux /= 2.5;
    break;
  case VL6180X_ALS_GAIN_5:
    lux /= 5;
    break;
  case VL6180X_ALS_GAIN_10:
    lux /= 10;
    break;
  case VL6180X_ALS_GAIN_20:
    lux /= 20;
    break;
  case VL6180X_ALS_GAIN_40:
    lux /= 40;
    break;
  }
  lux *= 100;
  lux /= 100; // integration time in ms

  return lux;
}

uint8_t vl6180_readRange(const struct i2c_dt_spec* dev) 
{
  // wait for device to be ready for range measurement
  while (!(vl6180_read8(dev, VL6180X_REG_RESULT_RANGE_STATUS) & 0x01))
    ;

  // Start a range measurement
  vl6180_write8(dev, VL6180X_REG_SYSRANGE_START, 0x01);

  // Poll until bit 2 is set
  while (!(vl6180_read8(dev, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04))
    ;

  // read range in mm
  uint8_t range = vl6180_read8(dev, VL6180X_REG_RESULT_RANGE_VAL);

  // clear interrupt
  vl6180_write8(dev, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

  return range;
}

