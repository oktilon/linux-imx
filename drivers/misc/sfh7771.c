// SPDX-License-Identifier: GPL-2.0-only
/*
 * @brief This file is part of the OSRAM SFH7771 sensor driver.
 * @details Chip is combined proximity and ambient light sensor.
 *
 * @copyright (C) 2023 Lanars Company
 *
 * @author Anton Kalistratov <work111acc@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define SFH7771_NAME			"sfh7771"

#define SFH7771_STARTUP_DELAY	50
#define SFH7771_RESET_TIME		10

#define SFH7771_SYSTEM_CONTROL		0x40 //! R/W  System Control
#define SFH7771_MODE_CONTROL 		0x41 //! R/W ALS and PS General Control
#define SFH7771_ALS_PS_CONTROL 		0x42 //! R/W ALS Gain and PS current Control
#define SFH7771_PERSISTENCE 		0x43 //! R/W PS Interrupt Persistence Control
#define SFH7771_PS_DATA_LSB 		0x44 //! R Output data of PS measurement, LSB
#define SFH7771_PS_DATA_MSB 		0x45 //! R Output data of PS measurement, MSB
#define SFH7771_ALS_VIS_DATA_LSB 	0x46 //! R Output data of ALS_VIS measurement, LSB
#define SFH7771_ALS_VIS_DATA_MSB 	0x47 //! R Output data of ALS_VIS measurement, MSB
#define SFH7771_ALS_IR_DATA_LSB 	0x48 //! R Output data of ALS_IR measurement, LSB
#define SFH7771_ALS_IR_DATA_MSB 	0x49 //! R Output data of ALS_IR measurement, MSB
#define SFH7771_INTERRUPT_CONTROL 	0x4A //! R/W Interrupt Control
#define SFH7771_PS_TH_LSB 			0x4B //! R/W PS interrupt upper threshold level, LSB
#define SFH7771_PS_TH_MSB 			0x4C //! R/W PS interrupt upper threshold level, MSB
#define SFH7771_PS_TL_LSB 			0x4D //! R/W PS interrupt lower threshold level, LSB
#define SFH7771_PS_TL_MSB 			0x4E //! R/W PS interrupt lower threshold level, MSB
#define SFH7771_ALS_VIS_TH_LSB 		0x4F //! R/W ALS_VIS interrupt upper threshold level, LSB
#define SFH7771_ALS_VIS_TH_MSB 		0x50 //! R/W ALS_VIS interrupt upper threshold level, MSB
#define SFH7771_ALS_VIS_TL_LSB 		0x51 //! R/W ALS_VIS interrupt lower threshold level, LSB
#define SFH7771_ALS_VIS_TL_MSB 		0x52 //! R/W ALS_VIS interrupt lower threshold level, MSB

//! System Control
#define SFH7771_PART_ID				0x01 //! Read Only
#define SFH7771_MANUFACT_ID			0x01 //! Read Only
#define SFH7771_PART_ID_MASK		0x07
#define SFH7771_MANUFACT_ID_MASK	0x38
#define SFH7771_INT_RESET			(1 << 6)
#define SFH7771_SW_RESET			(1 << 7)

//! ALS and PS General Control
#define SFH7771_SENS_MODE_MASK		0x0F
#define SFH7771_PS_MODE_MASK		0x80

typedef enum {								//! Repetition / Integration time (ALS)	Repetition time (PS)
	SFH7771_SENS_MODE_STANDBY,				//! standby								standby
	SFH7771_SENS_MODE_STANDBY_10MS,			//!	standby								10 ms
	SFH7771_SENS_MODE_STANDBY_40MS,			//!	standby								40 ms
	SFH7771_SENS_MODE_STANDBY_100MS,		//!	standby								100 ms
	SFH7771_SENS_MODE_STANDBY_400MS,		//!	standby								400 ms
	SFH7771_SENS_MODE_100_100MS_STANDBY,	//!	100 ms / 100 ms						standby
	SFH7771_SENS_MODE_100_100MS_100MS,		//!	100 ms / 100 ms						100 ms
	SFH7771_SENS_MODE_100_100MS_400MS,		//!	100 ms / 100 ms						400 ms
	SFH7771_SENS_MODE_400_100MS_STANDBY,	//!	400 ms / 100 ms						standby
	SFH7771_SENS_MODE_400_100MS_100MS,		//!	400 ms / 100 ms						100 ms
	SFH7771_SENS_MODE_400_400MS_STANDBY,	//!	400 ms / 400 ms						standby
	SFH7771_SENS_MODE_400_400MS_400MS,		//!	400 ms / 400 ms						400 ms
	SFH7771_SENS_MODE_50_50MS_50MS			//!	50 ms / 50 ms						50 ms
} SFH7771_SENS_MODE_T;

typedef enum {
	SFH7771_PS_MODE_NORMAL,		//!	Only one PS measurement is performed during one PS repetition time
	SFH7771_PS_MODE_TWICE		//!	Two independent PS measurement are performed within one PS repetition time
} SFH7771_PS_MODE_T;

//! ALS Gain and PS current Control
#define SFH7771_LED_CURRENT_MASK	0x03
#define SFH7771_ALS_GAIN_MASK		0x3C
#define SFH7771_PS_OUTPUT_MASK		0x40

typedef enum {
	SFH7771_LED_CURRENT_25MA,
	SFH7771_LED_CURRENT_50MA,
	SFH7771_LED_CURRENT_100MA,
	SFH7771_LED_CURRENT_200MA
} SFH7771_LED_CURRENT_T;

typedef enum {							//! ALS_VIS Gain	ALS_IR Gain
	SFH7771_ALS_GAIN_X1_X1,				//! X1				X1
	SFH7771_ALS_GAIN_X2_X1 = 0x04,		//! X2				X1
	SFH7771_ALS_GAIN_X2_X2,				//! X2				X2
	SFH7771_ALS_GAIN_X64_X64 = 0x0A,	//! X64				X64
	SFH7771_ALS_GAIN_X128_X64 = 0x0E,	//! X128			X64
	SFH7771_ALS_GAIN_X128_X128			//! X128			X128
} SFH7771_ALS_GAIN_T;

typedef enum {
	SFH7771_PROXIMITY_OUTPUT,
	SFH7771_INFRARED_DC_LEVEL_OUTPUT
} SFH7771_PS_OUTPUT_T;

//! Interrupt Persistence Control
#define SFH7771_PERSISTENCE_MASK	0x0F

typedef enum {
	SFH7771_PERSISTENCE_ACTIVE_AFTER_EACH,	/* Interrupt becomes active after each measurement
											(The mode indicates that a PS or ALS measurement has been finished and
											can be read via the register. It is independent of the ALS & PS measurement
											value and threshold settings) */
	SFH7771_PERSISTENCE_UPD_AFTER_EACH,		/* Interrupt status is updated after each measurement
											(The interrupt status is updated independently after each measurement.
											Active or Inactive status of the interrupt is depending on the values of the last
											measurement in combination with the interrupt settings: “interrupt mode”
											(register 0x4A) and “thresholds” register 0x4C and following.) */
	SFH7771_PERSISTENCE_UPD_ON2THR,			/* Interrupt status is updated if two consecutive threshold judgement are the same
											(The interrupt status only changes if the interrupt judgement of 2 consecutive
											measurement results are the same and different to the current interrupt status.) */

	/* SFH7771_PERSISTENCE_UPD_ON_N_THR 	0011 … 1111 Interrupt status is updated if threshold judgement are the
											same over consecutive set times (3 …15)
											(This is the same procedure like in the 0010 persistence mode, but instead
											of 2 consecutive threshold judgments more are needed (3 to 15 depending
											on the setting) to change the interrupt status.)
											e.g.:
											1010: 10 measurement results in a row need to fulfill the interrupt judgement
											to update the interrupt status */
} SFH7771_PERSISTENCE_T;

//! Interrupt Control
#define SFH7771_INT_TRIG_MASK		0x03
#define SFH7771_INT_LATCH_MASK		0x04
#define SFH7771_INT_ASSERT_MASK		0x08
#define SFH7771_PS_INT_MODE_MASK	0x30
#define SFH7771_ALS_INT_STATUS_MASK	0x40
#define SFH7771_PS_INT_STATUS_MASK	0x80

typedef enum {
	SFH7771_INT_TRIG_INACTIVE,
	SFH7771_INT_TRIG_PS,
	SFH7771_INT_TRIG_ALS,
	SFH7771_INT_TRIG_PS_ALS
} SFH7771_INT_TRIG_T;

typedef enum {
	SFH7771_INT_LATCHED,			//! INT is latched until INT registers is read or initialize
	SFH7771_INT_NON_LATCHED			//! INT is updated after each measurement
} SFH7771_INT_LATCH_T;

typedef enum {
	SFH7771_INT_ASSERT_STABLE,		//! INT “L” is stable if newer measurement results is also interrupt active
	SFH7771_INT_REASSERT			//! INT “L” is de-assert and re-assert if newer measurement results is also interrupt active
} SFH7771_INT_ASSERT_T;

typedef enum {
	SFH7771_PS_INT_MODE_ACTIVE,				//! PS_TH (PS high threshold 0x4B & 0x4C) is only active
		//! The INT state is active it the PS measurement result is equal or higher than the set PS_TH high threshold.
		//! The INT state is inactive, if the PS measurement result is lower than the set PS_TH high threshold.
	SFH7771_PS_INT_MODE_ACTIVE_HYSTERESIS,	//! PS_TH & PS_TL (PS high & low threshold) are active as hysteresis
		//! If the PS measurement signal is higher than the PS high threshold
		//! (PS_TH) the INT state is switched to active. If the PS measurement signal is lower than the PS low threshold (PS_TL)
		//! the INT state is inactive. If once interrupt signal becomes active, INT status is kept active until measurement result
		//! becomes less than PS_TL register value.
	SFH7771_PS_INT_MODE_ACTIVE_OUTSIDE_DET	//! PS_TH & PS_TL (PS high & low threshold) are active as outside detection
		//! In case of “PS outside detection” mode interrupt signal inactive means that measurement result is within registered
		//! threshold level and interrupt signal active means measurement result is out of registered threshold level.
} SFH7771_PS_INT_T;

struct sfh7771_chip {
	struct i2c_client		*client;
	struct mutex			mutex;

	unsigned char manufacturer;
	unsigned char part;

	SFH7771_SENS_MODE_T sens_mode;
	SFH7771_PS_MODE_T ps_mode;

	SFH7771_LED_CURRENT_T led_current;
	SFH7771_ALS_GAIN_T als_gain;
	SFH7771_PS_OUTPUT_T ps_ir_out;

	SFH7771_PERSISTENCE_T persistence;

	SFH7771_INT_TRIG_T int_trig;
	SFH7771_INT_LATCH_T int_latch;
	SFH7771_INT_ASSERT_T int_assert;
	SFH7771_PS_INT_T ps_int;

	unsigned char int_status;
	unsigned short ps_threshold_high;
	unsigned short ps_threshold_low;
	unsigned short als_threshold_high;
	unsigned short als_threshold_low;

	unsigned short ps_raw;
	unsigned short als_vis_raw;
	unsigned short als_ir_raw;

	bool irq_en;
};

static int __ps_raw_read(struct sfh7771_chip *chip)
{
	int ret;
	unsigned char block_data[2];
	struct i2c_client *client = chip->client;

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_read_i2c_block_data(client, SFH7771_PS_DATA_LSB, 2, block_data);
	mutex_unlock(&chip->mutex);
	if (ret >= 0) {
		chip->ps_raw = block_data[0];
		chip->ps_raw |= block_data[1] << 8;
		chip->int_status &= ~SFH7771_PS_INT_STATUS_MASK;
	}

	return ret;
}

static int __als_raw_read(struct sfh7771_chip *chip)
{
	int ret;
	unsigned char block_data[4];
	struct i2c_client *client = chip->client;

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_read_i2c_block_data(client, SFH7771_ALS_VIS_DATA_LSB, 4, block_data);
	mutex_unlock(&chip->mutex);
	if (ret >= 0) {
		chip->als_vis_raw = block_data[0];
		chip->als_vis_raw |= block_data[1] << 8;
		chip->als_ir_raw = block_data[2];
		chip->als_ir_raw |= block_data[3] << 8;
		chip->int_status &= ~SFH7771_ALS_INT_STATUS_MASK;
	}

	return ret;
}

static ssize_t ps_raw_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);

	if ((!chip->irq_en) || (chip->int_status & SFH7771_ALS_INT_STATUS_MASK)) {
		__ps_raw_read(chip);
	}

	return sprintf(buf, "%u\n", chip->ps_raw);
}
static DEVICE_ATTR_RO(ps_raw);

static ssize_t als_vis_raw_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);

	if ((!chip->irq_en) || (chip->int_status & SFH7771_ALS_INT_STATUS_MASK)) {
		__als_raw_read(chip);
	}

	return sprintf(buf, "%u\n", chip->als_vis_raw);
}
static DEVICE_ATTR_RO(als_vis_raw);

static ssize_t als_ir_raw_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);

	if ((!chip->irq_en) || (chip->int_status & SFH7771_ALS_INT_STATUS_MASK)) {
		__als_raw_read(chip);
	}

	return sprintf(buf, "%u\n", chip->als_ir_raw);
}
static DEVICE_ATTR_RO(als_ir_raw);

static ssize_t als_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%u %u\n", chip->als_threshold_low, chip->als_threshold_high);
}

static ssize_t als_threshold_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	unsigned short threshold_low, threshold_high;
	unsigned char block_data[4];
	int ret;

	ret = sscanf(buf, "%hu %hu", &threshold_low, &threshold_high);
	if (ret < 2) return -EINVAL;
	block_data[0] = threshold_high & 0xFF;
	block_data[1] = (threshold_high >> 8) & 0xFF;
	block_data[2] = threshold_low & 0xFF;
	block_data[3] = (threshold_low >> 8) & 0xFF;

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_i2c_block_data(chip->client,
					SFH7771_ALS_VIS_TH_LSB, 4, block_data);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting ALS threshold failed\n");
		return -EAGAIN;
	}

	chip->als_threshold_low = threshold_low;
	chip->als_threshold_high = threshold_high;
	return count;
}
static DEVICE_ATTR_RW(als_threshold);

static ssize_t ps_threshold_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%u %u\n", chip->ps_threshold_low, chip->ps_threshold_high);
}

static ssize_t ps_threshold_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	unsigned short threshold_low, threshold_high;
	unsigned char block_data[4];
	int ret;

	ret = sscanf(buf, "%hu %hu", &threshold_low, &threshold_high);
	if (ret < 2) return -EINVAL;
	block_data[0] = threshold_high & 0xFF;
	block_data[1] = (threshold_high >> 8) & 0xFF;
	block_data[2] = threshold_low & 0xFF;
	block_data[3] = (threshold_low >> 8) & 0xFF;

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_i2c_block_data(chip->client,
					SFH7771_PS_TH_LSB, 4, block_data);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting PS threshold failed\n");
		return -EAGAIN;
	}

	chip->ps_threshold_low = threshold_low;
	chip->ps_threshold_high = threshold_high;
	return count;
}
static DEVICE_ATTR_RW(ps_threshold);

static ssize_t persistence_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->persistence);
}

static ssize_t persistence_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	int persistence;
	int ret;

	ret = kstrtoint(buf, 0, &persistence);
	if (ret) return ret;

	if (persistence < SFH7771_PERSISTENCE_ACTIVE_AFTER_EACH || persistence > 15) {
		return -EINVAL;
	}

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_byte_data(chip->client,
					SFH7771_PERSISTENCE, persistence);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting persistence failed\n");
		return -EAGAIN;
	}

	chip->persistence = persistence;
	return count;
}
static DEVICE_ATTR_RW(persistence);

static ssize_t persistence_avail_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "[%d] Interrupt becomes active after each measurement\n"
						"[%d] Interrupt status is updated after each measurement\n"
						"[2 .. 15] Interrupt status is updated if two consecutive threshold judgement are the same\n",
				(int) SFH7771_PERSISTENCE_ACTIVE_AFTER_EACH,
				(int) SFH7771_PERSISTENCE_UPD_AFTER_EACH);
}
static DEVICE_ATTR_RO(persistence_avail);

static ssize_t als_gain_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_gain);
}

static ssize_t als_gain_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	int als_gain;
	int ret;

	ret = kstrtoint(buf, 0, &als_gain);
	if (ret) return ret;

	if (als_gain != SFH7771_ALS_GAIN_X1_X1 &&
		als_gain != SFH7771_ALS_GAIN_X2_X1 &&
		als_gain != SFH7771_ALS_GAIN_X2_X2 &&
		als_gain != SFH7771_ALS_GAIN_X64_X64 &&
		als_gain != SFH7771_ALS_GAIN_X128_X64 &&
		als_gain != SFH7771_ALS_GAIN_X128_X128) {
		return -EINVAL;
	}

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_byte_data(chip->client,
					SFH7771_ALS_PS_CONTROL, (chip->ps_ir_out << 6) | (als_gain << 2) | chip->led_current);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting ALS gain failed\n");
		return -EAGAIN;
	}

	chip->als_gain = als_gain;
	return count;
}
static DEVICE_ATTR_RW(als_gain);

static ssize_t als_gain_avail_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "<  > ALS_VIS Gain  ALS_IR Gain\n"
						"[%2d] X1            X1\n"
						"[%2d] X2            X1\n"
						"[%2d] X2            X2\n"
						"[%2d] X64           X64\n"
						"[%2d] X128          X64\n"
						"[%2d] X128          X128\n",
				SFH7771_ALS_GAIN_X1_X1,
				SFH7771_ALS_GAIN_X2_X1,
				SFH7771_ALS_GAIN_X2_X2,
				SFH7771_ALS_GAIN_X64_X64,
				SFH7771_ALS_GAIN_X128_X64,
				SFH7771_ALS_GAIN_X128_X128);
}
static DEVICE_ATTR_RO(als_gain_avail);

static ssize_t ps_rate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->ps_mode);
}

static ssize_t ps_rate_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	int ps_op_mode;
	int ret;

	ret = kstrtoint(buf, 0, &ps_op_mode);
	if (ret) return ret;

	if (ps_op_mode < SFH7771_PS_MODE_NORMAL || ps_op_mode > SFH7771_PS_MODE_TWICE) {
		return -EINVAL;
	}

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_byte_data(chip->client,
					SFH7771_MODE_CONTROL, (ps_op_mode << 4) | chip->sens_mode);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting PS rate failed\n");
		return -EAGAIN;
	}
	chip->ps_mode = ps_op_mode;

	return count;
}
static DEVICE_ATTR_RW(ps_rate);

static ssize_t ps_rate_avail_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "[%d] - Only one PS measurement is performed during one PS repetition time\n"
						"[%d] - Two independent PS measurement are performed within one PS repetition time\n",
						SFH7771_PS_MODE_NORMAL, SFH7771_PS_MODE_TWICE);
}
static DEVICE_ATTR_RO(ps_rate_avail);

static ssize_t rate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->sens_mode);
}

static ssize_t rate_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	int sens_mode;
	int ret;

	ret = kstrtoint(buf, 0, &sens_mode);
	if (ret) return ret;

	if (sens_mode < SFH7771_SENS_MODE_STANDBY || sens_mode > SFH7771_SENS_MODE_50_50MS_50MS) {
		return -EINVAL;
	}

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_byte_data(chip->client,
					SFH7771_MODE_CONTROL, (chip->ps_mode << 4) | sens_mode);
	mutex_unlock(&chip->mutex);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Setting ALS & PS rate failed\n");
		return -EAGAIN;
	}
	chip->sens_mode = sens_mode;

	return count;
}
static DEVICE_ATTR_RW(rate);

static ssize_t rate_avail_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "<  > Repetition / Integration time (ALS)  Repetition time (PS)\n"
						"[%2d] standby                              standby\n"
						"[%2d] standby                              10 ms\n"
						"[%2d] standby                              40 ms\n"
						"[%2d] standby                              100 ms\n"
						"[%2d] standby                              400 ms\n"
						"[%2d] 100 ms / 100 ms                      standby\n"
						"[%2d] 100 ms / 100 ms                      100 ms\n"
						"[%2d] 100 ms / 100 ms                      400 ms\n"
						"[%2d] 400 ms / 100 ms                      standby\n"
						"[%2d] 400 ms / 100 ms                      100 ms\n"
						"[%2d] 400 ms / 400 ms                      standby\n"
						"[%2d] 400 ms / 400 ms                      400 ms\n"
						"[%2d] 50 ms / 50 ms                        50 ms\n",
				SFH7771_SENS_MODE_STANDBY,
				SFH7771_SENS_MODE_STANDBY_10MS,
				SFH7771_SENS_MODE_STANDBY_40MS,
				SFH7771_SENS_MODE_STANDBY_100MS,
				SFH7771_SENS_MODE_STANDBY_400MS,
				SFH7771_SENS_MODE_100_100MS_STANDBY,
				SFH7771_SENS_MODE_100_100MS_100MS,
				SFH7771_SENS_MODE_100_100MS_400MS,
				SFH7771_SENS_MODE_400_100MS_STANDBY,
				SFH7771_SENS_MODE_400_100MS_100MS,
				SFH7771_SENS_MODE_400_400MS_STANDBY,
				SFH7771_SENS_MODE_400_400MS_400MS,
				SFH7771_SENS_MODE_50_50MS_50MS);
}
static DEVICE_ATTR_RO(rate_avail);

static ssize_t chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sfh7771_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "Manufacturer %s [%d] part [%d]\n",
		(chip->manufacturer == SFH7771_MANUFACT_ID) ? "OSRAM" : "Unknown",
		chip->manufacturer, chip->part);
}
static DEVICE_ATTR_RO(chip_id);

static struct attribute *sfh7771_attribute_attrs[] = {
	&dev_attr_als_ir_raw.attr,
	&dev_attr_als_vis_raw.attr,
	&dev_attr_ps_raw.attr,
	&dev_attr_als_threshold.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_persistence.attr,
	&dev_attr_persistence_avail.attr,
	&dev_attr_als_gain.attr,
	&dev_attr_als_gain_avail.attr,
	&dev_attr_ps_rate.attr,
	&dev_attr_ps_rate_avail.attr,
	&dev_attr_rate.attr,
	&dev_attr_rate_avail.attr,
	&dev_attr_chip_id.attr,
	NULL
};
ATTRIBUTE_GROUPS(sfh7771_attribute);

static irqreturn_t sfh7771_irq(int irq, void *data)
{
	struct sfh7771_chip *chip = data;
	char *envp[2];
	int status;

	mutex_lock(&chip->mutex);
	status = i2c_smbus_read_byte_data(chip->client, SFH7771_INTERRUPT_CONTROL);
	mutex_unlock(&chip->mutex);
	if (status < 0) return IRQ_HANDLED;
	chip->int_status = status & (SFH7771_ALS_INT_STATUS_MASK | SFH7771_PS_INT_STATUS_MASK);

	envp[1] = NULL;
	if (status & SFH7771_ALS_INT_STATUS_MASK) {
		//! Notify to read ALS data
		envp[0] = "ALS and IR threshold reached";
		kobject_uevent_env(&chip->client->dev.kobj, KOBJ_CHANGE, envp);
		sysfs_notify(&chip->client->dev.kobj, NULL, "als_vis_raw");
		sysfs_notify(&chip->client->dev.kobj, NULL, "als_ir_raw");
	}
	if (status & SFH7771_PS_INT_STATUS_MASK) {
		//! Notify to read PS data
		envp[0] = "PS threshold reached";
		kobject_uevent_env(&chip->client->dev.kobj, KOBJ_CHANGE, envp);
		sysfs_notify(&chip->client->dev.kobj, NULL, "ps_raw");
	}

	return IRQ_HANDLED;
}

static int sfh7771_detect(struct sfh7771_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;

	ret = i2c_smbus_read_byte_data(client, SFH7771_SYSTEM_CONTROL);
	if (ret < 0) goto error;

	chip->manufacturer = (ret & SFH7771_MANUFACT_ID_MASK) >> 3;
	chip->part = ret & SFH7771_PART_ID_MASK;
	if ((chip->manufacturer == SFH7771_MANUFACT_ID) &&
		(chip->part == SFH7771_PART_ID)) {
		dev_info(&client->dev, "SFH7771 chip found\n");
		return 0;
	}

error:
	dev_err(&client->dev, "SFH7771 chip not found\n");
	return -ENODEV;
}

static int sfh7771_chip_on(struct sfh7771_chip *chip)
{
	int ret;

	//! Reset the chip
	ret = i2c_smbus_write_byte_data(chip->client, SFH7771_SYSTEM_CONTROL, SFH7771_SW_RESET | SFH7771_INT_RESET);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Chip reset failed\n");
		return -EAGAIN;
	}

	ret = i2c_smbus_write_byte_data(chip->client, SFH7771_INTERRUPT_CONTROL,
			(SFH7771_PS_INT_MODE_ACTIVE << 4) | (SFH7771_INT_REASSERT << 3) | (SFH7771_INT_NON_LATCHED << 2) | SFH7771_INT_TRIG_PS_ALS);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Interrupt settings failed\n");
		return -EAGAIN;
	}

	usleep_range(SFH7771_RESET_TIME, SFH7771_RESET_TIME * 2);

	//! Default values on reset
	chip->sens_mode = SFH7771_SENS_MODE_STANDBY;
	chip->ps_mode = SFH7771_PS_MODE_NORMAL;
	chip->led_current = SFH7771_LED_CURRENT_200MA;
	chip->als_gain = SFH7771_ALS_GAIN_X1_X1;
	chip->ps_ir_out = SFH7771_PROXIMITY_OUTPUT;
	chip->persistence = SFH7771_PERSISTENCE_UPD_AFTER_EACH;
	chip->ps_threshold_low = 0x00;
	chip->ps_threshold_high = 0xFFF;
	chip->als_threshold_low = 0x00;
	chip->als_threshold_high = 0xFFFF;
	chip->int_trig = SFH7771_INT_TRIG_PS_ALS;
	chip->int_latch = SFH7771_INT_NON_LATCHED;
	chip->int_assert = SFH7771_INT_REASSERT;
	chip->ps_int = SFH7771_PS_INT_MODE_ACTIVE;

	return ret;
}

static void sfh7771_chip_off(struct sfh7771_chip *chip)
{
	int ret;

	mutex_lock(&chip->mutex);
	ret = i2c_smbus_write_byte_data(chip->client,
					SFH7771_INTERRUPT_CONTROL, SFH7771_INT_TRIG_INACTIVE);
	if (ret < 0) goto __err;
	ret = i2c_smbus_write_byte_data(chip->client,
				SFH7771_MODE_CONTROL, (SFH7771_PS_MODE_NORMAL << 4) | SFH7771_SENS_MODE_STANDBY);
	if (ret < 0) goto __err;
	mutex_unlock(&chip->mutex);

	chip->sens_mode = SFH7771_SENS_MODE_STANDBY;
	chip->ps_mode = SFH7771_PS_MODE_NORMAL;
	return;
__err:
	mutex_unlock(&chip->mutex);
	dev_err(&chip->client->dev, "Chip off failed\n");
	return;
}

static int sfh7771_remove(struct i2c_client *client)
{
	struct sfh7771_chip *chip = i2c_get_clientdata(client);

	sfh7771_chip_off(chip);

	return 0;
}

static int sfh7771_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sfh7771_chip *chip;
	int err;

	chip = devm_kzalloc(&client->dev, sizeof *chip, GFP_KERNEL);
	if (!chip) {
		dev_err(&chip->client->dev, "No memory for Kernel\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, chip);
	chip->client  = client;
	mutex_init(&chip->mutex);

	usleep_range(SFH7771_STARTUP_DELAY, SFH7771_STARTUP_DELAY * 2);
	err = sfh7771_detect(chip);
	if (err < 0) {
		dev_err(&chip->client->dev, "SFH7771 detection failed\n");
		return err;
	}

	//! Start chip
	err = sfh7771_chip_on(chip);
	if (err < 0) return err;

	//! Interrupt configuration
	err = request_threaded_irq(client->irq, NULL, sfh7771_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					SFH7771_NAME, chip);
	if (err) {
		dev_err(&chip->client->dev, "Could not get IRQ %d ret -> %d\n",
			chip->client->irq, err);
	}

	chip->irq_en = !err;

	return err;
}

static const struct of_device_id sfh7771_of_match[] = {
	{ .compatible = "osram," SFH7771_NAME,},
	{ }
};
MODULE_DEVICE_TABLE(of, sfh7771_of_match);

static const struct i2c_device_id sfh7771_id[] = {
	{SFH7771_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, sfh7771_id);

static struct i2c_driver sfh7771_driver = {
	.driver	 = {
		.name	= SFH7771_NAME,
		.of_match_table = sfh7771_of_match,
		.dev_groups = sfh7771_attribute_groups,
	},
	.probe	  = sfh7771_probe,
	.remove	  = sfh7771_remove,
	.id_table = sfh7771_id,
};

module_i2c_driver(sfh7771_driver);

MODULE_DESCRIPTION("SFH7771 combined ALS and proximity sensor");
MODULE_AUTHOR("Anton Kalistratov, Lanars Company");
MODULE_LICENSE("GPL v2");