// SPDX-License-Identifier: GPL-2.0
/*
 * Awinic AW20036/AW20054/AW20072/AW20108 LED driver
 *
 * Copyright (c) 2023, SberDevices. All Rights Reserved.
 *
 * Author: Martin Kurbanov <mmkurbanov@sberdevices.ru>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/units.h>
#include <linux/workqueue.h>

#define AW200XX_DIM_MAX                  (BIT(6) - 1)
#define AW200XX_FADE_MAX                 (BIT(8) - 1)
#define AW200XX_IMAX_DEFAULT_uA          60000
#define AW200XX_IMAX_MAX_uA              160000
#define AW200XX_IMAX_MIN_uA              3300

/* Page 0 */
#define AW200XX_REG_PAGE0_BASE 0xc000

/* Select page register */
#define AW200XX_REG_PAGE       0xF0
#define AW200XX_PAGE_MASK      (GENMASK(7, 6) | GENMASK(2, 0))
#define AW200XX_PAGE_SHIFT     0
#define AW200XX_NUM_PAGES      6
#define AW200XX_PAGE_SIZE      256
#define AW200XX_REG(page, reg) \
	(AW200XX_REG_PAGE0_BASE + (page) * AW200XX_PAGE_SIZE + (reg))
#define AW200XX_REG_MAX \
	AW200XX_REG(AW200XX_NUM_PAGES - 1, AW200XX_PAGE_SIZE - 1)
#define AW200XX_PAGE0 0
#define AW200XX_PAGE1 1
#define AW200XX_PAGE2 2
#define AW200XX_PAGE3 3
#define AW200XX_PAGE4 4
#define AW200XX_PAGE5 5

/* Chip ID register */
#define AW200XX_REG_IDR       AW200XX_REG(AW200XX_PAGE0, 0x00)
#define AW200XX_IDR_CHIPID    0x18

/* Sleep mode register */
#define AW200XX_REG_SLPCR     AW200XX_REG(AW200XX_PAGE0, 0x01)
#define AW200XX_SLPCR_ACTIVE  0x00

/* Reset register */
#define AW200XX_REG_RSTR      AW200XX_REG(AW200XX_PAGE0, 0x02)
#define AW200XX_RSTR_RESET    0x01

/* Global current configuration register */
#define AW200XX_REG_GCCR        AW200XX_REG(AW200XX_PAGE0, 0x03)
#define AW200XX_GCCR_IMAX_MASK  GENMASK(7, 4)
#define AW200XX_GCCR_IMAX(x)    ((x) << 4)
#define AW200XX_GCCR_ALLON      BIT(3)

/* Fast clear display control register */
#define AW200XX_REG_FCD       AW200XX_REG(AW200XX_PAGE0, 0x04)
#define AW200XX_FCD_CLEAR     0x01

/* Display size configuration */
#define AW200XX_REG_DSIZE          AW200XX_REG(AW200XX_PAGE0, 0x80)
#define AW200XX_DSIZE_COLUMNS_MAX  12

#define AW200XX_LED2REG(x, columns) \
	((x) + (((x) / (columns)) * (AW200XX_DSIZE_COLUMNS_MAX - (columns))))

/* DIM current configuration register on page 1 */
#define AW200XX_REG_DIM_PAGE1(x, columns) \
	AW200XX_REG(AW200XX_PAGE1, AW200XX_LED2REG(x, columns))

/*
 * DIM current configuration register (page 4).
 * The even address for current DIM configuration.
 * The odd address for current FADE configuration
 */
#define AW200XX_REG_DIM(x, columns) \
	AW200XX_REG(AW200XX_PAGE4, AW200XX_LED2REG(x, columns) * 2)
#define AW200XX_REG_DIM2FADE(x) ((x) + 1)
#define AW200XX_REG_FADE2DIM(fade) \
	DIV_ROUND_UP((fade) * AW200XX_DIM_MAX, AW200XX_FADE_MAX)

/*
 * Duty ratio of display scan (see p.15 of datasheet for formula):
 *   duty = (592us / 600.5us) * (1 / (display_rows + 1))
 *
 * Multiply to 1000 (MILLI) to improve the accuracy of calculations.
 */
#define AW200XX_DUTY_RATIO(rows) \
	(((592UL * USEC_PER_SEC) / 600500UL) * (MILLI / (rows)) / MILLI)

struct aw200xx_chipdef {
	u32 channels;
	u32 display_size_rows_max;
	u32 display_size_columns;
};

struct aw200xx_led {
	struct led_classdev cdev;
	struct aw200xx *chip;
	int dim;
	u32 num;
};

enum aw200xx_board_mode {
	AW200XX_BOARD_OFF = 0,
	AW200XX_BOARD_SOLID,
	AW200XX_BOARD_BREATH,
};

enum aw200xx_board_color {
	AW200XX_COLOR_WHITE = 0,
	AW200XX_COLOR_RED,
	AW200XX_COLOR_GREEN,
	AW200XX_COLOR_BLUE,
};

struct aw200xx {
	const struct aw200xx_chipdef *cdef;
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex mutex;
	u32 num_leds;
	u32 display_rows;
	struct gpio_desc *hwen;

	/* Board-level controls (whole ring) */
	enum aw200xx_board_mode board_mode;
	enum aw200xx_board_color board_color;

	/*
	 * board_brightness: runtime brightness currently applied (used by breath).
	 * solid_brightness: user/boot configured "solid target" brightness.
	 * When switching breath -> solid, we restore solid_brightness.
	 */
	u8 board_brightness; /* 0..255 */
	u8 solid_brightness; /* 0..255 */

	u8 breath_step;      /* 1..255 */
	u32 breath_interval_ms;
	bool breath_dir_up;
	struct delayed_work breath_work;

	struct aw200xx_led leds[] __counted_by(num_leds);
};

static ssize_t dim_show(struct device *dev, struct device_attribute *devattr,
			char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw200xx_led *led = container_of(cdev, struct aw200xx_led, cdev);
	int dim = led->dim;

	if (dim < 0)
		return sysfs_emit(buf, "auto\n");

	return sysfs_emit(buf, "%d\n", dim);
}

static ssize_t dim_store(struct device *dev, struct device_attribute *devattr,
			 const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw200xx_led *led = container_of(cdev, struct aw200xx_led, cdev);
	struct aw200xx *chip = led->chip;
	u32 columns = chip->cdef->display_size_columns;
	int dim;
	ssize_t ret;

	if (sysfs_streq(buf, "auto")) {
		dim = -1;
	} else {
		ret = kstrtoint(buf, 0, &dim);
		if (ret)
			return ret;

		if (dim > AW200XX_DIM_MAX)
			return -EINVAL;
	}

	mutex_lock(&chip->mutex);

	if (dim >= 0) {
		ret = regmap_write(chip->regmap,
				   AW200XX_REG_DIM_PAGE1(led->num, columns),
				   dim);
		if (ret)
			goto out_unlock;
	}

	led->dim = dim;
	ret = count;

out_unlock:
	mutex_unlock(&chip->mutex);
	return ret;
}
static DEVICE_ATTR_RW(dim);

static struct attribute *dim_attrs[] = {
	&dev_attr_dim.attr,
	NULL
};
ATTRIBUTE_GROUPS(dim);

static int aw200xx_set_channel_brightness_locked(struct aw200xx *chip, u32 chan,
						 u8 brightness)
{
	u32 columns = chip->cdef->display_size_columns;
	u32 reg = AW200XX_REG_DIM(chan, columns);
	u8 dim = AW200XX_REG_FADE2DIM(brightness);
	int ret;

	ret = regmap_write(chip->regmap, reg, dim);
	if (ret)
		return ret;

	return regmap_write(chip->regmap, AW200XX_REG_DIM2FADE(reg), brightness);
}

static int aw200xx_board_apply_locked(struct aw200xx *chip, u8 brightness)
{
	u32 chan;
	u8 r = 0, g = 0, b = 0;

	switch (chip->board_color) {
	case AW200XX_COLOR_RED:
		r = brightness;
		break;
	case AW200XX_COLOR_GREEN:
		g = brightness;
		break;
	case AW200XX_COLOR_BLUE:
		b = brightness;
		break;
	case AW200XX_COLOR_WHITE:
	default:
		r = g = b = brightness;
		break;
	}

	/*
	 * Board wiring confirmed by you:
	 *   ch(3n+0)=R, ch(3n+1)=G, ch(3n+2)=B
	 */
	for (chan = 0; chan < chip->cdef->channels; chan++) {
		u8 v;

		switch (chan % 3) {
		case 0:
			v = r;
			break;
		case 1:
			v = g;
			break;
		default:
			v = b;
			break;
		}

		/* Ignore errors? No, fail fast. */
		if (aw200xx_set_channel_brightness_locked(chip, chan, v))
			return -EIO;
	}

	return 0;
}

static void aw200xx_breath_workfn(struct work_struct *work)
{
	struct aw200xx *chip =
		container_of(to_delayed_work(work), struct aw200xx, breath_work);
	u8 br;
	u8 step;
	int ret;

	mutex_lock(&chip->mutex);

	if (chip->board_mode != AW200XX_BOARD_BREATH) {
		mutex_unlock(&chip->mutex);
		return;
	}

	br = chip->board_brightness;
	step = chip->breath_step ? chip->breath_step : 5;

	if (chip->breath_dir_up) {
		if (br + step >= 255) {
			br = 255;
			chip->breath_dir_up = false;
		} else {
			br += step;
		}
	} else {
		if (br <= step) {
			br = 0;
			chip->breath_dir_up = true;
		} else {
			br -= step;
		}
	}

	chip->board_brightness = br;
	ret = aw200xx_board_apply_locked(chip, br);

	mutex_unlock(&chip->mutex);

	if (!ret && chip->board_mode == AW200XX_BOARD_BREATH)
		schedule_delayed_work(&chip->breath_work,
				      msecs_to_jiffies(chip->breath_interval_ms));
}

static ssize_t board_brightness_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct aw200xx *chip = dev_get_drvdata(dev);

	/* Expose the "solid target" brightness to userspace */
	return sysfs_emit(buf, "%u\n", chip->solid_brightness);
}

static ssize_t board_brightness_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct aw200xx *chip = dev_get_drvdata(dev);
	unsigned int v;
	int ret;

	ret = kstrtouint(buf, 0, &v);
	if (ret)
		return ret;
	if (v > 255)
		return -EINVAL;

	mutex_lock(&chip->mutex);

	/*
	 * Always update the solid target.
	 * If we're currently in SOLID, apply immediately.
	 * If we're in BREATH, keep breathing based on board_brightness, but the
	 * next switch to SOLID will restore this solid_brightness.
	 */
	chip->solid_brightness = (u8)v;

	if (chip->board_mode == AW200XX_BOARD_SOLID) {
		chip->board_brightness = chip->solid_brightness;
		ret = aw200xx_board_apply_locked(chip, chip->board_brightness);
	} else {
		ret = 0;
	}

	mutex_unlock(&chip->mutex);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(board_brightness);

static ssize_t board_color_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct aw200xx *chip = dev_get_drvdata(dev);

	switch (chip->board_color) {
	case AW200XX_COLOR_RED:
		return sysfs_emit(buf, "red\n");
	case AW200XX_COLOR_GREEN:
		return sysfs_emit(buf, "green\n");
	case AW200XX_COLOR_BLUE:
		return sysfs_emit(buf, "blue\n");
	case AW200XX_COLOR_WHITE:
	default:
		return sysfs_emit(buf, "white\n");
	}
}

static ssize_t board_color_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct aw200xx *chip = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&chip->mutex);

	if (sysfs_streq(buf, "red")) {
		chip->board_color = AW200XX_COLOR_RED;
	} else if (sysfs_streq(buf, "green")) {
		chip->board_color = AW200XX_COLOR_GREEN;
	} else if (sysfs_streq(buf, "blue")) {
		chip->board_color = AW200XX_COLOR_BLUE;
	} else if (sysfs_streq(buf, "white")) {
		chip->board_color = AW200XX_COLOR_WHITE;
	} else {
		ret = -EINVAL;
		goto out;
	}

	if (chip->board_mode == AW200XX_BOARD_SOLID)
		ret = aw200xx_board_apply_locked(chip, chip->board_brightness);

out:
	mutex_unlock(&chip->mutex);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(board_color);

static ssize_t board_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw200xx *chip = dev_get_drvdata(dev);

	switch (chip->board_mode) {
	case AW200XX_BOARD_OFF:
		return sysfs_emit(buf, "off\n");
	case AW200XX_BOARD_BREATH:
		return sysfs_emit(buf, "breath\n");
	case AW200XX_BOARD_SOLID:
	default:
		return sysfs_emit(buf, "solid\n");
	}
}

static ssize_t board_mode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct aw200xx *chip = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&chip->mutex);

	if (sysfs_streq(buf, "off")) {
		chip->board_mode = AW200XX_BOARD_OFF;
		cancel_delayed_work(&chip->breath_work);
		ret = aw200xx_board_apply_locked(chip, 0);
	} else if (sysfs_streq(buf, "solid")) {
		chip->board_mode = AW200XX_BOARD_SOLID;
		cancel_delayed_work(&chip->breath_work);

		/* Restore solid target brightness when leaving breath */
		chip->board_brightness = chip->solid_brightness;
		ret = aw200xx_board_apply_locked(chip, chip->board_brightness);
	} else if (sysfs_streq(buf, "breath")) {
		chip->board_mode = AW200XX_BOARD_BREATH;
		chip->breath_dir_up = true;
		if (!chip->breath_interval_ms)
			chip->breath_interval_ms = 20;
		if (!chip->breath_step)
			chip->breath_step = 5;
		/* start from 0 for visible breathe-in */
		chip->board_brightness = 0;
		ret = aw200xx_board_apply_locked(chip, 0);
		if (!ret)
			schedule_delayed_work(&chip->breath_work,
					      msecs_to_jiffies(chip->breath_interval_ms));
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&chip->mutex);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(board_mode);

static struct attribute *aw200xx_board_attrs[] = {
	&dev_attr_board_mode.attr,
	&dev_attr_board_color.attr,
	&dev_attr_board_brightness.attr,
	NULL,
};
ATTRIBUTE_GROUPS(aw200xx_board);

static int aw200xx_brightness_set(struct led_classdev *cdev,
				  enum led_brightness brightness)
{
	struct aw200xx_led *led = container_of(cdev, struct aw200xx_led, cdev);
	struct aw200xx *chip = led->chip;
	int dim;
	u32 reg;
	int ret;

	mutex_lock(&chip->mutex);

	reg = AW200XX_REG_DIM(led->num, chip->cdef->display_size_columns);

	dim = led->dim;
	if (dim < 0)
		dim = AW200XX_REG_FADE2DIM(brightness);

	ret = regmap_write(chip->regmap, reg, dim);
	if (ret)
		goto out_unlock;

	ret = regmap_write(chip->regmap,
			   AW200XX_REG_DIM2FADE(reg), brightness);

out_unlock:
	mutex_unlock(&chip->mutex);

	return ret;
}

static u32 aw200xx_imax_from_global(const struct aw200xx *const chip,
				    u32 global_imax_uA)
{
	u64 led_imax_uA;

	/*
	 * The output current of each LED (see p.14 of datasheet for formula):
	 *   Iled = Imax * (dim / 63) * ((fade + 1) / 256) * duty
	 *
	 * The value of duty is determined by the following formula:
	 *   duty = (592us / 600.5us) * (1 / (display_rows + 1))
	 *
	 * Calculated for the maximum values of fade and dim.
	 * We divide by 1000 because we earlier multiplied by 1000 to improve
	 * accuracy when calculating the duty.
	 */
	led_imax_uA = global_imax_uA * AW200XX_DUTY_RATIO(chip->display_rows);
	do_div(led_imax_uA, MILLI);

	return led_imax_uA;
}

static u32 aw200xx_imax_to_global(const struct aw200xx *const chip,
				  u32 led_imax_uA)
{
	u32 duty = AW200XX_DUTY_RATIO(chip->display_rows);

	/* The output current of each LED (see p.14 of datasheet for formula) */
	return (led_imax_uA * 1000U) / duty;
}

#define AW200XX_IMAX_MULTIPLIER1    10000
#define AW200XX_IMAX_MULTIPLIER2    3333
#define AW200XX_IMAX_BASE_VAL1      0
#define AW200XX_IMAX_BASE_VAL2      8

/*
 * The AW200XX has a 4-bit register (GCCR) to configure the global current,
 * which ranges from 3.3mA to 160mA. The following table indicates the values
 * of the global current, divided into two parts:
 *
 * +-----------+-----------------+-----------+-----------------+
 * | reg value | global max (mA) | reg value | global max (mA) |
 * +-----------+-----------------+-----------+-----------------+
 * | 0         | 10              | 8         | 3.3             |
 * | 1         | 20              | 9         | 6.7             |
 * | 2         | 30              | 10        | 10              |
 * | 3         | 40              | 11        | 13.3            |
 * | 4         | 60              | 12        | 20              |
 * | 5         | 80              | 13        | 26.7            |
 * | 6         | 120             | 14        | 40              |
 * | 7         | 160             | 15        | 53.3            |
 * +-----------+-----------------+-----------+-----------------+
 *
 * The left part  with a multiplier of 10, and the right part  with a multiplier
 * of 3.3.
 * So we have two formulas to calculate the global current:
 *   for the left part of the table:
 *     imax = coefficient * 10
 *
 *   for the right part of the table:
 *     imax = coefficient * 3.3
 *
 * The coefficient table consists of the following values:
 *   1, 2, 3, 4, 6, 8, 12, 16.
 */
static int aw200xx_set_imax(const struct aw200xx *const chip,
			    u32 led_imax_uA)
{
	u32 g_imax_uA = aw200xx_imax_to_global(chip, led_imax_uA);
	static const u32 coeff_table[] = {1, 2, 3, 4, 6, 8, 12, 16};
	u32 gccr_imax = UINT_MAX;
	u32 cur_imax = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_table); i++) {
		u32 imax;

		/* select closest ones */
		imax = coeff_table[i] * AW200XX_IMAX_MULTIPLIER1;
		if (g_imax_uA >= imax && imax > cur_imax) {
			cur_imax = imax;
			gccr_imax = i + AW200XX_IMAX_BASE_VAL1;
		}

		imax = coeff_table[i] * AW200XX_IMAX_MULTIPLIER2;
		imax = DIV_ROUND_CLOSEST(imax, 100) * 100;
		if (g_imax_uA >= imax && imax > cur_imax) {
			cur_imax = imax;
			gccr_imax = i + AW200XX_IMAX_BASE_VAL2;
		}
	}

	if (gccr_imax == UINT_MAX)
		return -EINVAL;

	return regmap_update_bits(chip->regmap, AW200XX_REG_GCCR,
				  AW200XX_GCCR_IMAX_MASK,
				  AW200XX_GCCR_IMAX(gccr_imax));
}

static int aw200xx_chip_reset(const struct aw200xx *const chip)
{
	int ret;

	ret = regmap_write(chip->regmap, AW200XX_REG_RSTR, AW200XX_RSTR_RESET);
	if (ret)
		return ret;

	/* According to the datasheet software reset takes at least 1ms */
	fsleep(1000);

	regcache_mark_dirty(chip->regmap);
	return regmap_write(chip->regmap, AW200XX_REG_FCD, AW200XX_FCD_CLEAR);
}

static int aw200xx_chip_init(const struct aw200xx *const chip)
{
	u32 columns;
	u32 chan;
	int ret;

	if (!chip || !chip->cdef)
		return -EINVAL;

	columns = chip->cdef->display_size_columns;

	ret = regmap_write(chip->regmap, AW200XX_REG_DSIZE,
			   chip->display_rows - 1);
	if (ret)
		return ret;

	ret = regmap_write(chip->regmap, AW200XX_REG_SLPCR,
			   AW200XX_SLPCR_ACTIVE);
	if (ret)
		return ret;

	ret = regmap_update_bits(chip->regmap, AW200XX_REG_GCCR,
				 AW200XX_GCCR_ALLON, AW200XX_GCCR_ALLON);
	if (ret)
		return ret;

	/*
	 * Match legacy/vendor init: program Page1 DIM (per-channel current
	 * scaling) to max (0x3f) for all channels. Without this, even with
	 * Page4 DIM/FADE maxed, LEDs may appear much dimmer.
	 *
	 * Reference: leds-aw20072-reg.h sets page=0xC1 and writes 0x00..0x47=0x3f.
	 */
	for (chan = 0; chan < chip->cdef->channels; chan++) {
		ret = regmap_write(chip->regmap,
				   AW200XX_REG_DIM_PAGE1(chan, columns),
				   AW200XX_DIM_MAX);
		if (ret)
			return ret;
	}

	return 0;
}

static int aw200xx_chip_check(const struct aw200xx *const chip)
{
	struct device *dev = &chip->client->dev;
	u32 chipid;
	int ret;

	ret = regmap_read(chip->regmap, AW200XX_REG_IDR, &chipid);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read chip ID\n");

	if (chipid != AW200XX_IDR_CHIPID)
		return dev_err_probe(dev, -ENODEV,
				     "Chip reported wrong ID: %x\n", chipid);

	return 0;
}

static void aw200xx_enable(const struct aw200xx *const chip)
{
	gpiod_set_value_cansleep(chip->hwen, 1);

	/*
	 * After HWEN pin set high the chip begins to load the OTP information,
	 * which takes 200us to complete. About 200us wait time is needed for
	 * internal oscillator startup and display SRAM initialization. After
	 * display SRAM initialization, the registers in page1 to page5 can be
	 * configured via i2c interface.
	 */
	fsleep(400);
}

static void aw200xx_disable(const struct aw200xx *const chip)
{
	return gpiod_set_value_cansleep(chip->hwen, 0);
}

static int aw200xx_probe_get_display_rows(struct device *dev,
					  struct aw200xx *chip)
{
	struct fwnode_handle *child;
	u32 max_source = 0;

	device_for_each_child_node(dev, child) {
		u32 source;
		int ret;

		ret = fwnode_property_read_u32(child, "reg", &source);
		if (ret || source >= chip->cdef->channels)
			continue;

		max_source = max(max_source, source);
	}

	if (max_source == 0)
		return -EINVAL;

	chip->display_rows = max_source / chip->cdef->display_size_columns + 1;

	return 0;
}

static int aw200xx_probe_fw(struct device *dev, struct aw200xx *chip)
{
	struct fwnode_handle *child;
	u32 current_min, current_max, min_uA;
	int ret;
	int i;

	ret = aw200xx_probe_get_display_rows(dev, chip);
	if (ret)
		return dev_err_probe(dev, ret,
				     "No valid led definitions found\n");

	current_max = aw200xx_imax_from_global(chip, AW200XX_IMAX_MAX_uA);
	current_min = aw200xx_imax_from_global(chip, AW200XX_IMAX_MIN_uA);
	min_uA = UINT_MAX;
	i = 0;

	device_for_each_child_node(dev, child) {
		struct led_init_data init_data = {};
		struct aw200xx_led *led;
		u32 source, imax;

		ret = fwnode_property_read_u32(child, "reg", &source);
		if (ret) {
			dev_err(dev, "Missing reg property\n");
			chip->num_leds--;
			continue;
		}

		if (source >= chip->cdef->channels) {
			dev_err(dev, "LED reg %u out of range (max %u)\n",
				source, chip->cdef->channels);
			chip->num_leds--;
			continue;
		}

		ret = fwnode_property_read_u32(child, "led-max-microamp",
					       &imax);
		if (ret) {
			dev_info(&chip->client->dev,
				 "DT property led-max-microamp is missing\n");
		} else if (imax < current_min || imax > current_max) {
			dev_err(dev, "Invalid value %u for led-max-microamp\n",
				imax);
			chip->num_leds--;
			continue;
		} else {
			min_uA = min(min_uA, imax);
		}

		led = &chip->leds[i];
		led->dim = -1;
		led->num = source;
		led->chip = chip;
		led->cdev.brightness_set_blocking = aw200xx_brightness_set;
		led->cdev.max_brightness = AW200XX_FADE_MAX;
		led->cdev.groups = dim_groups;
		init_data.fwnode = child;

		ret = devm_led_classdev_register_ext(dev, &led->cdev,
						     &init_data);
		if (ret) {
			fwnode_handle_put(child);
			break;
		}

		i++;
	}

	if (!chip->num_leds)
		return -EINVAL;

	if (min_uA == UINT_MAX) {
		min_uA = aw200xx_imax_from_global(chip,
						  AW200XX_IMAX_DEFAULT_uA);
	}

	return aw200xx_set_imax(chip, min_uA);
}

static const struct regmap_range_cfg aw200xx_ranges[] = {
	{
		.name = "aw200xx",
		.range_min = 0,
		.range_max = AW200XX_REG_MAX,
		.selector_reg = AW200XX_REG_PAGE,
		.selector_mask = AW200XX_PAGE_MASK,
		.selector_shift = AW200XX_PAGE_SHIFT,
		.window_start = 0,
		.window_len = AW200XX_PAGE_SIZE,
	},
};

static const struct regmap_range aw200xx_writeonly_ranges[] = {
	regmap_reg_range(AW200XX_REG(AW200XX_PAGE1, 0x00), AW200XX_REG_MAX),
};

static const struct regmap_access_table aw200xx_readable_table = {
	.no_ranges = aw200xx_writeonly_ranges,
	.n_no_ranges = ARRAY_SIZE(aw200xx_writeonly_ranges),
};

static const struct regmap_range aw200xx_readonly_ranges[] = {
	regmap_reg_range(AW200XX_REG_IDR, AW200XX_REG_IDR),
};

static const struct regmap_access_table aw200xx_writeable_table = {
	.no_ranges = aw200xx_readonly_ranges,
	.n_no_ranges = ARRAY_SIZE(aw200xx_readonly_ranges),
};

static const struct regmap_config aw200xx_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW200XX_REG_MAX,
	.ranges = aw200xx_ranges,
	.num_ranges = ARRAY_SIZE(aw200xx_ranges),
	.rd_table = &aw200xx_readable_table,
	.wr_table = &aw200xx_writeable_table,
	.cache_type = REGCACHE_MAPLE,
	.disable_locking = true,
};

static void aw200xx_chip_reset_action(void *data)
{
	aw200xx_chip_reset(data);
}

static void aw200xx_disable_action(void *data)
{
	aw200xx_disable(data);
}

static void aw200xx_cancel_breath_work_action(void *data)
{
	struct aw200xx *chip = data;

	cancel_delayed_work_sync(&chip->breath_work);
}

static int aw200xx_add_board_groups(struct device *dev)
{
	const struct attribute_group *const *grp;
	int ret;

	for (grp = aw200xx_board_groups; *grp; grp++) {
		ret = devm_device_add_group(dev, *grp);
		if (ret)
			return ret;
	}

	return 0;
}

static int aw200xx_apply_boot_board_cfg(struct aw200xx *chip)
{
	struct device *dev = &chip->client->dev;
	const char *mode = NULL;
	const char *color = NULL;
	u32 br = 0;
	u32 interval_ms = 0;
	u32 step = 0;
	int ret;

	/* Defaults: solid white @ 0 (off) unless DT asks otherwise */
	chip->board_mode = AW200XX_BOARD_OFF;
	chip->board_color = AW200XX_COLOR_WHITE;
	chip->board_brightness = 0;
	chip->solid_brightness = 0;
	chip->breath_interval_ms = 20;
	chip->breath_step = 5;

	device_property_read_string(dev, "awinic,boot-mode", &mode);
	device_property_read_string(dev, "awinic,boot-color", &color);
	device_property_read_u32(dev, "awinic,boot-brightness", &br);
	device_property_read_u32(dev, "awinic,breath-interval-ms", &interval_ms);
	device_property_read_u32(dev, "awinic,breath-step", &step);

	if (br > 255)
		br = 255;
	chip->solid_brightness = (u8)br;
	chip->board_brightness = chip->solid_brightness;

	if (interval_ms)
		chip->breath_interval_ms = interval_ms;
	if (step && step <= 255)
		chip->breath_step = (u8)step;

	if (color) {
		if (!strcmp(color, "red"))
			chip->board_color = AW200XX_COLOR_RED;
		else if (!strcmp(color, "green"))
			chip->board_color = AW200XX_COLOR_GREEN;
		else if (!strcmp(color, "blue"))
			chip->board_color = AW200XX_COLOR_BLUE;
		else
			chip->board_color = AW200XX_COLOR_WHITE;
	}

	if (mode) {
		if (!strcmp(mode, "solid"))
			chip->board_mode = AW200XX_BOARD_SOLID;
		else if (!strcmp(mode, "breath"))
			chip->board_mode = AW200XX_BOARD_BREATH;
		else if (!strcmp(mode, "off"))
			chip->board_mode = AW200XX_BOARD_OFF;
	}

	/*
	 * Apply boot state (called with chip->mutex held by probe).
	 * chip_init() has already enabled ALLON + active mode.
	 */
	if (chip->board_mode == AW200XX_BOARD_OFF)
		return aw200xx_board_apply_locked(chip, 0);

	if (chip->board_mode == AW200XX_BOARD_SOLID)
		return aw200xx_board_apply_locked(chip, chip->board_brightness);

	/* breath */
	chip->breath_dir_up = true;
	chip->board_brightness = 0; /* keep solid_brightness unchanged */
	ret = aw200xx_board_apply_locked(chip, 0);
	if (ret)
		return ret;

	schedule_delayed_work(&chip->breath_work,
			      msecs_to_jiffies(chip->breath_interval_ms));
	return 0;
}

static int aw200xx_probe(struct i2c_client *client)
{
	const struct aw200xx_chipdef *cdef;
	struct aw200xx *chip;
	int count;
	int ret;

	cdef = device_get_match_data(&client->dev);
	if (!cdef)
		return -ENODEV;

	count = device_get_child_node_count(&client->dev);
	if (!count || count > cdef->channels)
		return dev_err_probe(&client->dev, -EINVAL,
				     "Incorrect number of leds (%d)", count);

	chip = devm_kzalloc(&client->dev, struct_size(chip, leds, count),
			    GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->cdef = cdef;
	chip->num_leds = count;
	chip->client = client;
	i2c_set_clientdata(client, chip);

	INIT_DELAYED_WORK(&chip->breath_work, aw200xx_breath_workfn);

	chip->regmap = devm_regmap_init_i2c(client, &aw200xx_regmap_config);
	if (IS_ERR(chip->regmap))
		return PTR_ERR(chip->regmap);

	chip->hwen = devm_gpiod_get_optional(&client->dev, "enable",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(chip->hwen))
		return dev_err_probe(&client->dev, PTR_ERR(chip->hwen),
				     "Cannot get enable GPIO");

	aw200xx_enable(chip);

	ret = devm_add_action(&client->dev, aw200xx_disable_action, chip);
	if (ret)
		return ret;

	ret = devm_add_action(&client->dev, aw200xx_cancel_breath_work_action, chip);
	if (ret)
		return ret;

	ret = aw200xx_chip_check(chip);
	if (ret)
		return ret;

	ret = devm_mutex_init(&client->dev, &chip->mutex);
	if (ret)
		return ret;

	/* Need a lock now since after call aw200xx_probe_fw, sysfs nodes created */
	mutex_lock(&chip->mutex);

	ret = aw200xx_chip_reset(chip);
	if (ret)
		goto out_unlock;

	ret = devm_add_action(&client->dev, aw200xx_chip_reset_action, chip);
	if (ret)
		goto out_unlock;

	ret = aw200xx_probe_fw(&client->dev, chip);
	if (ret)
		goto out_unlock;

	ret = aw200xx_chip_init(chip);
	if (ret)
		goto out_unlock;

	/* Create board-level sysfs controls under the i2c device */
	ret = aw200xx_add_board_groups(&client->dev);
	if (ret)
		goto out_unlock;

	ret = aw200xx_apply_boot_board_cfg(chip);

out_unlock:
	if (ret)
		aw200xx_disable(chip);

	mutex_unlock(&chip->mutex);
	return ret;
}

static const struct aw200xx_chipdef aw20036_cdef = {
	.channels = 36,
	.display_size_rows_max = 3,
	.display_size_columns = 12,
};

static const struct aw200xx_chipdef aw20054_cdef = {
	.channels = 54,
	.display_size_rows_max = 6,
	.display_size_columns = 9,
};

static const struct aw200xx_chipdef aw20072_cdef = {
	.channels = 72,
	.display_size_rows_max = 6,
	.display_size_columns = 12,
};

static const struct aw200xx_chipdef aw20108_cdef = {
	.channels = 108,
	.display_size_rows_max = 9,
	.display_size_columns = 12,
};

static const struct i2c_device_id aw200xx_id[] = {
	{ "aw20036" },
	{ "aw20054" },
	{ "aw20072" },
	{ "aw20108" },
	{}
};
MODULE_DEVICE_TABLE(i2c, aw200xx_id);

static const struct of_device_id aw200xx_match_table[] = {
	{ .compatible = "awinic,aw20036", .data = &aw20036_cdef, },
	{ .compatible = "awinic,aw20054", .data = &aw20054_cdef, },
	{ .compatible = "awinic,aw20072", .data = &aw20072_cdef, },
	{ .compatible = "awinic,aw20108", .data = &aw20108_cdef, },
	{}
};
MODULE_DEVICE_TABLE(of, aw200xx_match_table);

static struct i2c_driver aw200xx_driver = {
	.driver = {
		.name = "aw200xx",
		.of_match_table = aw200xx_match_table,
	},
	.probe = aw200xx_probe,
	.id_table = aw200xx_id,
};
module_i2c_driver(aw200xx_driver);

MODULE_AUTHOR("Martin Kurbanov <mmkurbanov@sberdevices.ru>");
MODULE_DESCRIPTION("AW200XX LED driver");
MODULE_LICENSE("GPL");
