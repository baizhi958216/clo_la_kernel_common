// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2024 Linaro Limited
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#include <asm/byteorder.h>

#define DEFAULT_VOLTAGE_STEPPER_RATE	38400

#define LDO_STEPPER_CTL_REG		0x3b
#define STEP_RATE_MASK			GENMASK(1, 0)

#define LDO_VSET_LB_REG			0x40

#define LDO_ENABLE_REG			0x46
#define ENABLE_BIT			BIT(7)

/*
 * MISC registers reside in the primary I2C address space (0x08).
 * LDO registers reside in the secondary I2C address space (0x09).
 */
#define MISC_BASE			0x900
#define MISC_CHIP_ENABLE_REG		(MISC_BASE + 0x50)
#define CHIP_ENABLE_BIT			BIT(0)

struct pm8008_chip {
	struct device		*dev;
	struct regmap		*regmap_primary;
	struct mutex		lock;
	int			enable_count;
};

struct pm8008_regulator {
	struct pm8008_chip	*chip;
	struct regmap		*regmap;
	struct regulator_desc	desc;
	unsigned int		base;
};

struct pm8008_regulator_data {
	const char			*name;
	const char			*supply_name;
	unsigned int			base;
	int				min_dropout_uV;
	const struct linear_range	*voltage_range;
};

static const struct linear_range nldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(528000, 0, 122, 8000),
};

static const struct linear_range pldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(1504000, 0, 237, 8000),
};

static const struct pm8008_regulator_data pm8008_reg_data[] = {
	{ "ldo1", "vdd-l1-l2", 0x4000, 225000, nldo_ranges, },
	{ "ldo2", "vdd-l1-l2", 0x4100, 225000, nldo_ranges, },
	{ "ldo3", "vdd-l3-l4", 0x4200, 300000, pldo_ranges, },
	{ "ldo4", "vdd-l3-l4", 0x4300, 300000, pldo_ranges, },
	{ "ldo5", "vdd-l5",    0x4400, 200000, pldo_ranges, },
	{ "ldo6", "vdd-l6",    0x4500, 200000, pldo_ranges, },
	{ "ldo7", "vdd-l7",    0x4600, 200000, pldo_ranges, },
};

static int pm8008_regulator_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	unsigned int mV;
	__le16 val;
	int ret;

	ret = regulator_list_voltage_linear_range(rdev, sel);
	if (ret < 0)
		return ret;

	mV = DIV_ROUND_UP(ret, 1000);

	val = cpu_to_le16(mV);

	ret = regmap_bulk_write(preg->regmap, preg->base + LDO_VSET_LB_REG,
			&val, sizeof(val));
	if (ret < 0)
		return ret;

	return 0;
}

static int pm8008_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	unsigned int uV;
	__le16 val;
	int ret;

	ret = regmap_bulk_read(preg->regmap, preg->base + LDO_VSET_LB_REG,
			&val, sizeof(val));
	if (ret < 0)
		return ret;

	uV = le16_to_cpu(val) * 1000;

	return regulator_map_voltage_linear_range(rdev, uV, INT_MAX);
}

static int pm8008_chip_aggregate(struct pm8008_chip *chip)
{
	int rc = 0;
	bool enable;

	enable = (chip->enable_count > 0);

	/*
	 * MISC_CHIP_ENABLE_REG (0x950) resides in the primary I2C address space (0x08).
	 * We must use regmap_primary here, not the secondary regmap used for LDOs.
	 */
	rc = regmap_update_bits(chip->regmap_primary, MISC_CHIP_ENABLE_REG,
				CHIP_ENABLE_BIT, enable ? CHIP_ENABLE_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "failed to %s chip rc=%d\n",
			enable ? "enable" : "disable", rc);

	return rc;
}

static int pm8008_regulator_enable(struct regulator_dev *rdev)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	struct pm8008_chip *chip = preg->chip;
	int rc;

	mutex_lock(&chip->lock);
	chip->enable_count++;
	if (chip->enable_count == 1) {
		rc = pm8008_chip_aggregate(chip);
		if (rc < 0) {
			chip->enable_count--;
			mutex_unlock(&chip->lock);
			return rc;
		}
	}
	mutex_unlock(&chip->lock);

	rc = regulator_enable_regmap(rdev);
	if (rc < 0) {
		mutex_lock(&chip->lock);
		chip->enable_count--;
		if (chip->enable_count == 0)
			pm8008_chip_aggregate(chip);
		mutex_unlock(&chip->lock);
	}

	return rc;
}

static int pm8008_regulator_disable(struct regulator_dev *rdev)
{
	struct pm8008_regulator *preg = rdev_get_drvdata(rdev);
	struct pm8008_chip *chip = preg->chip;
	int rc;

	rc = regulator_disable_regmap(rdev);
	if (rc < 0)
		return rc;

	mutex_lock(&chip->lock);
	if (chip->enable_count > 0) {
		chip->enable_count--;
		if (chip->enable_count == 0)
			pm8008_chip_aggregate(chip);
	}
	mutex_unlock(&chip->lock);

	return 0;
}

static const struct regulator_ops pm8008_regulator_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.set_voltage_sel	= pm8008_regulator_set_voltage_sel,
	.get_voltage_sel	= pm8008_regulator_get_voltage_sel,
	.enable			= pm8008_regulator_enable,
	.disable		= pm8008_regulator_disable,
	.is_enabled		= regulator_is_enabled_regmap,
};

static int pm8008_regulator_probe(struct platform_device *pdev)
{
	const struct pm8008_regulator_data *data;
	struct regulator_config config = {};
	struct device *dev = &pdev->dev;
	struct pm8008_regulator *preg;
	struct pm8008_chip *chip;
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct regmap *regmap;
	struct regmap *regmap_primary;
	unsigned int val;
	int ret, i;

	regmap = dev_get_regmap(dev->parent, "secondary");
	if (!regmap)
		return -EINVAL;

	regmap_primary = dev_get_regmap(dev->parent, "primary");
	if (!regmap_primary)
		return -EINVAL;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->regmap_primary = regmap_primary;
	mutex_init(&chip->lock);
	chip->enable_count = 0;

	for (i = 0; i < ARRAY_SIZE(pm8008_reg_data); i++) {
		data = &pm8008_reg_data[i];

		preg = devm_kzalloc(dev, sizeof(*preg), GFP_KERNEL);
		if (!preg)
			return -ENOMEM;

		preg->chip = chip;
		preg->regmap = regmap;
		preg->base = data->base;

		desc = &preg->desc;

		desc->name = data->name;
		desc->supply_name = data->supply_name;
		desc->of_match = data->name;
		desc->regulators_node = of_match_ptr("regulators");
		desc->ops = &pm8008_regulator_ops;
		desc->type = REGULATOR_VOLTAGE;
		desc->owner = THIS_MODULE;

		desc->linear_ranges = data->voltage_range;
		desc->n_linear_ranges = 1;
		desc->uV_step = desc->linear_ranges[0].step;
		desc->min_uV = desc->linear_ranges[0].min;
		desc->n_voltages = linear_range_values_in_range(&desc->linear_ranges[0]);

		ret = regmap_read(regmap, preg->base + LDO_STEPPER_CTL_REG, &val);
		if (ret < 0) {
			dev_err(dev, "failed to read step rate: %d\n", ret);
			return ret;
		}
		val &= STEP_RATE_MASK;
		desc->ramp_delay = DEFAULT_VOLTAGE_STEPPER_RATE >> val;

		desc->min_dropout_uV = data->min_dropout_uV;

		desc->enable_reg = preg->base + LDO_ENABLE_REG;
		desc->enable_mask = ENABLE_BIT;

		/* Update initial enable count based on true hardware state */
		ret = regmap_read(regmap, desc->enable_reg, &val);
		if (!ret && (val & desc->enable_mask))
			chip->enable_count++;

		config.dev = dev->parent;
		config.driver_data = preg;
		config.regmap = regmap;

		rdev = devm_regulator_register(dev, desc, &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(dev, "failed to register regulator %s: %d\n",
					desc->name, ret);
			return ret;
		}
	}

	/* Initialize hardware global state based on active regulators */
	pm8008_chip_aggregate(chip);

	return 0;
}

static const struct platform_device_id pm8008_regulator_id_table[] = {
	{ "pm8008-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, pm8008_regulator_id_table);

static struct platform_driver pm8008_regulator_driver = {
	.driver	= {
		.name = "qcom-pm8008-regulator",
	},
	.probe = pm8008_regulator_probe,
	.id_table = pm8008_regulator_id_table,
};
module_platform_driver(pm8008_regulator_driver);

MODULE_DESCRIPTION("Qualcomm PM8008 PMIC regulator driver");
MODULE_LICENSE("GPL");
