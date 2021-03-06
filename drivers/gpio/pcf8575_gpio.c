/*
 * PCF8575 I2C GPIO EXPANDER DRIVER
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

/*
 * Driver for TI PCF-8575 16 bit I2C gpio expander. Based on
 * gpio-pcf857x Linux 4.7 kernel driver.
 */

#include <common.h>
#include <dm.h>
#include <i2c.h>
#include <asm-generic/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

struct pcf8575_chip {
	int gpio_count;		/* No GPIOs supported by the chip */
	unsigned int out;	/* software latch */
	const char *bank_name;	/* Name of the expander bank */
};

/* Read/Write to 16-bit I/O expander */

static int pcf8575_i2c_write_le16(struct udevice *dev, unsigned int word)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct i2c_msg msg;
	u8 buf[2] = { word & 0xff, word >> 8, };
	int ret;

	msg.addr = chip->chip_addr;
	msg.buf = buf;
	msg.flags = 0;
	msg.len = 2;
	ret = dm_i2c_xfer(dev, &msg, 1);

	if (ret)
		printf("%s i2c write failed to addr %x\n", __func__, msg.addr);

	return ret;
}

static int pcf8575_i2c_read_le16(struct udevice *dev)
{
	struct dm_i2c_chip *chip = dev_get_parent_platdata(dev);
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	msg.addr = chip->chip_addr;
	msg.buf = buf;
	msg.flags = I2C_M_RD;
	msg.len = 2;

	ret = dm_i2c_xfer(dev, &msg, 1);
	if (ret) {
		printf("%s i2c read failed to addr %x\n", __func__, msg.addr);
		return ret;
	}

	return (msg.buf[1] << 8) | msg.buf[0];
}

static int pcf8575_direction_input(struct udevice *dev, unsigned offset)
{
	struct pcf8575_chip *pc = dev_get_platdata(dev);
	int status;

	pc->out |= BIT(offset);
	status = pcf8575_i2c_write_le16(dev, pc->out);

	return status;
}

static int pcf8575_direction_output(struct udevice *dev,
				    unsigned int offset, int value)
{
	struct pcf8575_chip *pc = dev_get_platdata(dev);
	int ret;

	if (value)
		pc->out |= BIT(offset);
	else
		pc->out &= ~BIT(offset);

	ret = pcf8575_i2c_write_le16(dev, pc->out);

	return ret;
}

static int pcf8575_get_value(struct udevice *dev, unsigned int offset)
{
	int             value;

	value = pcf8575_i2c_read_le16(dev);

	return (value < 0) ? value : ((value & BIT(offset)) >> offset);
}

static int pcf8575_set_value(struct udevice *dev, unsigned int offset,
			     int value)
{
	return pcf8575_direction_output(dev, offset, value);
}

static int pcf8575_ofdata_platdata(struct udevice *dev)
{
	struct pcf8575_chip *pc = dev_get_platdata(dev);
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	int n_latch;

	uc_priv->gpio_count = fdtdec_get_int(gd->fdt_blob, dev->of_offset,
					     "gpio-count", 16);
	uc_priv->bank_name = fdt_getprop(gd->fdt_blob, dev->of_offset,
					 "gpio-bank-name", NULL);
	if (!uc_priv->bank_name)
		uc_priv->bank_name = fdt_get_name(gd->fdt_blob,
						  dev->of_offset, NULL);

	/* NOTE:  these chips have strange "quasi-bidirectional" I/O pins.
	 * We can't actually know whether a pin is configured (a) as output
	 * and driving the signal low, or (b) as input and reporting a low
	 * value ... without knowing the last value written since the chip
	 * came out of reset (if any).  We can't read the latched output.
	 * In short, the only reliable solution for setting up pin direction
	 * is to do it explicitly.
	 *
	 * Using n_latch avoids that trouble.  When left initialized to zero,
	 * our software copy of the "latch" then matches the chip's all-ones
	 * reset state.  Otherwise it flags pins to be driven low.
	 */

	n_latch = fdtdec_get_uint(gd->fdt_blob, dev->of_offset,
				  "lines-initial-states", 0);
	pc->out = ~n_latch;

	return 0;
}

static int pcf8575_gpio_probe(struct udevice  *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	debug("%s GPIO controller with %d gpios probed\n",
	      uc_priv->bank_name, uc_priv->gpio_count);

	return 0;
}

static const struct dm_gpio_ops pcf8575_gpio_ops = {
	.direction_input	= pcf8575_direction_input,
	.direction_output	= pcf8575_direction_output,
	.get_value		= pcf8575_get_value,
	.set_value		= pcf8575_set_value,
};

static const struct udevice_id pcf8575_gpio_ids[] = {
	{ .compatible = "nxp,pcf8575" },
	{ .compatible = "ti,pcf8575" },
	{ }
};

U_BOOT_DRIVER(gpio_pcf8575) = {
	.name	= "gpio_pcf8575",
	.id	= UCLASS_GPIO,
	.ops	= &pcf8575_gpio_ops,
	.of_match = pcf8575_gpio_ids,
	.ofdata_to_platdata = pcf8575_ofdata_platdata,
	.probe	= pcf8575_gpio_probe,
	.platdata_auto_alloc_size = sizeof(struct pcf8575_chip),
};
