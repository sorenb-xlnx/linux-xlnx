/*
 * Userspace clock driver
 *
 *  Copyright (C) 2013 Xilinx
 *
 *  Sören Brinkmann <soren.brinkmann@xilinx.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Expose clock controls through sysfs to userspace.
 *
 * By writing 0/1 to 'enable' the clock can be disabled/enabled. Reading
 * that file returns the current state - 0 = disabled, 1 = enabled.
 *
 * Reading 'set_rate' returns the current clock frequency in Hz. Writing
 * the file requests setting a new frequency in Hz.
 */

#include <linux/clk-provider.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/slab.h>

#define DRIVER_NAME	"clk-userspace"

struct usclk_data {
	struct clk *clk;
	int enabled;
};

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usclk_data *pdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pdata->enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long enable;
	int ret;
	struct usclk_data *pdata = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &enable);
	if (ret)
		return -EINVAL;

	enable = !!enable;
	if (enable == pdata->enabled)
		return count;

	if (enable)
		ret = clk_prepare_enable(pdata->clk);
	else
		clk_disable_unprepare(pdata->clk);

	if (ret)
		return -EBUSY;

	pdata->enabled = enable;
	return count;
}

static DEVICE_ATTR(enable, 0644, enable_show, enable_store);

static ssize_t set_rate_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usclk_data *pdata = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", clk_get_rate(pdata->clk));
}

static ssize_t set_rate_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 0;
	unsigned long rate;
	struct usclk_data *pdata = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &rate);
	if (ret)
		return -EINVAL;

	rate = clk_round_rate(pdata->clk, rate);
	ret = clk_set_rate(pdata->clk, rate);
	if (ret)
		return -EBUSY;

	return count;
}

static DEVICE_ATTR(set_rate, 0644, set_rate_show, set_rate_store);

static const struct attribute *usclk_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_set_rate.attr,
	NULL
};

static const struct attribute_group usclk_attr_grp = {
	.attrs = (struct attribute **)usclk_attrs,
};

static int usclk_setup(void)
{
	int ret;
	int i;
	struct usclk_data *pdata;
	u32 clock_count;
	struct class *clk_class;
	struct device *dev;
	struct device_node *np = of_find_compatible_node(NULL, NULL,
			"clk-userspace");

	ret = of_property_read_u32(np, "clock-count", &clock_count);
	if (ret || !clock_count)
		return ret;

	pdata = kzalloc(clock_count * sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	clk_class = class_create(THIS_MODULE, "clk");
	if (!clk_class) {
		pr_err("unable to create class\n");
		goto err_free;
	}

	for (i = 0; i < clock_count; i++) {
		pdata[i].clk = of_clk_get(np, i);
		if (IS_ERR(pdata[i].clk)) {
			pr_warn("input clock #%u not found\n", i);
			continue;
		}

		dev = device_create(clk_class, NULL, MKDEV(0, 0), NULL,
				of_clk_get_parent_name(np, i));
		if (!dev) {
			pr_warn("unable to create device #%d\n", i);
			continue;
		}

		dev_set_drvdata(dev, &pdata[i]);
		sysfs_create_group(&dev->kobj, &usclk_attr_grp);
	}

	return 0;

err_free:
	kfree(pdata);

	return ret;
}
late_initcall(usclk_setup);
