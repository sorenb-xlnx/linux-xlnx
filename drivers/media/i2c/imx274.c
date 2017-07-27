/*
 * imx274.c - IMX274 CMOS Image Sensor driver
 *
 * Copyright (C) 2017, Leopard Imaging, Inc.
 *
 * Leon Luo <leonl@leopardimaging.com>
 * Edwin Zou <edwinz@leopardimaging.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/ratelimit.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

/*
 * See "SHR, SVR Setting" in datasheet
 */
#define IMX274_DEFAULT_FRAME_LENGTH		(4550)
#define IMX274_MAX_FRAME_LENGTH			(0x000fffff)

/*
 * See "Frame Rate Adjustment" in datasheet
 */
#define IMX274_PIXCLK_CONST1			(72000000)
#define IMX274_PIXCLK_CONST2			(1000000)

/*
 * The input gain is shifted by IMX274_GAIN_SHIFT to get
 * decimal number. The real gain is
 * (float)input_gain_value / (1 << IMX274_GAIN_SHIFT)
 */
#define IMX274_GAIN_SHIFT			(8)

/*
 * See "Analog Gain" in datasheet
 * min gain is 1X
 * max gain is 22.5X, round to 23
 */
#define IMX274_GAIN_REG_MAX			(1957)
#define IMX274_MIN_GAIN				(0x01 << IMX274_GAIN_SHIFT)
#define IMX274_MAX_GAIN				(23 << IMX274_GAIN_SHIFT)
#define IMX274_DEF_GAIN				(20 << IMX274_GAIN_SHIFT)

/*
 * 1 line time in us = (HMAX / 72)
 */
#define IMX274_MIN_EXPOSURE_TIME		(260 / 72)

#define IMX274_DEFAULT_MODE			IMX274_MODE_3840X2160
#define IMX274_MAX_WIDTH			(3840)
#define IMX274_MAX_HEIGHT			(2160)
#define IMX274_MAX_FRAME_RATE			(120)
#define IMX274_MIN_FRAME_RATE			(5)
#define IMX274_DEF_FRAME_RATE			(60)

/*
 * register SHR is limited to (SVR value + 1) x VMAX value - 4
 */
#define IMX274_SHR_LIMIT_CONST			(4)

/*
 * Constants for sensor reset delay
 */
#define IMX274_RESET_DELAY1			(2000)
#define IMX274_RESET_DELAY2			(2200)

#define DRIVER_NAME "IMX274"

/*
 * IMX274 register definitions
 */
#define IMX274_FRAME_LENGTH_ADDR_1		0x30FA /* VMAX, MSB */
#define IMX274_FRAME_LENGTH_ADDR_2		0x30F9 /* VMAX */
#define IMX274_FRAME_LENGTH_ADDR_3		0x30F8 /* VMAX, LSB */
#define IMX274_SVR_REG_MSB			0x300F /* SVR */
#define IMX274_SVR_REG_LSB			0x300E /* SVR */
#define IMX274_HMAX_REG_MSB			0x30F7 /* HMAX */
#define IMX274_HMAX_REG_LSB			0x30F6 /* HMAX */
#define IMX274_COARSE_TIME_ADDR_MSB		0x300D /* SHR */
#define IMX274_COARSE_TIME_ADDR_LSB		0x300C /* SHR */
#define IMX274_ANALOG_GAIN_ADDR_LSB		0x300A /* ANALOG GAIN LSB */
#define IMX274_ANALOG_GAIN_ADDR_MSB		0x300B /* ANALOG GAIN MSB */
#define IMX274_VFLIP_REG			0x301A /* VERTICAL FLIP */
#define IMX274_STANDBY_REG			0x3000 /* STANDBY */

#define IMX274_TABLE_WAIT_MS			0
#define IMX274_TABLE_END			1

/*
 * imx274 I2C operation related structure
 */
struct reg_8 {
	u16 addr;
	u8 val;
};

#define imx274_reg struct reg_8

static struct regmap_config imx274_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

/*
 * imx274 format related structure
 */
struct imx274_frmfmt {
	u32 mbus_code;
	enum v4l2_colorspace colorspace;
	struct v4l2_frmsize_discrete size;
	const int *framerates;
	int num_framerates;
	bool hdr_en;
	int mode;
};

/*
 * imx274 test pattern related structure
 */
enum {
	TEST_PATTERN_DISABLED,
	TEST_PATTERN_GRAY_IMAGE,
	TEST_PATTERN_COLOR_BARS,
};

static const char * const tp_qmenu[] = {
	"Disabled",
	"Gray Image",
	"Color Bars",
};

/*
 *  All-pixel scan mode (10-bit)
 * imx274 mode1(refer to datasheet) register configuration with
 * 3840x2160 resolution, raw10 data and mipi four lane output
 */
static const imx274_reg imx274_mode1_3840x2160_raw10[] = {
	{0x3004, 0x01},
	{0x3005, 0x01},
	{0x3006, 0x00},
	{0x3007, 0x02},
	{0x300C, 0xff}, /* SHR */
	{0x300D, 0x00}, /* SHR */
	{0x300E, 0x00}, /* SVR, 0 */
	{0x300F, 0x00}, /* SVR */
	{0x3018, 0xA2}, /* output XVS, HVS */
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x01},
	{0x30F6, 0x07}, /* HMAX, 263 */
	{0x30F7, 0x01}, /* HMAX */
	{0x30F8, 0xC6}, /* VMAX, 4550 */
	{0x30F9, 0x11}, /* VMAX */
	{0x30FA, 0x00}, /* VMAX */

	{0x30dd, 0x01}, /* crop to 2160 */
	{0x30de, 0x06},
	{0x30df, 0x00},
	{0x30e0, 0x12},
	{0x30e1, 0x00},
	{0x3037, 0x01}, /* to crop to 3840 */
	{0x3038, 0x0c},
	{0x3039, 0x00},
	{0x303a, 0x0c},
	{0x303b, 0x0f},

	{0x30EE, 0x01},
	{0x3130, 0x86},
	{0x3131, 0x08},
	{0x3132, 0x7E},
	{0x3133, 0x08},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x16},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x1F},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x1A},
	{0x366C, 0x19},
	{0x366D, 0x17},
	{0x3A41, 0x08},

	{IMX274_TABLE_END, 0x00}
};

/*
 * Horizontal/vertical 2/2-line binning
 * (Horizontal and vertical weightedbinning, 10-bit)
 * imx274 mode3(refer to datasheet) register configuration with
 * 1920x1080 resolution, raw10 data and mipi four lane output
 */
static const imx274_reg imx274_mode3_1920x1080_raw10[] = {
	{0x3004, 0x02},
	{0x3005, 0x21},
	{0x3006, 0x00},
	{0x3007, 0x11},
	{0x300C, 0xff}, /* SHR */
	{0x300D, 0x00}, /* SHR */
	{0x300E, 0x01}, /* SVR , 0x00: 120fps; 0x01: 60fps */
	{0x300F, 0x00}, /* SVR */
	{0x3018, 0xA2}, /* output XVS, HVS */
	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x02},

	{0x30F6, 0x04}, /* HMAX, 260 */
	{0x30F7, 0x01}, /* HMAX */
	{0x30F8, 0x06}, /* VMAX, 2310 */
	{0x30F9, 0x09}, /* VMAX */
	{0x30FA, 0x00}, /* VMAX */

	{0x30dd, 0x01}, /* to crop to 1920x1080 */
	{0x30de, 0x05},
	{0x30df, 0x00},
	{0x30e0, 0x04},
	{0x30e1, 0x00},
	{0x3037, 0x01},
	{0x3038, 0x0c},
	{0x3039, 0x00},
	{0x303a, 0x0c},
	{0x303b, 0x0f},

	{0x30EE, 0x01},
	{0x3130, 0x4E},
	{0x3131, 0x04},
	{0x3132, 0x46},
	{0x3133, 0x04},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x1A},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x00},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x1A},
	{0x366C, 0x19},
	{0x366D, 0x17},
	{0x3A41, 0x08},

	{IMX274_TABLE_END, 0x00}
};

/*
 * Vertical 2/3 subsampling binning horizontal 3 binning
 * imx274 mode5(refer to datasheet) register configuration with
 * 1280x720 resolution, raw10 data and mipi four lane output
 */
static const imx274_reg imx274_mode5_1280x720_raw10[] = {
	{0x3004, 0x03},
	{0x3005, 0x31},
	{0x3006, 0x00},
	{0x3007, 0x09},

	{0x300C, 0xff}, /* SHR */
	{0x300D, 0x00}, /* SHR */
	{0x300E, 0x01}, /* SVR , 0x00: 120fps; 0x01: 60fps */
	{0x300F, 0x00}, /* SVR */
	{0x3018, 0xA2}, /* output XVS, HVS */

	{0x301A, 0x00},
	{0x306B, 0x05},
	{0x30E2, 0x03},

	{0x30F6, 0x04}, /* HMAX, 260 */
	{0x30F7, 0x01}, /* HMAX */
	{0x30F8, 0x06}, /* VMAX, 2310 */
	{0x30F9, 0x09}, /* VMAX */
	{0x30FA, 0x00}, /* VMAX */

	{0x30DD, 0x01},
	{0x30DE, 0x07},
	{0x30DF, 0x00},
	{0x40E0, 0x04},
	{0x30E1, 0x00},
	{0x3030, 0xD4},
	{0x3031, 0x02},
	{0x3032, 0xD0},
	{0x3033, 0x02},

	{0x30EE, 0x01},
	{0x3130, 0xE2},
	{0x3131, 0x02},
	{0x3132, 0xDE},
	{0x3133, 0x02},
	{0x3342, 0x0A},
	{0x3343, 0x00},
	{0x3344, 0x1B},
	{0x3345, 0x00},
	{0x33A6, 0x01},
	{0x3528, 0x0E},
	{0x3554, 0x00},
	{0x3555, 0x01},
	{0x3556, 0x01},
	{0x3557, 0x01},
	{0x3558, 0x01},
	{0x3559, 0x00},
	{0x355A, 0x00},
	{0x35BA, 0x0E},
	{0x366A, 0x1B},
	{0x366B, 0x19},
	{0x366C, 0x17},
	{0x366D, 0x17},
	{0x3A41, 0x04},

	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 first step register configuration for
 * starting stream
 */
static const imx274_reg imx274_start_1[] = {
	{IMX274_STANDBY_REG, 0x12},
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 second step register configuration for
 * starting stream
 */
static const imx274_reg imx274_start_2[] = {
	{0x3120, 0xF0}, /* clock settings */
	{0x3121, 0x00}, /* clock settings */
	{0x3122, 0x02}, /* clock settings */
	{0x3129, 0x9C}, /* clock settings */
	{0x312A, 0x02}, /* clock settings */
	{0x312D, 0x02}, /* clock settings */

	{0x310B, 0x00},

	/* PLSTMG */
	{0x304C, 0x00}, /* PLSTMG01 */
	{0x304D, 0x03},
	{0x331C, 0x1A},
	{0x331D, 0x00},
	{0x3502, 0x02},
	{0x3529, 0x0E},
	{0x352A, 0x0E},
	{0x352B, 0x0E},
	{0x3538, 0x0E},
	{0x3539, 0x0E},
	{0x3553, 0x00},
	{0x357D, 0x05},
	{0x357F, 0x05},
	{0x3581, 0x04},
	{0x3583, 0x76},
	{0x3587, 0x01},
	{0x35BB, 0x0E},
	{0x35BC, 0x0E},
	{0x35BD, 0x0E},
	{0x35BE, 0x0E},
	{0x35BF, 0x0E},
	{0x366E, 0x00},
	{0x366F, 0x00},
	{0x3670, 0x00},
	{0x3671, 0x00},

	/* PSMIPI */
	{0x3304, 0x32}, /* PSMIPI1 */
	{0x3305, 0x00},
	{0x3306, 0x32},
	{0x3307, 0x00},
	{0x3590, 0x32},
	{0x3591, 0x00},
	{0x3686, 0x32},
	{0x3687, 0x00},

	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 third step register configuration for
 * starting stream
 */
static const imx274_reg imx274_start_3[] = {
	{IMX274_STANDBY_REG, 0x00},
	{0x303E, 0x02}, /* SYS_MODE = 2 */
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 forth step register configuration for
 * starting stream
 */
static const imx274_reg imx274_start_4[] = {
	{0x30F4, 0x00},
	{0x3018, 0xA2}, /* XHS VHS OUTUPT */
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 register configuration for stoping stream
 */
static const imx274_reg imx274_stop[] = {
	{IMX274_STANDBY_REG, 0x01},
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 disable test pattern register configuration
 */
static const imx274_reg imx274_tp_disabled[] = {
	{0x303C, 0x00},
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 gray image test pattern register configuration
 */
static const imx274_reg imx274_tp_gray_image[] = {
	{0x303C, 0x11},
	{0x303D, 0x03},
	{0x370E, 0x01},
	{0x377F, 0x01},
	{0x3781, 0x01},
	{0x370B, 0x11},
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 color bar test pattern register configuration
 */
static const imx274_reg imx274_tp_color_bars[] = {
	{0x303C, 0x11},
	{0x303D, 0x0A},
	{0x370E, 0x01},
	{0x377F, 0x01},
	{0x3781, 0x01},
	{0x370B, 0x11},
	{IMX274_TABLE_END, 0x00}
};

/*
 * imx274 mode related structure
 */
enum {
	IMX274_MODE_3840X2160,
	IMX274_MODE_1920X1080,
	IMX274_MODE_1280X720,

	IMX274_MODE_START_STREAM_1,
	IMX274_MODE_START_STREAM_2,
	IMX274_MODE_START_STREAM_3,
	IMX274_MODE_START_STREAM_4,
	IMX274_MODE_STOP_STREAM
};

static const imx274_reg *mode_table[] = {
	[IMX274_MODE_3840X2160]		= imx274_mode1_3840x2160_raw10,
	[IMX274_MODE_1920X1080]		= imx274_mode3_1920x1080_raw10,
	[IMX274_MODE_1280X720]		= imx274_mode5_1280x720_raw10,

	[IMX274_MODE_START_STREAM_1]	= imx274_start_1,
	[IMX274_MODE_START_STREAM_2]	= imx274_start_2,
	[IMX274_MODE_START_STREAM_3]	= imx274_start_3,
	[IMX274_MODE_START_STREAM_4]	= imx274_start_4,
	[IMX274_MODE_STOP_STREAM]	= imx274_stop,
};

/*
 * imx274 framerate related structure
 */
static const int imx274_framerate[] = {
	60
};

/*
 * imx274 format related structure
 */
static const struct imx274_frmfmt imx274_formats[] = {
	{MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB, {3840, 2160},
		imx274_framerate, 1, 0, IMX274_MODE_3840X2160},
	{MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB, {1920, 1080},
		imx274_framerate, 1, 0, IMX274_MODE_1920X1080},
	{MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_COLORSPACE_SRGB, {1280, 720},
		imx274_framerate, 1, 0, IMX274_MODE_1280X720},
};

/*
 * minimal frame length for each mode
 * refer to datasheet section "Frame Rate Adjustment (CSI-2)"
 */
static int min_frame_len[] = {
	4550, /* mode 1, 4K */
	2310, /* mode 3, 1080p */
	2310 /* mode 5, 720p */
};

/*
 * minimal numbers of SHR register
 * refer to datasheet table "Shutter Setting (CSI-2)"
 */
static int min_SHR[] = {
	12, /* mode 1, 4K */
	8, /* mode 3, 1080p */
	8 /* mode 5, 720p */
};

static int max_frame_rate[] = {
	60, /* mode 1 , 4K */
	120, /* mode 3, 1080p */
	120 /* mode 5, 720p */
};

/*
 * struct imx274_ctrls - imx274 ctrl structure
 * @handler: V4L2 ctrl handler structure
 * @exposure: Pointer to expsure ctrl structure
 * @gain: Pointer to gain ctrl structure
 * @vflip: Pointer to vflip ctrl structure
 * @test_pattern: Pointer to test pattern ctrl structure
 */
struct imx274_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *test_pattern;
};

/*
 * struct stim274 - imx274 device structure
 * @sd: V4L2 subdevice structure
 * @pd: Media pad structure
 * @client: Pointer to I2C client
 * @ctrls: imx274 control structure
 * @format: V4L2 media bus frame format structure
 * @frame_rate: V4L2 frame rate structure
 * @regmap: Pointer to regmap structure
 * @reset_gpio: Pointer to reset gpio
 * @lock: Mutex structure
 * @mode_index: Resolution mode index
 */
struct stimx274 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct i2c_client *client;
	struct imx274_ctrls ctrls;
	struct v4l2_mbus_framefmt format;
	struct v4l2_fract frame_interval;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock; /* mutex lock for operations */
	u32 mode_index;
};

/*
 * Function declaration
 */
static int imx274_set_gain(struct stimx274 *priv, s64 val);
static int imx274_set_exposure(struct stimx274 *priv, s64 val);
static int imx274_set_vflip(struct stimx274 *priv, int val);
static int imx274_set_test_pattern(struct stimx274 *priv, int val);
static int __imx274_set_frame_interval(struct stimx274 *priv,
				       struct v4l2_fract frame_interval);

static inline void msleep_range(unsigned int delay_base)
{
	usleep_range(delay_base * 1000, delay_base * 1000 + 500);
}

/*
 * v4l2_ctrl and v4l2_subdev related operations
 */
static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct stimx274, ctrls.handler)->sd;
}

static inline struct stimx274 *to_imx274(struct v4l2_subdev *sd)
{
	return container_of(sd, struct stimx274, sd);
}

/*
 * regmap_util_write_table_8 - Function for writing register table
 * @regmap: Pointer to device reg map structure
 * @table: Table containing register values
 * @wait_ms_addr: Flag for performing delay
 * @end_addr: Flag for incating end of table
 *
 * This is used to write register table into sensor's reg map.
 *
 * Return: 0 on success, errors otherwise
 */
int regmap_util_write_table_8(struct regmap *regmap,
			      const struct reg_8 table[],
			      u16 wait_ms_addr, u16 end_addr)
{
	int err;
	const struct reg_8 *next;
	u8 val;

	int range_start = -1;
	int range_count = 0;
	u8 range_vals[16];
	int max_range_vals = ARRAY_SIZE(range_vals);

	for (next = table;; next++) {
		if  ((next->addr != range_start + range_count) ||
		     (next->addr == end_addr) ||
		     (next->addr == wait_ms_addr) ||
		     (range_count == max_range_vals)) {
			if (range_count == 1)
				err = regmap_write(regmap,
						   range_start, range_vals[0]);
			else if (range_count > 1)
				err = regmap_bulk_write(regmap, range_start,
							&range_vals[0],
							range_count);

			if (err)
				return err;

			range_start = -1;
			range_count = 0;

			/* Handle special address values */
			if (next->addr == end_addr)
				break;

			if (next->addr == wait_ms_addr) {
				msleep_range(next->val);
				continue;
			}
		}

		val = next->val;

		if (range_start == -1)
			range_start = next->addr;

		range_vals[range_count++] = val;
	}
	return 0;
}

/*
 * Regsiter related operations
 */
static inline int imx274_read_reg(struct stimx274 *priv, u16 addr, u8 *val)
{
	int err;

	err = regmap_read(priv->regmap, addr, (unsigned int *)val);
	if (err)
		v4l2_err(&priv->sd,
			 "%s : i2c read failed, addr = %x\n", __func__, addr);
	else
		v4l2_dbg(2, debug, &priv->sd,
			 "%s : addr 0x%x, val=0x%x\n", __func__,
			 addr, *val);
	return err;
}

static inline int imx274_write_reg(struct stimx274 *priv, u16 addr, u8 val)
{
	int err;

	err = regmap_write(priv->regmap, addr, val);
	if (err)
		v4l2_err(&priv->sd,
			 "%s : i2c write failed, %x = %x\n", __func__,
			 addr, val);
	else
		v4l2_dbg(2, debug, &priv->sd,
			 "%s : addr 0x%x, val=0x%x\n", __func__,
			 addr, val);
	return err;
}

static int imx274_write_table(struct stimx274 *priv, const imx274_reg table[])
{
	return regmap_util_write_table_8(priv->regmap,
		table, IMX274_TABLE_WAIT_MS, IMX274_TABLE_END);
}

/*
 * imx274_start_stream - Function for starting stream per mode index
 * @priv: Pointer to device structure
 * @mode: Mode index value
 *
 * This is used to start steam per mode index.
 * mode = 0, start stream for sensor Mode 1: 4K/raw10
 * mode = 1, start stream for sensor Mode 3: 1080p/raw10
 * mode = 2, start stream for sensor Mode 5: 720p/raw10
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_start_stream(struct stimx274 *priv, int mode)
{
	int err = 0;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_1]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_2]);
	if (err)
		return err;

	err = imx274_write_table(priv, mode_table[mode]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_3]);
	if (err)
		return err;

	msleep(20);
	err = imx274_write_table(priv, mode_table[IMX274_MODE_START_STREAM_4]);
	if (err)
		return err;

	return 0;
}

/*
 * imx274_reset - Function called to reset the sensor
 * @priv: Pointer to device structure
 * @rst: Input value for determining the sensor's end state after reset
 *
 * Set the senor in reset and then
 * if rst = 0, keep it in reset;
 * if rst = 1, bring it out of reset.
 *
 */
static void imx274_reset(struct stimx274 *priv, int rst)
{
	gpiod_set_value_cansleep(priv->reset_gpio, 0);
	usleep_range(IMX274_RESET_DELAY1, IMX274_RESET_DELAY2);
	gpiod_set_value_cansleep(priv->reset_gpio, !!rst);
	usleep_range(IMX274_RESET_DELAY1, IMX274_RESET_DELAY2);
}

/**
 * imx274_g_volatile_ctrl - get the imx274 V4L2 controls
 * @ctrl: Pointer to V4L2 control
 *
 * This is used to get the imx274 V4L2 controls.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct stimx274 *imx274 = to_imx274(sd);
	int ret = 0;

	mutex_lock(&imx274->lock);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : get V4L2_CID_EXPOSURE\n", __func__);
		ctrl->val = imx274->ctrls.exposure->val;
		break;

	case V4L2_CID_GAIN:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : get V4L2_CID_GAIN\n", __func__);
		ctrl->val = imx274->ctrls.gain->val;
		break;

	case V4L2_CID_VFLIP:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : get V4L2_CID_VFLIP\n", __func__);
		ctrl->val = imx274->ctrls.vflip->val;
		break;

	case V4L2_CID_TEST_PATTERN:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : get V4L2_CID_TEST_PATTERN\n", __func__);
		ctrl->val = imx274->ctrls.test_pattern->val;
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&imx274->lock);
	return ret;
}

/**
 * imx274_s_ctrl - This is used to set the imx274 V4L2 controls
 * @ctrl: V4L2 control to be set
 *
 * This function is used to set the V4L2 controls for the imx274 sensor.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct stimx274 *imx274 = to_imx274(sd);
	int ret = -EINVAL;

	mutex_lock(&imx274->lock);

	v4l2_dbg(1, debug, &imx274->sd,
		 "%s : s_ctrl: %s, value: %d\n", __func__,
		ctrl->name, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : set V4L2_CID_EXPOSURE\n", __func__);
		ret = imx274_set_exposure(imx274, ctrl->val);
		break;

	case V4L2_CID_GAIN:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : set V4L2_CID_GAIN\n", __func__);
		ret = imx274_set_gain(imx274, ctrl->val);
		break;

	case V4L2_CID_VFLIP:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : set V4L2_CID_VFLIP\n", __func__);
		ret = imx274_set_vflip(imx274, ctrl->val);
		break;

	case V4L2_CID_TEST_PATTERN:
		v4l2_dbg(1, debug, &imx274->sd,
			 "%s : set V4L2_CID_TEST_PATTERN\n", __func__);
		ret = imx274_set_test_pattern(imx274, ctrl->val);
		break;
	}

	mutex_unlock(&imx274->lock);
	return ret;
}

/**
 * imx274_get_fmt - Get the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @fmt: Pointer to pad level media bus format
 *
 * This function is used to get the pad format information.
 *
 * Return: 0 on success
 */
static int imx274_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct stimx274 *imx274 = to_imx274(sd);

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&imx274->lock);
	fmt->format = imx274->format;
	mutex_unlock(&imx274->lock);

	return 0;
}

/**
 * imx274_set_fmt - This is used to set the pad format
 * @sd: Pointer to V4L2 Sub device structure
 * @cfg: Pointer to sub device pad information structure
 * @format: Pointer to pad level media bus format
 *
 * This function is used to set the pad format.
 *
 * Return: 0 on success
 */
static int imx274_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt = &format->format;
	struct stimx274 *imx274 = to_imx274(sd);
	struct i2c_client *client = imx274->client;
	int index;

	v4l2_dbg(1, debug, client, "%s: width = %d height = %d\n",
		 __func__, fmt->width, fmt->height);

	if (format->pad)
		return -EINVAL;

	mutex_lock(&imx274->lock);

	for (index = 0; index < ARRAY_SIZE(imx274_formats); index++) {
		if (imx274_formats[index].size.width == fmt->width &&
		    imx274_formats[index].size.height == fmt->height)
			break;
	}

	if (index >= ARRAY_SIZE(imx274_formats)) {
		/* default to first format */
		index = 0;
	}

	imx274->mode_index = index;

	if (fmt->width > IMX274_MAX_WIDTH)
		fmt->width = IMX274_MAX_WIDTH;
	if (fmt->height > IMX274_MAX_HEIGHT)
		fmt->height = IMX274_MAX_HEIGHT;
	fmt->width = fmt->width & (~3);
	fmt->height = fmt->height & (~3);
	fmt->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *fmt;
	else
		imx274->format = *fmt;

	mutex_unlock(&imx274->lock);
	return 0;
}

/**
 * imx274_g_frame_interval - Get the frame interval
 * @sd: Pointer to V4L2 Sub device structure
 * @fi: Pointer to V4l2 Sub device frame interval structure
 *
 * This function is used to get the frame interval.
 *
 * Return: 0 on success
 */
static int imx274_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct stimx274 *imx274 = to_imx274(sd);

	mutex_lock(&imx274->lock);

	fi->interval = imx274->frame_interval;
	v4l2_dbg(1, debug, &imx274->sd, "%s frame rate = %d / %d\n",
		 __func__, imx274->frame_interval.numerator,
		imx274->frame_interval.denominator);

	mutex_unlock(&imx274->lock);

	return 0;
}

/**
 * imx274_s_frame_interval - Set the frame interval
 * @sd: Pointer to V4L2 Sub device structure
 * @fi: Pointer to V4l2 Sub device frame interval structure
 *
 * This function is used to set the frame intervavl.
 *
 * Return: 0 on success
 */
static int imx274_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct stimx274 *imx274 = to_imx274(sd);
	u64 req_frame_rate;
	int ret;

	v4l2_dbg(1, debug, &imx274->sd, "%s: input frame interval = %d / %d",
		 __func__, fi->interval.numerator, fi->interval.denominator);

	if (fi->interval.denominator == 0)
		return -EINVAL;

	mutex_lock(&imx274->lock);

	req_frame_rate = (u64)(fi->interval.denominator
				/ fi->interval.numerator);

	/* boundary check */
	if (req_frame_rate > max_frame_rate[imx274->mode_index]) {
		fi->interval.numerator = 1;
		fi->interval.denominator = max_frame_rate[imx274->mode_index];
	} else if (req_frame_rate < IMX274_MIN_FRAME_RATE) {
		fi->interval.numerator = 1;
		fi->interval.denominator = IMX274_MIN_FRAME_RATE;
	}

	imx274->frame_interval = fi->interval;

	ret = __imx274_set_frame_interval(imx274,  imx274->frame_interval);

	if (!ret) {
		/* update exposure time accordingly */
		ret = imx274_set_exposure(imx274, imx274->ctrls.exposure->val);

		v4l2_dbg(1, debug, &imx274->sd, "set frame interval to %uus\n",
			 fi->interval.numerator * 1000000
			 / fi->interval.denominator);
	}

	mutex_unlock(&imx274->lock);
	return ret;
}

/**
 * imx274_load_default - load default control values
 * @priv: Pointer to device structure
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_load_default(struct stimx274 *priv)
{
	int ret;

	/* load default control values */
	priv->frame_interval.numerator = 1;
	priv->frame_interval.denominator = IMX274_DEF_FRAME_RATE;
	priv->ctrls.exposure->val = 1000000 / IMX274_DEF_FRAME_RATE;
	priv->ctrls.gain->val = IMX274_DEF_GAIN;
	priv->ctrls.vflip->val = 0;
	priv->ctrls.test_pattern->val = TEST_PATTERN_DISABLED;

	/* update frame rate */
	ret = __imx274_set_frame_interval(priv,
					  priv->frame_interval);
	if (ret)
		return ret;

	/* update exposure time */
	ret = imx274_set_exposure(priv, priv->ctrls.exposure->val);
	if (ret)
		return ret;

	/* update gain */
	ret = imx274_set_gain(priv, priv->ctrls.gain->val);
	if (ret)
		return ret;

	/* update vflip */
	ret = imx274_set_vflip(priv, priv->ctrls.vflip->val);
	if (ret)
		return ret;

	return 0;
}

/**
 * imx274_s_stream - It is used to start/stop the streaming.
 * @sd: V4L2 Sub device
 * @on: Flag (True / False)
 *
 * This function controls the start or stop of streaming for the
 * imx274 sensor.
 *
 * Return: 0 on success, errors otherwise
 */
static int imx274_s_stream(struct v4l2_subdev *sd, int on)
{
	struct stimx274 *imx274 = to_imx274(sd);
	int ret = 0;

	mutex_lock(&imx274->lock);

	v4l2_dbg(1, debug, &imx274->sd, "%s : %s, mode index = %d\n", __func__,
		 on ? "Stream Start" : "Stream Stop", imx274->mode_index);

	if (on) {
		/* start stream */
		ret = imx274_start_stream(imx274, imx274->mode_index);
		if (ret)
			goto fail;

		ret = imx274_load_default(imx274);
		if (ret)
			goto fail;

	} else {
		/* stop stream */
		ret = imx274_write_table(imx274,
					 mode_table[IMX274_MODE_STOP_STREAM]);
		if (ret)
			goto fail;
	}

	v4l2_dbg(1, debug, &imx274->sd,
		 "%s : Done: mode = %d\n", __func__, imx274->mode_index);

	mutex_unlock(&imx274->lock);
	return 0;

fail:
	v4l2_err(&imx274->sd, "s_stream failed\n");

	mutex_unlock(&imx274->lock);
	return ret;
}

static inline void imx274_calculate_frame_length_regs(imx274_reg *regs,
						      u32 frame_length)
{
	regs->addr = IMX274_FRAME_LENGTH_ADDR_1;
	regs->val = (frame_length >> 16) & 0x0f;
	(regs + 1)->addr = IMX274_FRAME_LENGTH_ADDR_2;
	(regs + 1)->val = (frame_length >> 8) & 0xff;
	(regs + 2)->addr = IMX274_FRAME_LENGTH_ADDR_3;
	(regs + 2)->val = (frame_length) & 0xff;
}

static inline void imx274_calculate_coarse_time_regs(imx274_reg *regs,
						     u32 coarse_time)
{
	regs->addr = IMX274_COARSE_TIME_ADDR_MSB;
	regs->val = (coarse_time >> 8) & 0x00ff;
	(regs + 1)->addr = IMX274_COARSE_TIME_ADDR_LSB;
	(regs + 1)->val = (coarse_time) & 0x00ff;
}

static inline void imx274_calculate_gain_regs(imx274_reg *regs, u16 gain)
{
	regs->addr = IMX274_ANALOG_GAIN_ADDR_MSB;
	regs->val = (gain >> 8) & 0x07;

	(regs + 1)->addr = IMX274_ANALOG_GAIN_ADDR_LSB;
	(regs + 1)->val = (gain) & 0xff;
}

/*
 * imx274_get_frame_length - Function for obtaining current frame length
 * @priv: Pointer to device structure
 * @val: Pointer to obainted value
 *
 * frame_length = vmax x (svr + 1), in unit of hmax.
 *
 * Return: 0 on success
 */
static int imx274_get_frame_length(struct stimx274 *priv, s64 *val)
{
	int err;
	u16 svr;
	u32 vmax;
	u8 reg_val[3];

	/* svr */
	err = imx274_read_reg(priv, IMX274_SVR_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv, IMX274_SVR_REG_MSB, &reg_val[1]);
	if (err)
		goto fail;
	svr = (reg_val[1] << 8) + reg_val[0];

	/* vmax */
	err =  imx274_read_reg(priv, IMX274_FRAME_LENGTH_ADDR_3, &reg_val[0]);
	err |=  imx274_read_reg(priv, IMX274_FRAME_LENGTH_ADDR_2, &reg_val[1]);
	err |=  imx274_read_reg(priv, IMX274_FRAME_LENGTH_ADDR_1, &reg_val[2]);
	if (err)
		goto fail;
	vmax = ((reg_val[2] & 0x07) << 16) + (reg_val[1] << 8) + reg_val[0];

	*val = vmax * (svr + 1);
	return 0;

fail:
	v4l2_err(&priv->sd, "Get frame_length error\n");
	return err;
}

static int imx274_clamp_coarse_time(struct stimx274 *priv, s64 *val,
				    s64 *frame_length)
{
	int err;

	err = imx274_get_frame_length(priv, frame_length);
	if (err)
		return err;

	if (*frame_length < min_frame_len[priv->mode_index])
		*frame_length = min_frame_len[priv->mode_index];

	*val = *frame_length - *val; /* convert to raw shr */
	if (*val > *frame_length - IMX274_SHR_LIMIT_CONST)
		*val = *frame_length - IMX274_SHR_LIMIT_CONST;
	else if (*val < min_SHR[priv->mode_index])
		*val = min_SHR[priv->mode_index];

	return 0;
}

/*
 * imx274_set_gain - Function called when setting analog gain
 * @priv: Pointer to device structure
 * @val: Value of gain. the real value = val << IMX274_GAIN_SHIFT;
 *
 * Set the analog gain based on input value.
 *
 * Return: 0 on success
 */
static int imx274_set_gain(struct stimx274 *priv, s64 val)
{
	imx274_reg reg_list[2];
	int err;
	u32 gain, gain_reg;
	int i;

	gain = (u32)(val);
	if (gain > IMX274_MAX_GAIN)
		gain = IMX274_MAX_GAIN;
	else if (gain < IMX274_MIN_GAIN)
		gain = IMX274_MIN_GAIN;

	/* convert to register value, refer to imx274 datasheet */
	gain_reg = (u32)2048 - (2048 << IMX274_GAIN_SHIFT) / gain;
	if (gain_reg > IMX274_GAIN_REG_MAX)
		gain_reg = IMX274_GAIN_REG_MAX;

	imx274_calculate_gain_regs(reg_list, (u16)gain_reg);

	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	/* convert register value back to gain value */
	priv->ctrls.gain->val = (2048 << IMX274_GAIN_SHIFT) / (2048 - gain_reg);

	v4l2_dbg(1, debug, &priv->sd,
		 "%s : GAIN control success, new gain = %d\n",
		 __func__, priv->ctrls.gain->val);

	return 0;

fail:
	v4l2_err(&priv->sd, "GAIN control error\n");
	return err;
}

/*
 * imx274_set_coarse_time - Function called when setting SHR value
 * @priv: Pointer to device structure
 * @val: Value for exposure time in number of line_length, or [HMAX]
 *
 * Set SHR value based on input value.
 *
 * Return: 0 on success
 */
static int imx274_set_coarse_time(struct stimx274 *priv, s64 *val)
{
	imx274_reg reg_list[2];
	int err;
	s64 coarse_time, frame_length;
	int i;

	coarse_time = *val;

	/* convert exposure_time to appropriate SHR value */
	err = imx274_clamp_coarse_time(priv, &coarse_time, &frame_length);
	if (err)
		goto fail;

	/* prepare SHR registers */
	imx274_calculate_coarse_time_regs(reg_list, coarse_time);

	/* write to SHR registers */
	for (i = 0; i < 2; i++) {
		err = imx274_write_reg(priv, reg_list[i].addr, reg_list[i].val);
		if (err)
			goto fail;
	}

	*val = frame_length - coarse_time;

	return 0;

fail:
	v4l2_err(&priv->sd, "EXPOSURE control error\n");
	return err;
}

/*
 * imx274_set_exposure - Function called when setting exposure time
 * @priv: Pointer to device structure
 * @val: Variable for exposure time
 *
 * Set exposure time based on input value.
 *
 * Return: 0 on success
 */
static int imx274_set_exposure(struct stimx274 *priv, s64 val)
{
	int err;
	u16 hmax;
	u8 reg_val[2];
	s64 coarse_time; /* exposure time in unit of line (HMAX)*/

	/* step 1: convert input exposure_time (val) into number of 1[HMAX] */

	/* obtain HMAX value */
	err = imx274_read_reg(priv, IMX274_HMAX_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv, IMX274_HMAX_REG_MSB, &reg_val[1]);
	if (err)
		return err;
	hmax = (reg_val[1] << 8) + reg_val[0];

	coarse_time = IMX274_PIXCLK_CONST1 * val / IMX274_PIXCLK_CONST2 / hmax;

	/* step 2: convert exposure_time into SHR value */

	/* set SHR */
	err = imx274_set_coarse_time(priv, &coarse_time);
	if (err)
		goto fail;

	v4l2_dbg(1, debug, &priv->sd,
		 "%s : EXPOSURE control success\n", __func__);

	priv->ctrls.exposure->val = coarse_time * IMX274_PIXCLK_CONST2 * hmax
						/ IMX274_PIXCLK_CONST1;
	return 0;

fail:
	v4l2_err(&priv->sd, "EXPOSURE control error\n");
	return err;
}

/*
 * imx274_set_vflip - Function called when setting vertical flip
 * @priv: Pointer to device structure
 * @val: Value for vflip setting
 *
 * Set vertical flip based on input value.
 * val = 0: normal, no vertical flip
 * val = 1: vertical flip enabled
 *
 * Return: 0 on success
 */
static int imx274_set_vflip(struct stimx274 *priv, int val)
{
	int err;

	err = imx274_write_reg(priv, IMX274_VFLIP_REG, val);
	if (err)
		goto fail;

	v4l2_dbg(1, debug, &priv->sd, "%s : VFLIP control success\n", __func__);
	priv->ctrls.vflip->val = val;
	return 0;

fail:
	v4l2_err(&priv->sd, "VFILP control error\n");
	return err;
}

/*
 * imx274_set_test_pattern - Function called when setting test pattern
 * @priv: Pointer to device structure
 * @val: Variable for test pattern
 *
 * Set to different test patterns based on input value.
 *
 * To come back from test pattern to live video, have to do reset
 *    and restart stream, then load default control values
 *
 * Return: 0 on success
 */
static int imx274_set_test_pattern(struct stimx274 *priv, int val)
{
	int err = 0;

	switch (val) {
	case TEST_PATTERN_DISABLED:
		err = imx274_write_table(priv, imx274_tp_disabled);
		/* reset sensor */
		imx274_reset(priv, 1);
		/* restart stream */
		err = imx274_start_stream(priv, priv->mode_index);
		if (!err)
			err = imx274_load_default(priv);
		break;

	case TEST_PATTERN_GRAY_IMAGE:
		err = imx274_write_table(priv, imx274_tp_gray_image);
		break;

	case TEST_PATTERN_COLOR_BARS:
		err = imx274_write_table(priv, imx274_tp_color_bars);
		break;

	default:
		return -EINVAL;
	}

	if (err)
		goto fail;

	v4l2_dbg(1, debug, &priv->sd,
		 "%s : TEST PATTERN control success\n", __func__);

	priv->ctrls.test_pattern->val = val;
	return 0;

fail:
	v4l2_err(&priv->sd, "TEST PATTERN control error\n");
	return err;
}

/*
 * imx274_set_frame_length - Function called when setting frame length
 * @priv: Pointer to device structure
 * @val: Variable for frame length (= VMAX, i.e. vertical drive period length)
 *
 * Set frame length based on input value.
 *
 * Return: 0 on success
 */
static int imx274_set_frame_length(struct stimx274 *priv, u32 val)
{
	imx274_reg reg_list[3];
	int err;
	u32 frame_length;
	int i;

	v4l2_dbg(1, debug, &priv->sd, "%s : input length = %d\n",
		 __func__, val);

	frame_length = (u32)val;

	imx274_calculate_frame_length_regs(reg_list, frame_length);
	for (i = 0; i < 3; i++) {
		err = imx274_write_reg(priv, reg_list[i].addr,
				       reg_list[i].val);
		if (err)
			goto fail;
	}

	return 0;

fail:
	v4l2_err(&priv->sd, "FRAME_LENGTH control error\n");
	return err;
}

/*
 * __imx274_set_frame_interval - Function called when setting frame interval
 * @priv: Pointer to device structure
 * @frame_interval: Variable for frame interval
 *
 * Change frame interval by altering VMAX value
 *
 * Return: 0 on success
 */
static int __imx274_set_frame_interval(struct stimx274 *priv,
				       struct v4l2_fract frame_interval)
{
	int err;
	s64 frame_length;
	u16 svr;
	u16 hmax;
	u8 reg_val[2];

	/* VMAX = 1/frame_rate x 72M / (SVR+1) / HMAX */
	/* frame_length (i.e. VMAX) = (frame_interval) x 72M /(SVR+1) / HMAX */

	/* read current reg values for SVR, HMAX */

	/* SVR */
	err = imx274_read_reg(priv, IMX274_SVR_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv, IMX274_SVR_REG_MSB, &reg_val[1]);
	if (err)
		goto fail;
	svr = (reg_val[1] << 8) + reg_val[0];
	v4l2_dbg(1, debug, &priv->sd,
		 "%s : register SVR = %dd\n", __func__, svr);

	/* HMAX */
	err = imx274_read_reg(priv, IMX274_HMAX_REG_LSB, &reg_val[0]);
	err |= imx274_read_reg(priv, IMX274_HMAX_REG_MSB, &reg_val[1]);
	if (err)
		goto fail;
	hmax = (reg_val[1] << 8) + reg_val[0];
	v4l2_dbg(1, debug, &priv->sd,
		 "%s : register HMAX = %dd\n", __func__, hmax);

	frame_length = IMX274_PIXCLK_CONST1 / (svr + 1) / hmax
					* frame_interval.numerator
					/ frame_interval.denominator;

	err = imx274_set_frame_length(priv, frame_length);
	if (err)
		goto fail;

	return 0;

fail:
	v4l2_err(&priv->sd, "FRAME_RATE control error\n");
	return err;
}

/*
 * imx274_open - Called on v4l2_open()
 * @sd: Pointer to V4L2 sub device structure
 * @fh: Pointer to V4L2 File handle
 *
 * This function is called on v4l2_open().
 *
 * Return: 0 on success
 */
static int imx274_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int imx274_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

/*
 * Media Operations
 */

static const struct media_entity_operations imx274_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};

static const struct v4l2_subdev_pad_ops imx274_pad_ops = {
	.get_fmt = imx274_get_fmt,
	.set_fmt = imx274_set_fmt,
};

static const struct v4l2_subdev_video_ops imx274_video_ops = {
	.g_frame_interval = imx274_g_frame_interval,
	.s_frame_interval = imx274_s_frame_interval,
	.s_stream = imx274_s_stream,
};

static const struct v4l2_subdev_internal_ops imx274_subdev_internal_ops = {
	.open = imx274_open,
	.close = imx274_close
};

static const struct v4l2_subdev_core_ops imx274_core_ops = {
};

static const struct v4l2_subdev_ops imx274_subdev_ops = {
	.core = &imx274_core_ops,
	.pad = &imx274_pad_ops,
	.video = &imx274_video_ops,
};

static const struct v4l2_ctrl_ops imx274_ctrl_ops = {
	.g_volatile_ctrl = imx274_g_volatile_ctrl,
	.s_ctrl	= imx274_s_ctrl,
};

static const struct of_device_id imx274_of_id_table[] = {
	{ .compatible = "sony,imx274" },
	{ }
};

MODULE_DEVICE_TABLE(of, imx274_of_id_table);
static const struct i2c_device_id imx274_id[] = {
	{ "IMX274", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx274_id);

static int imx274_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct stimx274 *imx274;
	int ret;

	/* initialize imx274 */
	imx274 = devm_kzalloc(&client->dev, sizeof(*imx274), GFP_KERNEL);
	if (!imx274)
		return -ENOMEM;

	mutex_init(&imx274->lock);

	/* initialize regmap */
	imx274->regmap = devm_regmap_init_i2c(client, &imx274_regmap_config);
	if (IS_ERR(imx274->regmap)) {
		dev_err(&client->dev,
			"regmap init failed: %ld\n", PTR_ERR(imx274->regmap));
		return -ENODEV;
	}

	/* initialize subdevice */
	imx274->client = client;
	sd = &imx274->sd;
	v4l2_i2c_subdev_init(sd, client, &imx274_subdev_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));
	sd->internal_ops = &imx274_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* initialize subdev media pad */
	imx274->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx274->pad);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : media entity init Failed %d\n", __func__, ret);
		return ret;
	}

	/* initialize sensor reset gpio */
	imx274->reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx274->reset_gpio)) {
		if (PTR_ERR(imx274->reset_gpio) != -EPROBE_DEFER)
			dev_err(&client->dev, "Reset GPIO not setup in DT");
		return PTR_ERR(imx274->reset_gpio);
	}

	/* pull sensor out of reset */
	imx274_reset(imx274, 1);

	/* initialize controls */
	ret = v4l2_ctrl_handler_init(&imx274->ctrls.handler, 2);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : ctrl handler init Failed\n", __func__);
		goto err_me;
	}

	/* add new controls */
	imx274->ctrls.gain = v4l2_ctrl_new_std(&imx274->ctrls.handler,
		&imx274_ctrl_ops,
		V4L2_CID_GAIN, IMX274_MIN_GAIN,
		IMX274_MAX_GAIN, 1, IMX274_DEF_GAIN);

	imx274->ctrls.exposure = v4l2_ctrl_new_std(&imx274->ctrls.handler,
		&imx274_ctrl_ops,
		V4L2_CID_EXPOSURE, IMX274_MIN_EXPOSURE_TIME,
		1000000 / IMX274_MIN_FRAME_RATE, 1,
		1000000 / IMX274_DEF_FRAME_RATE);

	imx274->ctrls.vflip = v4l2_ctrl_new_std(&imx274->ctrls.handler,
		&imx274_ctrl_ops,
		V4L2_CID_VFLIP, 0, 1, 1, 0);

	imx274->ctrls.test_pattern = v4l2_ctrl_new_std_menu_items(
		&imx274->ctrls.handler, &imx274_ctrl_ops,
		V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(tp_qmenu) - 1, 0, 0, tp_qmenu);

	imx274->sd.ctrl_handler = &imx274->ctrls.handler;
	if (imx274->ctrls.handler.error) {
		ret = imx274->ctrls.handler.error;
		goto err_ctrls;
	}

	/* setup default controls */
	ret = v4l2_ctrl_handler_setup(&imx274->ctrls.handler);
	if (ret) {
		dev_err(&client->dev,
			"Error %d setup default controls\n", ret);
		goto err_ctrls;
	}

	/* initialize format */
	imx274->mode_index = IMX274_MODE_3840X2160;
	imx274->format.width = imx274_formats[0].size.width;
	imx274->format.height = imx274_formats[0].size.height;
	imx274->format.field = V4L2_FIELD_NONE;
	imx274->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	imx274->format.colorspace = V4L2_COLORSPACE_SRGB;
	imx274->frame_interval.numerator = 1;
	imx274->frame_interval.denominator = IMX274_DEF_FRAME_RATE;

	/* register subdevice */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s : v4l2_async_register_subdev failed %d\n",
			__func__, ret);
		return ret;
	}

	v4l2_info(sd, "imx274 : imx274 probe Success !\n");
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(sd->ctrl_handler);
err_me:
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&imx274->lock);
	return ret;
}

static int imx274_remove(struct i2c_client *client)
{
	int ret;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct stimx274 *imx274 = to_imx274(sd);

	/* stop stream */
	ret = imx274_write_table(imx274,
				 mode_table[IMX274_MODE_STOP_STREAM]);
	if (ret)
		return ret;

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);
	return 0;
}

static struct i2c_driver imx274_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= imx274_of_id_table,
	},
	.probe		= imx274_probe,
	.remove		= imx274_remove,
	.id_table	= imx274_id,
};

module_i2c_driver(imx274_i2c_driver);

MODULE_AUTHOR("Leon Luo <leonl@leopardimaging.com>");
MODULE_DESCRIPTION("IMX274 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
