/*  clk-idt24x.c - Program 8T49N24x settings via I2C.
 *
 *  Copyright (C) 2017 by Integrated Device Technologies
 *
 *  08/2016 - Created by David Cater (david.cater@idt.com)
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>

// The configurations in the settings file have 0x317 registers (last offset is 0x316).
#define NUM_CONFIG_REGISTERS 0x317
#define WRITE_BLOCK_SIZE 32
#define DEBUGFS_BUFFER_LENGTH 200

/*
When I go from Q0 25MHz to Q0 75MHz:

2 bytes change in EEPROM data string.

DSM_INT R0025[0],R0026[7:0] : 35 => 30
NS2_Q0 R0040[7:0],R0041[7:0] : 14 => 4

In EEPROM
1. R0026
2. R0041

So that aligns with what I saw above.

Note that VCO_Frequency (metadata) also changed (3500 =>3000).  Presumably DSM_INT.

This is the personality I worked on (before chip overrides) with the workarounds in the workflow
scripts.  That affects NS1_Qx  (x in 1-3) and NS2_Qx. NS1_Qx contains the upper bits of NS_Qx, and NS2_Qx contains
the lower bits.  That is NOT the case for Q0, though.  In that case NS1_Q0 is the 1st stage
output divider (/5, /6, /4) and NS2_Q0 is the 16-bit second stage (with actual divide being 
twice the value stored in the register).

NS1_Q0 R003F[1:0]
*/


// Non output-specific registers
#define IDT24x_REG_DBL_DIS 0x6C
#define IDT24x_REG_DBL_DIS_MASK 0x01
#define IDT24x_REG_DSM_INT_8 0x25
#define IDT24x_REG_DSM_INT_8_MASK 0x01
#define IDT24x_REG_DSM_INT_7_0 0x26
#define IDT24x_REG_DSMFRAC_20_16 0x28
#define IDT24x_REG_DSMFRAC_20_16_MASK 0x1F
#define IDT24x_REG_DSMFRAC_15_8 0x29
#define IDT24x_REG_DSMFRAC_7_0 0x2A
#define IDT24x_REG_OUTEN 0x39
#define IDT24x_REG_Q_DIS 0x6F

// Q0
#define IDT24x_REG_OUTEN0_MASK 0x01
#define IDT24x_REG_Q0_DIS_MASK 0x01
#define IDT24x_REG_NS1_Q0 0x3F
#define IDT24x_REG_NS1_Q0_MASK 0x03
#define IDT24x_REG_NS2_Q0_15_8 0x40
#define IDT24x_REG_NS2_Q0_7_0 0x41

// Q1
#define IDT24x_REG_OUTEN1_MASK 0x02
#define IDT24x_REG_Q1_DIS_MASK 0x02
#define IDT24x_REG_N_Q1_17_16 0x42
#define IDT24x_REG_N_Q1_17_16_MASK 0x03
#define IDT24x_REG_N_Q1_15_8 0x43
#define IDT24x_REG_N_Q1_7_0 0x44
#define IDT24x_REG_NFRAC_Q1_27_24 0x57
#define IDT24x_REG_NFRAC_Q1_27_24_MASK 0x0F
#define IDT24x_REG_NFRAC_Q1_23_16 0x58
#define IDT24x_REG_NFRAC_Q1_15_8 0x59
#define IDT24x_REG_NFRAC_Q1_7_0 0x5A

// Q2
#define IDT24x_REG_OUTEN2_MASK 0x04
#define IDT24x_REG_Q2_DIS_MASK 0x04
#define IDT24x_REG_N_Q2_17_16 0x45
#define IDT24x_REG_N_Q2_17_16_MASK 0x03
#define IDT24x_REG_N_Q2_15_8 0x46
#define IDT24x_REG_N_Q2_7_0 0x47
#define IDT24x_REG_NFRAC_Q2_27_24 0x5B
#define IDT24x_REG_NFRAC_Q2_27_24_MASK 0x0F
#define IDT24x_REG_NFRAC_Q2_23_16 0x5C
#define IDT24x_REG_NFRAC_Q2_15_8 0x5D
#define IDT24x_REG_NFRAC_Q2_7_0 0x5E

// Q3
#define IDT24x_REG_OUTEN3_MASK 0x08
#define IDT24x_REG_Q3_DIS_MASK 0x08
#define IDT24x_REG_N_Q3_17_16 0x48
#define IDT24x_REG_N_Q3_17_16_MASK 0x03
#define IDT24x_REG_N_Q3_15_8 0x49
#define IDT24x_REG_N_Q3_7_0 0x4A
#define IDT24x_REG_NFRAC_Q3_27_24 0x5F
#define IDT24x_REG_NFRAC_Q3_27_24_MASK 0x0F
#define IDT24x_REG_NFRAC_Q3_23_16 0x60
#define IDT24x_REG_NFRAC_Q3_15_8 0x61
#define IDT24x_REG_NFRAC_Q3_7_0 0x62


#define IDT24x_MIN_FREQ		    1000000L
#define IDT24x_MAX_FREQ		    300000000L

#define IDT24x_VCO_MIN          2999997000u
#define IDT24x_VCO_MAX          4000004000u
#define IDT24x_VCO_OPT          3500000000u
#define IDT24x_MIN_INT_DIVIDER  6

#define DRV_NAME			"idt24x"


struct clk_idt24x {
    struct clk_hw hw;
    struct regmap *regmap;
    struct i2c_client *i2c_client;

    // min/max freq for this chip.
    u32 min_freq;
    u32 max_freq;

    u8 settings[NUM_CONFIG_REGISTERS];	// Will be filled in if user specified settings. TODO: remember how to dynamically allocate.
    bool has_settings;					// True if we have settings.  TODO: replace with u8* settings & check for null.

    // input clock reference and notification support
    struct clk *input_clk; 
    struct notifier_block input_clk_rate_change_nb;
    u32 input_clk_freq;

    // user inputs from device tree
    u32 xtal_freq;	// The xtal input freq.  Either input_clk or xtal_freq must be provided.
    bool doubler_disabled; // If xtal is being used, we need to know whether or not the doubler is enabled.  Read this from hw on probe.

    //TODO: support multiple clocks, which means multiple calls to devm_clk_register and multiple struct clk * that I will keep track of.
    // When set_rate is called, I will need to map the hw->clk pointer to the appropriate output number.
    // I believe this will also be the place to keep track of the frequencies that have been requested for the different outputs.

    u32 frequencies[4];	// Requested frequency for each output

    // Register values to read from the hw.  Need to read these so when we write these registers we don't accidentally modify the values
    // we're not setting.
    u8 regDSM_INT_8;
    u8 regDSMFRAC_20_16;
    u8 regOUTENx;
    u8 regQxDIS;
    u8 regNS1_Q0;
    u8 regN_Qx_17_16[3];
    u8 regNFRAC_Qx_27_24[3];


    struct dentry *debugfs_dirroot, *debugfs_fileaction, *debugfs_map;
    char idt24x_ker_buf[DEBUGFS_BUFFER_LENGTH];
    struct dentry *debugfs_fileclkfreq[4];
    u64 debugfs_frequencies[4];
};

struct clk_idt24x *idt24x_data_fordebugfs;

#define to_clk_idt24x(_hw)	container_of(_hw, struct clk_idt24x, hw)
#define to_clk_idt24x_from_client(_client) container_of(_client, struct clk_idt24x, i2c_client)
#define to_clk_idt24x_from_nb(_nb)	container_of(_nb, struct clk_idt24x, input_clk_rate_change_nb)

struct clk_register_offsets {
    u16 oe_offset;
    u8 oe_mask;
    u16 dis_offset;
    u8 dis_mask;

    // For N_Qx (fractional output)
    u16 n_17_16_offset;
    u8 n_17_16_mask;
    u16 n_15_8_offset;
    u16 n_7_0_offset;
    u16 nfrac_27_24_offset;
    u8 nfrac_27_24_mask;
    u16 nfrac_23_16_offset;
    u16 nfrac_15_8_offset;
    u16 nfrac_7_0_offset;

    // For NS_Qx (two-stage divider, integer output)
    u16 ns1_offset;
    u8 ns1_offset_mask;
    u16 ns2_15_8_offset;
    u16 ns2_7_0_offset;
};

struct idt24x_dividers {
    u16 dsmint;
    u32 dsmfrac;

    u8 ns1_q0;
    u16 ns2_q0;

    u32 nint[3];  // Q1-3
    u32 nfrac[3]; // Q1-3
};

enum clk_idt24x_variant {
    idt24x
};

static int bits_to_shift(unsigned int mask) {
    // mask is 32-bit word input to count zero bits on right
    unsigned int c = 32; // c will be the number of zero bits on the right
    mask &= ~mask + 1;
    if (mask) c--;
    if (mask & 0x0000FFFF) c -= 16;
    if (mask & 0x00FF00FF) c -= 8;
    if (mask & 0x0F0F0F0F) c -= 4;
    if (mask & 0x33333333) c -= 2;
    if (mask & 0x55555555) c -= 1;
    return c;
}

static u32 mask_and_shift(u32 value, u8 mask) {
    value &= mask;
    return value >> bits_to_shift(mask);
}

static int idt24x_get_offsets(u8 output_num, struct clk_register_offsets *offsets) {
    switch(output_num) {
        case 0:
            offsets->oe_offset = IDT24x_REG_OUTEN;
            offsets->oe_mask = IDT24x_REG_OUTEN0_MASK;
            offsets->dis_offset = IDT24x_REG_Q_DIS;
            offsets->dis_mask = IDT24x_REG_Q0_DIS_MASK;
            offsets->ns1_offset = IDT24x_REG_NS1_Q0;
            offsets->ns1_offset_mask = IDT24x_REG_NS1_Q0_MASK;
            offsets->ns2_15_8_offset = IDT24x_REG_NS2_Q0_15_8;
            offsets->ns2_7_0_offset = IDT24x_REG_NS2_Q0_7_0;
            break;
        case 1:
            offsets->oe_offset = IDT24x_REG_OUTEN;
            offsets->oe_mask = IDT24x_REG_OUTEN1_MASK;
            offsets->dis_offset = IDT24x_REG_Q_DIS;
            offsets->dis_mask = IDT24x_REG_Q1_DIS_MASK;
            offsets->n_17_16_offset = IDT24x_REG_N_Q1_17_16;
            offsets->n_17_16_mask = IDT24x_REG_N_Q1_17_16_MASK;
            offsets->n_15_8_offset = IDT24x_REG_N_Q1_15_8;
            offsets->n_7_0_offset = IDT24x_REG_N_Q1_7_0;
            offsets->nfrac_27_24_offset= IDT24x_REG_NFRAC_Q1_27_24;
            offsets->nfrac_27_24_mask = IDT24x_REG_NFRAC_Q1_27_24_MASK;
            offsets->nfrac_23_16_offset = IDT24x_REG_NFRAC_Q1_23_16;
            offsets->nfrac_15_8_offset = IDT24x_REG_NFRAC_Q1_15_8;
            offsets->nfrac_7_0_offset = IDT24x_REG_NFRAC_Q1_7_0;
            break;
        case 2:
            offsets->oe_offset = IDT24x_REG_OUTEN;
            offsets->oe_mask = IDT24x_REG_OUTEN2_MASK;
            offsets->dis_offset = IDT24x_REG_Q_DIS;
            offsets->dis_mask = IDT24x_REG_Q2_DIS_MASK;
            offsets->n_17_16_offset = IDT24x_REG_N_Q2_17_16;
            offsets->n_17_16_mask = IDT24x_REG_N_Q2_17_16_MASK;
            offsets->n_15_8_offset = IDT24x_REG_N_Q2_15_8;
            offsets->n_7_0_offset = IDT24x_REG_N_Q2_7_0;
            offsets->nfrac_27_24_offset = IDT24x_REG_NFRAC_Q2_27_24;
            offsets->nfrac_27_24_mask = IDT24x_REG_NFRAC_Q2_27_24_MASK;
            offsets->nfrac_23_16_offset = IDT24x_REG_NFRAC_Q2_23_16;
            offsets->nfrac_15_8_offset = IDT24x_REG_NFRAC_Q2_15_8;
            offsets->nfrac_7_0_offset = IDT24x_REG_NFRAC_Q2_7_0;
            break;
        case 3:
            offsets->oe_offset = IDT24x_REG_OUTEN;
            offsets->oe_mask = IDT24x_REG_OUTEN3_MASK;
            offsets->dis_offset = IDT24x_REG_Q_DIS;
            offsets->dis_mask = IDT24x_REG_Q3_DIS_MASK;
            offsets->n_17_16_offset = IDT24x_REG_N_Q3_17_16;
            offsets->n_17_16_mask = IDT24x_REG_N_Q3_17_16_MASK;
            offsets->n_15_8_offset = IDT24x_REG_N_Q3_15_8;
            offsets->n_7_0_offset = IDT24x_REG_N_Q3_7_0;
            offsets->nfrac_27_24_offset = IDT24x_REG_NFRAC_Q3_27_24;
            offsets->nfrac_27_24_mask = IDT24x_REG_NFRAC_Q3_27_24_MASK;
            offsets->nfrac_23_16_offset = IDT24x_REG_NFRAC_Q3_23_16;
            offsets->nfrac_15_8_offset = IDT24x_REG_NFRAC_Q3_15_8;
            offsets->nfrac_7_0_offset = IDT24x_REG_NFRAC_Q3_7_0;
            break;
        default:
            return -EINVAL;
            break;
    }
    return 0;
}

static int regmapBulkWriteWithRetry(struct regmap *map, unsigned int offset, u8 val[], int val_count, int maxAttempts) {
	int err = 0;
	int count = 1;
	do {
		err = regmap_bulk_write(map, offset, val, val_count);
		if (err == 0) return 0;

		usleep_range(100, 200);
	} while (count++ <= maxAttempts);
	return err;
}


static int regmapWriteWithRetry(struct regmap *map, unsigned int offset, unsigned int val, int maxAttempts) {
	int err = 0;
	int count = 1;
	do {
		err = regmap_write(map, offset, val);
		if (err == 0) return 0;
		usleep_range(100, 200);
	} while (count++ <= maxAttempts);
	return err;
}


static int i2cwritebulk(struct i2c_client *client, struct regmap *map, unsigned int reg, u8 val[], size_t val_count) {
    char dbg[128];
	u8 block[WRITE_BLOCK_SIZE];
	unsigned int blockOffset = reg;
    
    int x;
    int err = 0;
	int currentOffset = 0;
	sprintf(dbg, "I2C->0x%04x : [hex] . First byte: %02x, Second byte: %02x", reg, reg >> 8, reg & 0xFF);
    dev_info(&client->dev, dbg);
	dbg[0] = 0;

    for (x = 0; x < val_count; x++) {
        char data[4];
		block[currentOffset++] = val[x];
        sprintf(data, "%02x ", val[x]);
        strcat(dbg, data);
		if (x > 0 && (x+1) % WRITE_BLOCK_SIZE == 0) {
		    dev_info(&client->dev, dbg);
			dbg[0] = '\0';
			sprintf(dbg, "(loop) calling regmap_bulk_write @ 0x%04x [%d bytes]", blockOffset, WRITE_BLOCK_SIZE);
			dev_info(&client->dev, dbg);
			dbg[0] = '\0';
			err = regmapBulkWriteWithRetry(map, blockOffset, block, WRITE_BLOCK_SIZE, 5);
			if (err != 0) break;
			blockOffset += WRITE_BLOCK_SIZE;
			currentOffset = 0;
		}
    }
	if (err == 0 && currentOffset > 0) {
	    dev_info(&client->dev, dbg);
		sprintf(dbg, "(final) calling regmap_bulk_write @ 0x%04x [%d bytes]", blockOffset, currentOffset);
		dev_info(&client->dev, dbg);
		err = regmapBulkWriteWithRetry(map, blockOffset, block, currentOffset, 5);
	}

    return err;
}

static int i2cwrite(struct i2c_client *client, struct regmap *map, unsigned int reg, unsigned int val) {
    int err;
    dev_info(&client->dev, "I2C->0x%x : [hex] %x", reg, val);
    err = regmapWriteWithRetry(map, reg, val, 5);
    usleep_range(100, 200);
    return err;
}

static int i2cwritewithmask(struct i2c_client *client, struct regmap *map, unsigned int reg, u8 val, u8 original, u8 mask) {
    return i2cwrite(client, map, reg, ((val << bits_to_shift(mask)) & mask) | (original & ~mask));
}

static int idt24x_read_all_settings(struct clk_idt24x *data, char* output_buffer, int count) {
	u8 settings[NUM_CONFIG_REGISTERS];
	int err = 0;
	int x;

	err = regmap_bulk_read(data->regmap, 0x0, settings, NUM_CONFIG_REGISTERS);
	if (!err) {
		output_buffer[0] = '\0';
		for (x = 0; x < ARRAY_SIZE(settings); x++) {
			char dbg[4];
			if ((strlen(output_buffer) + 4) > count) {
				return -EINVAL;
			}
			sprintf(dbg, "%02x ", settings[x]);
			strcat(output_buffer, dbg);
		}
	}
	return err;
}

/**
 * idt24x_read_from_hw() - Get the current values on the hw
 * @data:	Driver data structure
 * @fout:	Factory frequency output
 * Returns 0 on success, negative errno otherwise.
 */
static int idt24x_read_from_hw(struct clk_idt24x *data)
{
    int err;
    struct i2c_client *client = data->i2c_client;
    u32 tmp;
	u8 output;
	bool verbose = true;

    err = regmap_read(data->regmap, IDT24x_REG_DSM_INT_8, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_DSM_INT_8: %i", err);
        return err;
    }
    data->regDSM_INT_8 = tmp;
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regDSM_INT_8: 0x%x", data->regDSM_INT_8);

    err = regmap_read(data->regmap, IDT24x_REG_DSMFRAC_20_16_MASK, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_DSMFRAC_20_16_MASK: %i", err);
        return err;
    }
    data->regDSMFRAC_20_16 = tmp;
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regDSMFRAC_20_16: 0x%x", data->regDSMFRAC_20_16);


    err = regmap_read(data->regmap, IDT24x_REG_OUTEN, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_OUTEN: %i", err);
        return err;
    }
    data->regOUTENx = tmp;
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regOUTENx: 0x%x", data->regOUTENx);


    err = regmap_read(data->regmap, IDT24x_REG_Q_DIS, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_Q_DIS: %i", err);
        return err;
    }
    data->regQxDIS = tmp;
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regQxDIS: 0x%x", data->regQxDIS);


    err = regmap_read(data->regmap, IDT24x_REG_NS1_Q0, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_NS1_Q0: %i", err);
        return err;
    }
    data->regNS1_Q0 = tmp;
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regNS1_Q0: 0x%x", data->regNS1_Q0);

    for (output = 1; output <= 3; output++) {
        struct clk_register_offsets offsets;
        err = idt24x_get_offsets(output, &offsets);
        if (err) {
            dev_err(&client->dev, "idt24x_read_from_hw: error calling idt24x_get_offsets: %i", err);
            return err;
        }

        err = regmap_read(data->regmap, offsets.n_17_16_offset, &tmp);
        if (err) {
            dev_err(&client->dev, "idt24x_read_from_hw: error reading n_17_16_offset for output %d (offset: 0x%x): %i", output, offsets.n_17_16_offset, err);
            return err;
        }
        data->regN_Qx_17_16[output-1] = tmp;
		if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regN_Qx_17_16[Q%u]: 0x%x", output, data->regN_Qx_17_16[output-1]);

        err = regmap_read(data->regmap, offsets.nfrac_27_24_offset, &tmp);
        if (err) {
            dev_err(&client->dev, "idt24x_read_from_hw: error reading nfrac_27_24_offset for output %d (offset: 0x%x): %i", output, offsets.nfrac_27_24_offset, err);
            return err;
        }
        data->regNFRAC_Qx_27_24[output-1] = tmp;
		if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: regNFRAC_Qx_27_24[Q%u]: 0x%x", output, data->regNFRAC_Qx_27_24[output-1]);
    }

    dev_info(&client->dev, "idt24x_read_from_hw: initial values read from chip successfully");

    // I believe the only other thing we need is DBL_DIS, and we only need that if xtal-freq is specified.
    if (0 == data->xtal_freq) {
        return err;
    }

    err = regmap_read(data->regmap, IDT24x_REG_DBL_DIS, &tmp);
    if (err) {
        dev_err(&client->dev, "idt24x_read_from_hw: error reading IDT24x_REG_DBL_DIS: %i", err);
        return err;
    }
    data->doubler_disabled = mask_and_shift(tmp, IDT24x_REG_DBL_DIS_MASK);
	if (verbose) dev_info(&client->dev, "idt24x_read_from_hw: doubler_disabled: %d", data->doubler_disabled);
    

    return 0;
}

/**
 * idt24x_calc_divs() - Calculate dividers to generate the specified frequency.
 * @data:			driver data structure. contains all requested frequencies for all outputs.
 * @divs:	        divider structure for returning all calculatiosn (idt24x_dividers)
 *
 * Returns 0 on success, negative errno otherwise.
 *
 * Calculate the clock dividers (dsmint, dsmfrac for vco; ns1/ns2 for q0, 
 * n/nfrac for q1-3) for a given target frequency.
 */
static int idt24x_calc_divs(struct clk_idt24x *data, struct idt24x_dividers *divs)
{
    u64 rem;
    u32 min_div, max_div;
    u32 walk, div = 0;
    u32 vco, freq;
	bool is_lower_vco = false;
	u32 best_vco = 0;
    
    //TODO: Calculate the appropriate dividers for all requested frequencies, not just Q2.

    if (data->frequencies[0] != 0) 
        dev_err(&data->i2c_client->dev, "idt24x_calc_divs: Q0 NOT IMPLEMENTED");
    if (data->frequencies[1] != 0) 
        dev_err(&data->i2c_client->dev, "idt24x_calc_divs: Q1 NOT IMPLEMENTED");
    if (data->frequencies[3] != 0) 
        dev_err(&data->i2c_client->dev, "idt24x_calc_divs: Q3 NOT IMPLEMENTED");

    freq = data->frequencies[2];
    if (freq == 0) {
        dev_err(&data->i2c_client->dev, "idt24x_calc_divs: Q2 NOT SPECIFIED");
        return -EINVAL;
    }

    divs->dsmint = 0;
    divs->dsmfrac = 0;

    divs->ns1_q0 = 0;
    divs->ns2_q0 = 0;

    // First, determine the minimum divider for the output frequency. 
    min_div = IDT24x_MIN_INT_DIVIDER;
    //u32 max_div = math.ceil(IDT24x_VCO_MAX / freq / 2) * 2
    max_div = div64_u64((u64)IDT24x_VCO_MAX, freq * 2) * 2;

    dev_info(&data->i2c_client->dev, "calc_divs. min_div: %u, max_div: %u", min_div, max_div);

    walk = min_div;

    while (walk <= max_div) {
        u32 vco = freq * walk;
	    dev_info(&data->i2c_client->dev, "calc_divs. walk: %u, freq: %u, vco: %u", walk, freq, vco);
        if (vco >= IDT24x_VCO_MIN && vco <= IDT24x_VCO_MAX) {
/*
            if (freq<=Fields.VCOoptimization.DefaultValue):
                if ((freq>best_freq)or(not is_lower_vco)):
                    final_option=option
                    is_lower_vco=True
                    best_freq=freq
            elif (not is_lower_vco):
                if (freq>best_freq):
                    final_option=option
                    best_freq=freq
*/			
			if (vco <= IDT24x_VCO_OPT) {
				if (vco > best_vco || !is_lower_vco) {
					is_lower_vco = true;
					div = walk;
					best_vco = vco;
				}
			} else if (!is_lower_vco) {
				if (vco > best_vco) {
					div = walk;
					best_vco = vco;
				}
			}
        }
        // Must be even.
        walk += 2;
    }

    if (div != 0) {
        // Found a divider in range.
        u32 pfd;
		// The value written to the chip is half the calculated divider.
        divs->nint[1] = div64_u64((u64)div, 2);
        divs->nfrac[1] = 0;
        vco = div * freq;
        pfd = (data->input_clk_freq == 0 ? data->xtal_freq : data->input_clk_freq) * (data->doubler_disabled ? 1 : 2);
		
		// dsm = vco/pfd
		// dsmfrac = dsm-floot(dsm) * 2^21
		// rem = vco % pfd
		// therefore:
		// dsmfrac = (rem * 2^21)/pfd
        divs->dsmint = div64_u64_rem(vco, pfd, &rem);
		divs->dsmfrac = div64_u64(rem * 1<<21, pfd);

        dev_info(&data->i2c_client->dev, "calc_divs. integer div: %u, frac div: %u, vco: %u, pfd: %u, dsmint: %u, dsmfrac: %u, rem: %llu", 
            divs->nint[1], divs->nfrac[1], vco, pfd, divs->dsmint, divs->dsmfrac, rem);
    }
    else {
        dev_err(&data->i2c_client->dev, "idt24x_calc_divs: no integer divider in range found. This case isn't supported yet.");
        return -EINVAL;
    }
    return 0;
}

/**
 * idt24x_enable_output() - Enable/disable a particular output
 * @data:		Driver data structure
 * @output: 	Output to enable/disable
 * @enable:		Enable (true/false)
 * Passes on regmap_write() return value.
 */
static int idt24x_enable_output(struct clk_idt24x *data, u8 output, bool enable) {
	// When we enable an output, make sure we enable it in the original data we read from the chip and 
	// cached as well, or else we will accidentally turn off outputs.  E.g., we start with all outputs off
	// in regOUTENx, enable Q1 with the appropriate mask, and then turn Q1 back off when we turn Q2 on (because
	// Q1 was off in regOUTENx).

    struct clk_register_offsets offsets;
	int err;
	bool verbose = true;
	struct i2c_client *client = data->i2c_client;

    err = idt24x_get_offsets(output, &offsets);
    if (err) {
        dev_err(&client->dev, "idt24x_enable_output: error calling idt24x_get_offsets for %d: %i", output, err);
        return err;
    }

	if (verbose) 
		dev_info(&client->dev, "idt24x_enable_output: q%u enable? %d. regOUTENx before: 0x%x, regQxDIS before: 0x%x", output, 
			enable, data->regOUTENx, data->regQxDIS);

	data->regOUTENx = data->regOUTENx & ~offsets.oe_mask;
	if (enable) data->regOUTENx |= (1 << bits_to_shift(offsets.oe_mask));

	data->regQxDIS = data->regQxDIS & ~offsets.dis_mask;
	if (verbose) 
		dev_info(&client->dev, "idt24x_enable_output: q%u enable? %d. regQxDIS mask: 0x%x, before checking enable: 0x%x", output, 
			enable, offsets.dis_mask, data->regQxDIS);
	if (!enable) data->regQxDIS |= (1 << bits_to_shift(offsets.dis_mask));

	if (verbose) 
		dev_info(&client->dev, "idt24x_enable_output: q%u enable? %d. regOUTENx after: 0x%x, regQxDIS after: 0x%x", output, 
			enable, data->regOUTENx, data->regQxDIS);

    err = i2cwrite(client, data->regmap, IDT24x_REG_OUTEN, data->regOUTENx);
    if (err) {
        dev_err(&client->dev, "idt24x_enable_output: error setting IDT24x_REG_OUTEN: %i", err);
        return err;
    }

    err = i2cwrite(client, data->regmap, IDT24x_REG_Q_DIS, data->regQxDIS);
    if (err) {
        dev_err(&client->dev, "idt24x_enable_output: error setting IDT24x_REG_Q_DIS: %i", err);
        return err;
    }

 	return 0;
}

/**
 * idt24x_update_device() - Write all values to hardware that we have calculated.
 * @data:		Driver data structure
 * @divs:		Dividers data structure. Contains both VCO divs and divs for all outputs.
 * Passes on regmap_bulk_write() return value.
 */
static int idt24x_update_device(struct clk_idt24x *data, struct idt24x_dividers *divs)
{
    int err;
	struct i2c_client *client = data->i2c_client;
	int x = -1;
	bool verbose = true;

    dev_info(&client->dev, "idt24x_update_device. integer div[1]: %u, frac div[1]: %u", divs->nint[1], divs->nfrac[1]);

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting DSM_INT_8 (val %u @ %u)", divs->dsmint >> 8, IDT24x_REG_DSM_INT_8);
    // First update DSMINT and DSMFRAC.
    err = i2cwritewithmask(
        client, data->regmap, IDT24x_REG_DSM_INT_8, 
        (divs->dsmint >> 8) & IDT24x_REG_DSM_INT_8_MASK, data->regDSM_INT_8, IDT24x_REG_DSM_INT_8_MASK);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_DSM_INT_8: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting DSM_INT_7_0 (val %u @ 0x%x)", divs->dsmint & 0xFF, IDT24x_REG_DSM_INT_7_0);
    err = i2cwrite(client, data->regmap, IDT24x_REG_DSM_INT_7_0, divs->dsmint & 0xFF);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_DSM_INT_7_0: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_DSMFRAC_20_16 (val %u @ 0x%x)", divs->dsmfrac >> 16, IDT24x_REG_DSMFRAC_20_16);
    err = i2cwritewithmask(
        client, data->regmap, IDT24x_REG_DSMFRAC_20_16, 
        (divs->dsmfrac >> 16) & IDT24x_REG_DSMFRAC_20_16_MASK, data->regDSM_INT_8, IDT24x_REG_DSMFRAC_20_16_MASK);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_DSMFRAC_20_16: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_DSMFRAC_15_8 (val %u @ 0x%x)", (divs->dsmfrac >> 8) & 0xFF, IDT24x_REG_DSMFRAC_15_8);
    err = i2cwrite(client, data->regmap, IDT24x_REG_DSMFRAC_15_8, (divs->dsmfrac >> 8) & 0xFF);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_DSMFRAC_15_8: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_DSMFRAC_7_0 (val %u @ 0x%x)", divs->dsmfrac & 0xFF, IDT24x_REG_DSMFRAC_7_0);
    err = i2cwrite(client, data->regmap, IDT24x_REG_DSMFRAC_7_0, divs->dsmfrac & 0xFF);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_DSMFRAC_7_0: %i", err);
        return err;
    }


    // Now update dividers.
	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_NS1_Q0 (val %u @ 0x%x)", divs->ns1_q0 >> 8, IDT24x_REG_NS1_Q0);
    err = i2cwritewithmask(
        client, data->regmap, IDT24x_REG_NS1_Q0,
        (divs->ns1_q0 >> 8) & IDT24x_REG_NS1_Q0_MASK, data->regNS1_Q0, IDT24x_REG_NS1_Q0_MASK);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_NS1_Q0: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_NS2_Q0_15_8 (val %u @ 0x%x)", (divs->ns2_q0 >> 8) & 0xFF, IDT24x_REG_NS2_Q0_15_8);
    err = i2cwrite(client, data->regmap, IDT24x_REG_NS2_Q0_15_8, (divs->ns2_q0 >> 8) & 0xFF);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_NS2_Q0_15_8: %i", err);
        return err;
    }

	if (verbose) dev_info(&client->dev, "idt24x_update_device: setting IDT24x_REG_NS2_Q0_7_0 (val %u @ 0x%x)", divs->ns2_q0 & 0xFF, IDT24x_REG_NS2_Q0_7_0);
    err = i2cwrite(client, data->regmap, IDT24x_REG_NS2_Q0_7_0, divs->ns2_q0 & 0xFF);
    if (err) {
        dev_err(&client->dev, "idt24x_update_device: error setting IDT24x_REG_NS2_Q0_7_0: %i", err);
        return err;
    }

	idt24x_enable_output(data, 0, data->frequencies[0] != 0);

	if (verbose) dev_info(&client->dev, "idt24x_update_device: writing values for q1-q3");
    for (x = 1; x <= 3; x++) {
        struct clk_register_offsets offsets;

		if (data->frequencies[x] != 0) {
		 	if (verbose) dev_info(&client->dev, "idt24x_update_device: calling idt24x_get_offsets for %u", x);
		    err = idt24x_get_offsets(x, &offsets);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error calling idt24x_get_offsets: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: (q%u, nint: %u, nfrac: %u)", x, divs->nint[x-1], divs->nfrac[x-1]);


			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting n_17_16_offset (q%u, val %u @ 0x%x)", x, divs->nint[x-1] >> 16, offsets.n_17_16_offset);
		    err = i2cwritewithmask(
		        client, data->regmap, offsets.n_17_16_offset,
		        (divs->nint[x-1] >> 16) & offsets.n_17_16_mask, data->regN_Qx_17_16[x-1], offsets.n_17_16_mask);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting n_17_16_offset: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting n_15_8_offset (q%u, val %u @ 0x%x)", x, (divs->nint[x-1] >> 8) & 0xFF, offsets.n_15_8_offset);
		    err = i2cwrite(client, data->regmap, offsets.n_15_8_offset, (divs->nint[x-1] >> 8) & 0xFF);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting n_15_8_offset: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting n_7_0_offset (q%u, val %u @ 0x%x)", x, divs->nint[x-1] & 0xFF, offsets.n_7_0_offset);
		    err = i2cwrite(client, data->regmap, offsets.n_7_0_offset, divs->nint[x-1] & 0xFF);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting n_7_0_offset: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting nfrac_27_24_offset (q%u, val %u @ 0x%x)", x, (divs->nfrac[x-1] >> 24), offsets.nfrac_27_24_offset);
		    err = i2cwritewithmask(
		        client, data->regmap, offsets.nfrac_27_24_offset,
		        (divs->nfrac[x-1] >> 24) & offsets.nfrac_27_24_mask, data->regNFRAC_Qx_27_24[x-1], offsets.nfrac_27_24_mask);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting nfrac_27_24_offset: %i", err);
		        return err;
		    }


			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting nfrac_23_16_offset (q%u, val %u @ 0x%x)", x, (divs->nfrac[x-1] >> 16) & 0xFF, offsets.nfrac_23_16_offset);
		    err = i2cwrite(client, data->regmap, offsets.nfrac_23_16_offset, (divs->nfrac[x-1] >> 16) & 0xFF);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting nfrac_23_16_offset: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting nfrac_15_8_offset (q%u, val %u @ 0x%x)", x, (divs->nfrac[x-1] >> 8) & 0xFF, offsets.nfrac_15_8_offset);
		    err = i2cwrite(client, data->regmap, offsets.nfrac_15_8_offset, (divs->nfrac[x-1] >> 8) & 0xFF);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting nfrac_15_8_offset: %i", err);
		        return err;
		    }

			if (verbose) dev_info(&client->dev, "idt24x_update_device: setting nfrac_7_0_offset (q%u, val %u @ 0x%x)", x, divs->nfrac[x-1] & 0xFF, offsets.nfrac_7_0_offset);
		    err = i2cwrite(client, data->regmap, offsets.nfrac_7_0_offset, divs->nfrac[x-1] & 0xFF);
		    if (err) {
		        dev_err(&client->dev, "idt24x_update_device: error setting nfrac_7_0_offset: %i", err);
		        return err;
		    }
		}
		idt24x_enable_output(data, x, data->frequencies[x] != 0);
    }
    return 0;
}

/**
 * idt24x_set_frequency() - Adjust output frequency
 * @data:       Driver data structure
 * @frequency:	Target frequency
 * Returns 0 on success.
 *
 * Update output frequency for big frequency changes (> 3,500 ppm).
 */
static int idt24x_set_frequency(struct clk_idt24x *data)
{
    int err;
    struct i2c_client *client = data->i2c_client;
    struct idt24x_dividers divs;

    if (0 == data->frequencies[2]) {
        idt24x_enable_output(data, 2, false);
        return 0;
    }

    if (0 == data->input_clk_freq && 0 == data->xtal_freq) {
        dev_err(&client->dev, "set_frequency: no input frequency; can't continue.");
        return -EINVAL;
    }

    err = idt24x_calc_divs(data, &divs);
    if (err) {
        dev_err(&client->dev, "set_frequency: error calling idt24x_calc_divs: %i", err);
        return err;
    } else {
        dev_info(&client->dev, "set_frequency. q2 divs: integer div: %u, frac div: %u", divs.nint[1], divs.nfrac[1]);
	}

    err = idt24x_update_device(data, &divs);
    if (err) {
        dev_err(&client->dev, "set_frequency: error updating the device: %i", err);
        return err;
    }

    return 0;
}


static u8 idt24x_get_output_num(struct clk *clk) {
    return 2;
}

static int idt24x_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
    int err = 0;
    struct clk_idt24x *data = to_clk_idt24x(hw);
    struct i2c_client *client = data->i2c_client;
    u8 output_num = idt24x_get_output_num(hw->clk);

    if (rate < data->min_freq || rate > data->max_freq) {
        dev_err(&client->dev, "requested frequency (%luHz) is out of range\n", rate);
        return -EINVAL;
    }

    //TODO: hw->clk is the pointer to the clock the user is requesting. I believe I will need a way to figure out
    // which clock that is (Q0-Q3) from the pointer.  Then I will need to set that rate in the data structure.
	// For now, just set it in Q2, and then call idt24x_set_frequency.  One disadvantage there; if there is an error,
	// we have already set frequencies[x] and replaced the old frequency.
    data->frequencies[output_num] = rate;
    data->debugfs_frequencies[output_num] = rate;

    err = idt24x_set_frequency(data);

    if (err != 0) 
    	dev_err(&client->dev, "error calling set_frequency: %d", err);

    return err;
}




static long idt24x_round_rate(struct clk_hw *hw, unsigned long rate,
    unsigned long *parent_rate)
{
/*
 * @round_rate:	Given a target rate as input, returns the closest rate actually
 *		supported by the clock. The parent rate is an input/output
 *		parameter.
*/
	//TODO: Figure out the closest rate that we can support within a low error threshold and return that rate.
    return rate;
}

static unsigned long idt24x_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
    struct clk_idt24x *data = to_clk_idt24x(hw);
	//TODO: add support for multiple clocks
	return data->frequencies[2];
}



// .recalc_rate is used to read the current values from the hardware and report the frequency 
// being provided by the clock. Without that, the clock will be initialized to 0 by default.  
// In the current use case, .set_rate will always be called.  So recalc_rate is not needed in the initial version.
// However, I'm getting an error when I don't include it, and the driver won't register.  The error says that 
// round_rate is required if recalc_rate is specified, which doesn't make sense.  But I'm adding back .recalc_rate
// anyway.
static const struct clk_ops idt24x_clk_ops = {
    .recalc_rate = idt24x_recalc_rate,
    .round_rate = idt24x_round_rate,
    .set_rate = idt24x_set_rate,
};

static bool idt24x_regmap_is_volatile(struct device *dev, unsigned int reg)
{
    return false;
}

static bool idt24x_regmap_is_writeable(struct device *dev, unsigned int reg)
{
    return true;
}

static const struct regmap_config idt24x_regmap_config = {
    .reg_bits = 16,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE,
    .max_register = 0xff,
    .writeable_reg = idt24x_regmap_is_writeable,
    .volatile_reg = idt24x_regmap_is_volatile,
};

/**
 * idt24x_clk_notifier_cb - Clock rate change callback
 * @nb:		Pointer to notifier block
 * @event:	Notification reason
 * @data:	Pointer to notification data object
 *
 * This function is called when the input clock frequency changes.
 * The callback checks whether a valid bus frequency can be generated after the
 * change. If so, the change is acknowledged, otherwise the change is aborted.
 * New dividers are written to the HW in the pre- or post change notification
 * depending on the scaling direction.
 *
 * Return:	NOTIFY_STOP if the rate change should be aborted, NOTIFY_OK
 *		to acknowledge the change, NOTIFY_DONE if the notification is
 *		considered irrelevant.
 */
static int idt24x_clk_notifier_cb(struct notifier_block *nb, unsigned long event, void *data) {
    struct clk_notifier_data *ndata = data;
    struct clk_idt24x *idt = to_clk_idt24x_from_nb(nb);
    int err = 0;

    dev_info(&idt->i2c_client->dev, "idt24x_clk_notifier_cb: input frequency changed: %lu Hz. event: %lu", ndata->new_rate, event);

    switch (event) {
        case PRE_RATE_CHANGE: {
            dev_info(&idt->i2c_client->dev, "PRE_RATE_CHANGE\n");
#if 0
            unsigned long input_clk = ndata->new_rate;
            unsigned long fscl = id->i2c_clk;
            unsigned int div_a, div_b;
            int ret;

            ret = cdns_i2c_calc_divs(&fscl, input_clk, &div_a, &div_b);
            if (ret) {
                dev_warn(id->adap.dev.parent,
                        "clock rate change rejected\n");
                return NOTIFY_STOP;
            }

            /* scale up */
            if (ndata->new_rate > ndata->old_rate)
                cdns_i2c_setclk(ndata->new_rate, id);
#endif
            return NOTIFY_OK;
        }
        case POST_RATE_CHANGE:
            idt->input_clk_freq = ndata->new_rate;
            // Can't call clock API clk_set_rate here; I believe it will be ignored if the rate is 
            // the same as we set previously.  Need to call our internal function.
            dev_info(&idt->i2c_client->dev, "POST_RATE_CHANGE. Calling idt24x_set_frequency\n");
            err = idt24x_set_frequency(idt);
            if (err) 
                dev_err(&idt->i2c_client->dev, "error calling idt24x_set_frequency (%i)\n", err);
            return NOTIFY_OK;
        case ABORT_RATE_CHANGE:
#if 0
            /* scale up */
            if (ndata->new_rate > ndata->old_rate)
                cdns_i2c_setclk(ndata->old_rate, id);
#endif
            return NOTIFY_OK;
        default:
            return NOTIFY_DONE;
    }
}
/* read file operation */
static ssize_t idt24x_debugfs_reader_action(struct file *fp, char __user *user_buffer, size_t count, loff_t *position) {
	return simple_read_from_buffer(user_buffer, count, position, idt24x_data_fordebugfs->idt24x_ker_buf, DEBUGFS_BUFFER_LENGTH);
}
static ssize_t idt24x_debugfs_reader_map(struct file *fp, char __user *user_buffer, size_t count, loff_t *position) {
	int err= 0;
	char* buf = devm_kzalloc(&idt24x_data_fordebugfs->i2c_client->dev, 5000, GFP_KERNEL);
	dev_info(&idt24x_data_fordebugfs->i2c_client->dev, "calling idt24x_debugfs_reader_map (count: %u)\n", count);
	err = idt24x_read_all_settings(idt24x_data_fordebugfs, buf, 5000);
	if (err) {
		dev_err(&idt24x_data_fordebugfs->i2c_client->dev, "error calling idt24x_read_all_settings (%i)\n", err);
		return 0;
	}
        // TMGCDR-1456. TODO: We're returning 1 byte too few.
	return simple_read_from_buffer(user_buffer, count, position, buf, strlen(buf));
}
 
/* write file operation */
static ssize_t idt24x_debugfs_writer_action(struct file *fp, const char __user *user_buffer,
	size_t count, loff_t *position) {
	int err = 0;
	u32 freq;

	if (count > DEBUGFS_BUFFER_LENGTH)
		return -EINVAL;
	
	//TODO: Support more than Q2.
	freq = idt24x_data_fordebugfs->debugfs_frequencies[2];
	if (freq) {
		printk(KERN_ALERT "idt24x_debugfs_writer: calling clk_set_rate with debugfs_frequencies");
		err = clk_set_rate(idt24x_data_fordebugfs->hw.clk, freq);
		if (err) {
			dev_err(&idt24x_data_fordebugfs->i2c_client->dev, "error calling clk_set_rate (%i)\n", err);
		}
	}
	else {
		printk(KERN_ALERT "idt24x_debugfs_writer: debugfs_frequencies[2] not set; no action");
	}

	return simple_write_to_buffer(idt24x_data_fordebugfs->idt24x_ker_buf, DEBUGFS_BUFFER_LENGTH, position, user_buffer, count);
}
 
static const struct file_operations idt24x_fops_debug_action = {
	.read = idt24x_debugfs_reader_action,
	.write = idt24x_debugfs_writer_action,
};

static const struct file_operations idt24x_fops_debug_map = {
	.read = idt24x_debugfs_reader_map
};


static int idt24x_expose_via_debugfs(struct i2c_client *client, struct clk_idt24x *data) {
	int output_num;

	/* create a directory by the name idt in /sys/kernel/debugfs */
	data->debugfs_dirroot = debugfs_create_dir("idt24x", NULL);
	
	/* create files in the above directory. This requires read and write file operations */
	data->debugfs_fileaction = debugfs_create_file("action", 0644, data->debugfs_dirroot, NULL, &idt24x_fops_debug_action);
	if (!data->debugfs_fileaction) {
		dev_err(&client->dev, "idt24x_expose_via_debugfs: error creating action file");
		return (-ENODEV);
	}


	data->debugfs_map = debugfs_create_file("map", 0444, data->debugfs_dirroot, NULL, &idt24x_fops_debug_map);
	if (!data->debugfs_map) {
		dev_err(&client->dev, "idt24x_expose_via_debugfs: error creating map file");
		return (-ENODEV);
	}

	for (output_num = 0; output_num <= 3; output_num++) {	
		char name[5];
		sprintf(name, "q%d", output_num);
		data->debugfs_fileclkfreq[output_num] = debugfs_create_u64(name, 0644, data->debugfs_dirroot, &data->debugfs_frequencies[output_num]);
		if (!data->debugfs_fileclkfreq[output_num]) {
			printk("error creating %s debugfs file", name);
			return (-ENODEV);
		}
	}

	dev_info(&client->dev, "idt24x_expose_via_debugfs: success");
	idt24x_data_fordebugfs = data;
	return 0;
}

static int idt24x_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct clk_idt24x *data;
    struct clk_init_data init;

    //u32 target_frequency;
    u32 tmp32;

    int err = 0;
    //char dbg[128];

	//TODO: Figure out how to define -DDEBUG in the Makefile (maybe with a Kconfig option?)
	// so I can use KERN_DEBUG and KERN_INFO instead of KERN_ALERT.
    printk(KERN_ALERT "idt24x_probe\n");
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        printk(KERN_ALERT "idt24x_probe: ENOMEM\n");
        return -ENOMEM;
    }
    printk(KERN_ALERT "idt24x_probe; data allocated\n");

    init.ops = &idt24x_clk_ops;
    init.flags = 0;
    init.num_parents = 0;
    data->hw.init = &init;
    data->i2c_client = client;

    data->min_freq = IDT24x_MIN_FREQ;
    data->max_freq = IDT24x_MAX_FREQ;

    init.name = "idt24x";

    dev_info(&client->dev, "attempting to get input-clk for the first time");
    data->input_clk = devm_clk_get(&client->dev, "input-clk");
    if (IS_ERR(data->input_clk)) {
        err = PTR_ERR(data->input_clk);
        dev_err(&client->dev, "Unable to get input-clk clock (%u). Attempting to get xtal-freq from device tree instead. Either input-clk or xtal-freq must be specified.\n", err);
        data->input_clk = NULL;
    } else {
        data->input_clk_freq = clk_get_rate(data->input_clk);
        dev_info(&client->dev, "Got input-freq from input-clk in device tree: %u Hz", data->input_clk_freq);

        data->input_clk_rate_change_nb.notifier_call = idt24x_clk_notifier_cb;
        if (clk_notifier_register(data->input_clk, &data->input_clk_rate_change_nb))
            dev_warn(&client->dev, "Unable to register clock notifier for input_clk.\n");
    }


    if (data->input_clk == NULL) {
        err = of_property_read_u32(client->dev.of_node, "xtal-freq", &tmp32);
        if (err) {
            dev_err(&client->dev, "'xtal-freq' property missing or error (%i)\n", err);
            return err;
        } else {
            data->xtal_freq = tmp32;
            dev_info(&client->dev, "xtal_freq: %u Hz", data->xtal_freq);
        }
    }

    printk(KERN_ALERT "idt24x_probe; about to read settings: %zu\n", ARRAY_SIZE(data->settings));

    err = of_property_read_u8_array(client->dev.of_node, "settings", data->settings, ARRAY_SIZE(data->settings));
    if (!err) {
        printk(KERN_ALERT "settings property specified in DTSI\n");
        data->has_settings = true;
    } else if (err == -EOVERFLOW) {
        printk(KERN_ALERT "EOVERFLOW error trying to read the settings. ARRAY_SIZE: %zu\n", ARRAY_SIZE(data->settings));
	} else {
        dev_info(&client->dev, "settings property not specified in DTSI (or there was an error: %i). The settings property is optional.\n", err);
	}

    // Don't read clk-freq or output-num from the DTSI.  It's the responsibility of the clock consumer to set the 
    // frequency using the clock API. And we should use clock-names to specify the output clock.

    data->regmap = devm_regmap_init_i2c(client, &idt24x_regmap_config);
    if (IS_ERR(data->regmap)) {
        dev_err(&client->dev, "failed to allocate register map\n");
        return PTR_ERR(data->regmap);
    }

    printk(KERN_ALERT "idt24x_probe; call i2c_set_clientdata\n");
    i2c_set_clientdata(client, data);

    if (data->has_settings) {
        // we've got a raw settings array.  we can use that instead of the other settings.  Write this
        // immediately.  Then we can read defaults off the hw and handle other code setting a
        // new frequency.
#if 0
        int x;
        dev_info(&client->dev, "writing all settings to chip [hex]: ");
		dbg[0] = 0;
        for (x = 0; x < ARRAY_SIZE(data->settings); x++) {
            char dbg2[4];
            sprintf(dbg2, "%02x ", data->settings[x]);
            strcat(dbg, dbg2);
			if (x > 0 && (x + 1) % 32 == 0) {
				dev_info(&client->dev, dbg);
				dbg[0] = '\0';
			}
        }
        dev_info(&client->dev, dbg);
#endif

        err = i2cwritebulk(data->i2c_client, data->regmap, 0, data->settings, ARRAY_SIZE(data->settings));
        if (err) {
            dev_err(&client->dev, "error writing all settings to chip (%i)\n", err);
            return err;
        } else {
            dev_info(&client->dev, "successfully wrote full settings array");
        }
    } 

	// Whether we wrote settings or not, read all current values from the hw.
    dev_info(&client->dev, "read from HW");
    err = idt24x_read_from_hw(data);
    if (err) {
        dev_err(&client->dev, "failed calling idt24x_read_from_hw (%i)\n", err);
        return err;
    }

    //TODO: Need to create all 4 clocks here, I believe.
	err = devm_clk_hw_register(&client->dev, &data->hw);
	if (err) {
		dev_err(&client->dev, "clock registration failed\n");
		return err;
	}

	err = of_clk_add_hw_provider(client->dev.of_node, of_clk_hw_simple_get, &data->hw);
	if (err) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return err;
	}

    if (!data->has_settings) {
        // The target frequency will not be specified in the DTSI anymore.  Either a consumer needs to use the clk API to request the rate,
        // or use debugfs to set the rate from user space.
        dev_info(&client->dev, "registered: no settings string specified\n");
    } else {
        dev_info(&client->dev, "registered: full settings string specified\n");
    }

    err = idt24x_expose_via_debugfs(client, data);
    if (err) {
        dev_err(&client->dev, "error calling idt24x_expose_via_debugfs: %i\n", err);
        return err;
    }

    return 0;
}

static int idt24x_remove(struct i2c_client *client)
{
    struct clk_idt24x *data = to_clk_idt24x_from_client(&client);
    printk(KERN_ALERT "idt24x_remove\n");
    of_clk_del_provider(client->dev.of_node);
    debugfs_remove_recursive(data->debugfs_dirroot);
    
    if (data->input_clk != NULL)
        clk_notifier_unregister(data->input_clk, &data->input_clk_rate_change_nb);
    return 0;
}


static const struct i2c_device_id idt24x_id[] = {
    { "idt24x", idt24x },
    { }
};
MODULE_DEVICE_TABLE(i2c, idt24x_id);

static const struct of_device_id idt24x_of_match[] = {
    { .compatible = "idt,idt24x" },
    {},
};
MODULE_DEVICE_TABLE(of, idt24x_of_match);

static struct i2c_driver idt24x_driver = {
    .driver = {
    .name = DRV_NAME,
    .of_match_table = idt24x_of_match,
    },
    .probe = idt24x_probe,
    .remove = idt24x_remove,
    .id_table = idt24x_id,
};

module_i2c_driver(idt24x_driver);

MODULE_DESCRIPTION("Common clock framework driver for the 8T49N24x");
MODULE_AUTHOR("David Cater <david.cater@idt.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

