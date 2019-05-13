// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ADF4371 SPI Wideband Synthesizer driver
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gcd.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

/* Registers address macro */
#define ADF4371_REG(x) (0x ## x)

/* ADF4371_REG0 */
#define ADF4371_ADDR_ASC_MSK		BIT(2)
#define ADF4371_ADDR_ASC(x)		FIELD_PREP(ADF4371_ADDR_ASC_MSK, x)
#define ADF4371_ADDR_ASC_R_MSK		BIT(5)
#define ADF4371_ADDR_ASC_R(x)		FIELD_PREP(ADF4371_ADDR_ASC_R_MSK, x)
#define ADF4371_RESET_CMD		0x81

/* ADF4371_REG17 */
#define ADF4371_FRAC2WORD_L_MSK		GENMASK(7, 1)
#define ADF4371_FRAC2WORD_L(x)		FIELD_PREP(ADF4371_FRAC2WORD_L_MSK, x)
#define ADF4371_FRAC1WORD_MSK		BIT(0)
#define ADF4371_FRAC1WORD(x)		FIELD_PREP(ADF4371_FRAC1WORD_MSK, x)

/* ADF4371_REG18 */
#define ADF4371_FRAC2WORD_H_MSK		GENMASK(6, 0)
#define ADF4371_FRAC2WORD_H(x)		FIELD_PREP(ADF4371_FRAC2WORD_H_MSK, x)

/* ADF4371_REG1A */
#define ADF4371_MOD2WORD_MSK		GENMASK(5, 0)
#define ADF4371_MOD2WORD(x)		FIELD_PREP(ADF4371_MOD2WORD_MSK, x)

/* ADF4371_REG24 */
#define ADF4371_RF_DIV_SEL_MSK		GENMASK(6, 4)
#define ADF4371_RF_DIV_SEL(x)		FIELD_PREP(ADF4371_RF_DIV_SEL_MSK, x)

/* ADF4371_REG32 */
#define ADF4371_TIMEOUT_MSK		GENMASK(1, 0)
#define ADF4371_TIMEOUT(x)		FIELD_PREP(ADF4371_TIMEOUT_MSK, x)

/* ADF4371_REG34 */
#define ADF4371_VCO_ALC_TOUT_MSK	GENMASK(4, 0)
#define ADF4371_VCO_ALC_TOUT(x)		FIELD_PREP(ADF4371_VCO_ALC_TOUT_MSK, x)

/* Specifications */
#define ADF4371_MIN_VCO_FREQ		4000000000ULL /* 4000 MHz */
#define ADF4371_MAX_VCO_FREQ		8000000000ULL /* 8000 MHz */
#define ADF4371_MAX_OUT_RF8_FREQ	ADF4371_MAX_VCO_FREQ /* Hz */
#define ADF4371_MIN_OUT_RF8_FREQ	(ADF4371_MIN_VCO_FREQ / 64) /* Hz */
#define ADF4371_MAX_OUT_RF16_FREQ	(ADF4371_MAX_VCO_FREQ * 2) /* Hz */
#define ADF4371_MIN_OUT_RF16_FREQ	(ADF4371_MIN_VCO_FREQ * 2) /* Hz */
#define ADF4371_MAX_OUT_RF32_FREQ	(ADF4371_MAX_VCO_FREQ * 4) /* Hz */
#define ADF4371_MIN_OUT_RF32_FREQ	(ADF4371_MIN_VCO_FREQ * 4) /* Hz */

#define ADF4371_MAX_FREQ_PFD		250000000UL /* Hz */
#define ADF4371_MAX_FREQ_REFIN		600000000UL /* Hz */

/* MOD1 is a 24-bit primary modulus with fixed value of 2^25 */
#define ADF4371_MODULUS1		33554432ULL
/* MOD2 is the programmable, 14-bit auxiliary fractional modulus */
#define ADF4371_MAX_MODULUS2		16384

#define ADF4371_CHECK_RANGE(freq, range) \
	((freq > ADF4371_MAX_ ## range) || (freq < ADF4371_MIN_ ## range))

enum {
	ADF4371_FREQ,
	ADF4371_POWER_DOWN
};

enum {
	ADF4371_CH_RF8,
	ADF4371_CH_RF16,
	ADF4371_CH_RF32,
	ADF4371_CH_RFAUX8
};

struct adf4371_pwrdown {
	unsigned int reg;
	unsigned int bit;
};

static const struct adf4371_pwrdown adf4371_pwrdown_ch[4] = {
	[ADF4371_CH_RF8] = { ADF4371_REG(25), 2 },
	[ADF4371_CH_RF16] = { ADF4371_REG(25), 3 },
	[ADF4371_CH_RF32] = { ADF4371_REG(25), 4 },
	[ADF4371_CH_RFAUX8] = { ADF4371_REG(72), 3 }
};

static const struct reg_sequence adf4371_reg_defaults[] = {
	{ ADF4371_REG(0),  0x18 },
	{ ADF4371_REG(12), 0x40 },
	{ ADF4371_REG(1E), 0x48 },
	{ ADF4371_REG(20), 0x14 },
	{ ADF4371_REG(22), 0x00 },
	{ ADF4371_REG(23), 0x00 },
	{ ADF4371_REG(24), 0x80 },
	{ ADF4371_REG(25), 0x07 },
	{ ADF4371_REG(27), 0xC5 },
	{ ADF4371_REG(28), 0x83 },
	{ ADF4371_REG(2C), 0x44 },
	{ ADF4371_REG(2D), 0x11 },
	{ ADF4371_REG(2E), 0x12 },
	{ ADF4371_REG(2F), 0x94 },
	{ ADF4371_REG(32), 0x04 },
	{ ADF4371_REG(35), 0xFA },
	{ ADF4371_REG(36), 0x30 },
	{ ADF4371_REG(39), 0x07 },
	{ ADF4371_REG(3A), 0x55 },
	{ ADF4371_REG(3E), 0x0C },
	{ ADF4371_REG(3F), 0x80 },
	{ ADF4371_REG(40), 0x50 },
	{ ADF4371_REG(41), 0x28 },
	{ ADF4371_REG(47), 0xC0 },
	{ ADF4371_REG(52), 0xF4 },
	{ ADF4371_REG(70), 0x03 },
	{ ADF4371_REG(71), 0x60 },
	{ ADF4371_REG(72), 0x32 },
};

static const struct regmap_config adf4371_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

struct adf4371_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct clk *clkin;
	struct mutex lock;
	unsigned long clkin_freq;
	unsigned long fpfd;
	unsigned int integer;
	unsigned int fract1;
	unsigned int fract2;
	unsigned int mod2;
	unsigned int rf_div_sel;
	unsigned int ref_div_factor;
};

static unsigned long long adf4371_pll_fract_n_get_rate(struct adf4371_state *st,
						       u32 channel)
{
	unsigned long long val, tmp;
	unsigned int ref_div_sel;

	val = (((u64)st->integer * ADF4371_MODULUS1) + st->fract1) * st->fpfd;
	tmp = (u64)st->fract2 * st->fpfd;
	do_div(tmp, st->mod2);
	val += tmp + ADF4371_MODULUS1 / 2;

	if (channel == ADF4371_CH_RF8 || channel == ADF4371_CH_RFAUX8)
		ref_div_sel = st->rf_div_sel;
	else
		ref_div_sel = 0;

	do_div(val, ADF4371_MODULUS1 * (1 << ref_div_sel));

	if (channel == ADF4371_CH_RF16)
		val <<= 1;
	else if (channel == ADF4371_CH_RF32)
		val <<= 2;

	return val;
}

static void adf4371_pll_fract_n_compute(unsigned long long vco,
				       unsigned long long pfd,
				       unsigned int *integer,
				       unsigned int *fract1,
				       unsigned int *fract2,
				       unsigned int *mod2)
{
	unsigned long long tmp;
	u32 gcd_div;

	tmp = do_div(vco, pfd);
	tmp = tmp * ADF4371_MODULUS1;
	*fract2 = do_div(tmp, pfd);

	*integer = vco;
	*fract1 = tmp;

	*mod2 = pfd;

	while (*mod2 > ADF4371_MAX_MODULUS2) {
		*mod2 >>= 1;
		*fract2 >>= 1;
	}

	gcd_div = gcd(*fract2, *mod2);
	*mod2 /= gcd_div;
	*fract2 /= gcd_div;
}

static int adf4371_set_freq(struct adf4371_state *st, unsigned long long freq,
			    unsigned int channel)
{
	u32 cp_bleed;
	u8 buf[10], int_mode = 0;
	int ret;

	switch (channel) {
	case ADF4371_CH_RF8:
	case ADF4371_CH_RFAUX8:
		if (ADF4371_CHECK_RANGE(freq, OUT_RF8_FREQ))
			return -EINVAL;

		st->rf_div_sel = 0;

		while (freq < ADF4371_MIN_VCO_FREQ) {
			freq <<= 1;
			st->rf_div_sel++;
		}
		break;
	case ADF4371_CH_RF16:
		/* ADF4371 RF16 8000...16000 MHz */
		if (ADF4371_CHECK_RANGE(freq, OUT_RF16_FREQ))
			return -EINVAL;

		freq >>= 1;
		break;
	case ADF4371_CH_RF32:
		/* ADF4371 RF32 16000...32000 MHz */
		if (ADF4371_CHECK_RANGE(freq, OUT_RF32_FREQ))
			return -EINVAL;

		freq >>= 2;
		break;
	default:
		return -EINVAL;
	}

	adf4371_pll_fract_n_compute(freq, st->fpfd, &st->integer, &st->fract1,
				    &st->fract2, &st->mod2);
	buf[0] = st->integer >> 8;
	buf[1] = 0x40; /* REG12 default */
	buf[2] = 0x00;
	buf[3] = st->fract2 & 0xFF;
	buf[4] = st->fract2 >> 7;
	buf[5] = st->fract2 >> 15;
	buf[6] = ADF4371_FRAC2WORD_L(st->fract2 & 0x7F) |
		 ADF4371_FRAC1WORD(st->fract1 >> 23);
	buf[7] = ADF4371_FRAC2WORD_H(st->fract2 >> 7);
	buf[8] = st->mod2 & 0xFF;
	buf[9] = ADF4371_MOD2WORD(st->mod2 >> 8);

	ret = regmap_bulk_write(st->regmap, ADF4371_REG(11),
				buf, ARRAY_SIZE(buf));
	if (ret < 0)
		return ret;
	/*
	 * The R counter allows the input reference frequency to be
	 * divided down to produce the reference clock to the PFD
	 */
	ret = regmap_write(st->regmap, ADF4371_REG(1F), st->ref_div_factor);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, ADF4371_REG(24),
				 ADF4371_RF_DIV_SEL_MSK,
				 ADF4371_RF_DIV_SEL(st->rf_div_sel));
	if (ret < 0)
		return ret;

	cp_bleed = DIV_ROUND_UP(400 * 1750, st->integer * 375);
	cp_bleed = clamp(cp_bleed, 1U, 255U);
	ret = regmap_write(st->regmap, ADF4371_REG(26), cp_bleed);
	if (ret < 0)
		return ret;
	/*
	 * Set to 1 when in INT mode (when FRAC1 = FRAC2 = 0),
	 * and set to 0 when in FRAC mode.
	 */
	if (st->fract1 == 0 && st->fract2 == 0)
		int_mode = 0x01;

	ret = regmap_write(st->regmap, ADF4371_REG(2B), int_mode);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, ADF4371_REG(10), st->integer & 0xFF);
}

static ssize_t adf4371_read(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    char *buf)
{
	struct adf4371_state *st = iio_priv(indio_dev);
	unsigned long long val = 0;
	unsigned int readval, reg, bit;
	int ret;

	switch ((u32)private) {
	case ADF4371_FREQ:
		val = adf4371_pll_fract_n_get_rate(st, chan->channel);
		ret = regmap_read(st->regmap, ADF4371_REG(7C), &readval);
		if (ret < 0)
			break;

		if (readval == 0x00) {
			dev_dbg(&st->spi->dev, "PLL un-locked\n");
			ret = -EBUSY;
		}
		break;
	case ADF4371_POWER_DOWN:
		reg = adf4371_pwrdown_ch[chan->channel].reg;
		bit = adf4371_pwrdown_ch[chan->channel].bit;

		ret = regmap_read(st->regmap, reg, &readval);
		if (ret < 0)
			break;

		val = !(readval & BIT(bit));
		break;
	default:
		ret = -EINVAL;
		val = 0;
		break;
	}

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t adf4371_write(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len)
{
	struct adf4371_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned int bit, readval, reg;
	int ret;

	ret = kstrtoull(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF4371_FREQ:
		ret = adf4371_set_freq(st, readin, chan->channel);
		break;
	case ADF4371_POWER_DOWN:
		reg = adf4371_pwrdown_ch[chan->channel].reg;
		bit = adf4371_pwrdown_ch[chan->channel].bit;
		ret = regmap_read(st->regmap, reg, &readval);
		if (ret < 0)
			break;

		readval &= ~BIT(bit);
		readval |= (!readin << bit);

		ret = regmap_write(st->regmap, reg, readval);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

#define _ADF4371_EXT_INFO(_name, _ident) { \
		.name = _name, \
		.read = adf4371_read, \
		.write = adf4371_write, \
		.private = _ident, \
		.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adf4371_ext_info[] = {
	/* Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz. Using scale is a bit ugly.
	 */
	_ADF4371_EXT_INFO("frequency", ADF4371_FREQ),
	_ADF4371_EXT_INFO("powerdown", ADF4371_POWER_DOWN),
	{ },
};

#define ADF4371_CHANNEL(_name, index) { \
		.type = IIO_ALTVOLTAGE, \
		.output = 1, \
		.channel = index, \
		.ext_info = adf4371_ext_info, \
		.extend_name = _name, \
	}

static const struct iio_chan_spec adf4371_chan[] = {
	ADF4371_CHANNEL("rf8", ADF4371_CH_RF8),
	ADF4371_CHANNEL("rf16", ADF4371_CH_RF16),
	ADF4371_CHANNEL("rf32", ADF4371_CH_RF32),
	ADF4371_CHANNEL("rfaux8", ADF4371_CH_RFAUX8),
};

static int adf4371_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adf4371_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info adf4371_info = {
	.debugfs_reg_access = &adf4371_reg_access,
};

static int adf4371_setup(struct adf4371_state *st)
{
	unsigned int synth_timeout = 2, timeout = 1, vco_alc_timeout = 1;
	unsigned int vco_band_div, tmp;
	u8 buf[5];
	int ret;

	/* Perform a software reset */
	ret = regmap_write(st->regmap, ADF4371_REG(0), ADF4371_RESET_CMD);
	if (ret < 0)
		return ret;

	ret = regmap_multi_reg_write(st->regmap, adf4371_reg_defaults,
				     ARRAY_SIZE(adf4371_reg_defaults));
	if (ret < 0)
		return ret;

	/* Set address in ascending order, so the bulk_write() will work */
	ret = regmap_update_bits(st->regmap, ADF4371_REG(0),
				 ADF4371_ADDR_ASC_MSK | ADF4371_ADDR_ASC_R_MSK,
				 ADF4371_ADDR_ASC(1) | ADF4371_ADDR_ASC_R(1));
	if (ret < 0)
		return ret;
	/*
	 * Calculate and maximize PFD frequency
	 * fPFD = REFIN × ((1 + D)/(R × (1 + T)))
	 * Where D is the REFIN doubler bit, T is the reference divide by 2,
	 * R is the reference division factor
	 * TODO: it is assumed D and T equal 0.
	 */
	do {
		st->ref_div_factor++;
		st->fpfd = st->clkin_freq / st->ref_div_factor;
	} while (st->fpfd > ADF4371_MAX_FREQ_PFD);

	/* Calculate Timeouts */
	vco_band_div = DIV_ROUND_UP(st->fpfd, 2400000U);

	tmp = DIV_ROUND_CLOSEST(st->fpfd, 1000000U);
	do {
		timeout++;
		if (timeout > 1023) {
			timeout = 2;
			synth_timeout++;
		}
	} while (synth_timeout * 1024 + timeout <= 20 * tmp);

	do {
		vco_alc_timeout++;
	} while (vco_alc_timeout * 1024 - timeout <= 50 * tmp);

	buf[0] = vco_band_div;
	buf[1] = timeout & 0xFF;
	buf[2] = ADF4371_TIMEOUT(timeout >> 8) | 0x04;
	buf[3] = synth_timeout;
	buf[4] = ADF4371_VCO_ALC_TOUT(vco_alc_timeout);

	return regmap_bulk_write(st->regmap, ADF4371_REG(30),
				 buf, ARRAY_SIZE(buf));
}

static void adf4371_clk_disable(void *data)
{
	struct adf4371_state *st = data;

	clk_disable_unprepare(st->clkin);
}

static int adf4371_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adf4371_state *st;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adf4371_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
	st->regmap = regmap;
	mutex_init(&st->lock);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &adf4371_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adf4371_chan;
	indio_dev->num_channels = ARRAY_SIZE(adf4371_chan);

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return PTR_ERR(st->clkin);

	ret = clk_prepare_enable(st->clkin);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4371_clk_disable, st);
	if (ret)
		return ret;

	st->clkin_freq = clk_get_rate(st->clkin);

	ret = adf4371_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "ADF4371 setup failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adf4371_id_table[] = {
	{ "adf4371", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, adf4371_id_table);

static const struct of_device_id adf4371_of_match[] = {
	{ .compatible = "adi,adf4371" },
	{ },
};
MODULE_DEVICE_TABLE(of, adf4371_of_match);

static struct spi_driver adf4371_driver = {
	.driver = {
		.name = "adf4371",
		.of_match_table = adf4371_of_match,
	},
	.probe = adf4371_probe,
	.id_table = adf4371_id_table,
};
module_spi_driver(adf4371_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4371 SPI PLL");
MODULE_LICENSE("GPL");
