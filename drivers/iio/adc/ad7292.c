// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7292 SPI ADC driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

#define ADI_VENDOR_ID 0x0018

/* AD7292 registers definition */
#define AD7292_REG_VENDOR_ID		0x00
#define AD7292_REG_ADC_SEQ		0x03
#define AD7292_REG_CONV_COMM		0x0E
#define AD7292_REG_ADC_CH0		0x10
#define AD7292_REG_ADC_CH1		0x11
#define AD7292_REG_ADC_CH2		0x12
#define AD7292_REG_ADC_CH3		0x13
#define AD7292_REG_ADC_CH4		0x14
#define AD7292_REG_ADC_CH5		0x15
#define AD7292_REG_ADC_CH6		0x16
#define AD7292_REG_ADC_CH7		0x17

#define AD7292_RD_FLAG_MSK(x)		(BIT(7) | ((x) & 0x3F))

/* AD7292_REG_ADC_CONVERSION */
#define AD7292_ADC_DATA_MASK		GENMASK(15, 6)
#define AD7292_ADC_DATA(x)		FIELD_GET(AD7292_ADC_DATA_MASK, x)

#define AD7291_VOLTAGE_CHAN(_chan)					\
{									\
	.type = IIO_VOLTAGE,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.indexed = 1,							\
	.channel = _chan,						\
}

static const struct iio_chan_spec ad7292_channels[] = {
	AD7291_VOLTAGE_CHAN(0),
	AD7291_VOLTAGE_CHAN(1),
	AD7291_VOLTAGE_CHAN(2),
	AD7291_VOLTAGE_CHAN(3),
	AD7291_VOLTAGE_CHAN(4),
	AD7291_VOLTAGE_CHAN(5),
	AD7291_VOLTAGE_CHAN(6),
	AD7291_VOLTAGE_CHAN(7)
};

static const struct iio_chan_spec ad7292_channels_diff[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.indexed = 1,
		.differential = 1,
		.channel = 0,
		.channel2 = 1,
	},
	AD7291_VOLTAGE_CHAN(2),
	AD7291_VOLTAGE_CHAN(3),
	AD7291_VOLTAGE_CHAN(4),
	AD7291_VOLTAGE_CHAN(5),
	AD7291_VOLTAGE_CHAN(6),
	AD7291_VOLTAGE_CHAN(7)
};

static unsigned int ad7292_channel_address[8] = {
	AD7292_REG_ADC_CH0, AD7292_REG_ADC_CH1, AD7292_REG_ADC_CH2,
	AD7292_REG_ADC_CH3, AD7292_REG_ADC_CH4, AD7292_REG_ADC_CH5,
	AD7292_REG_ADC_CH6, AD7292_REG_ADC_CH7
};

struct ad7292_state {
	struct spi_device *spi;
	struct regulator *reg;
	unsigned short vref_mv;
	union {
		__be16 d16;
		u8 d8[2];
	} data ____cacheline_aligned;
};

static int ad7292_spi_reg_read(struct ad7292_state *st, unsigned int addr)
{
	int ret;

	st->data.d8[0] = AD7292_RD_FLAG_MSK(addr);

	ret = spi_write_then_read(st->spi, st->data.d8, 1, &st->data.d16, 2);
	if (ret < 0)
		return ret;

	return be16_to_cpu(st->data.d16);
}

static int ad7292_spi_subreg_read(struct ad7292_state *st, unsigned int addr,
				  unsigned int sub_addr, unsigned int len)
{
	unsigned int shift = 16 - (8 * len);
	int ret;

	st->data.d8[0] = AD7292_RD_FLAG_MSK(addr);
	st->data.d8[1] = sub_addr;

	ret = spi_write_then_read(st->spi, st->data.d8, 2, &st->data.d16, len);
	if (ret < 0)
		return ret;

	return (be16_to_cpu(st->data.d16) >> shift);
}

static int ad7292_single_conversion(struct ad7292_state *st,
				    unsigned int chan_addr)
{
	int ret;

	st->data.d8[0] = chan_addr;
	st->data.d8[1] = AD7292_RD_FLAG_MSK(AD7292_REG_CONV_COMM);

	ret = spi_write_then_read(st->spi, &st->data.d8, 2, &st->data.d16, 2);

	if (ret < 0)
		return ret;

	return be16_to_cpu(st->data.d16);
}

static int ad7292_setup(struct ad7292_state *st)
{
	return 0;
}

static int ad7292_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7292_state *st = iio_priv(indio_dev);
	unsigned int ch_addr;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ch_addr = ad7292_channel_address[chan->channel];
		ret = ad7292_single_conversion(st, ch_addr);
		if (ret < 0)
			return ret;

		*val = AD7292_ADC_DATA(ret);

		return IIO_VAL_INT;
	default:
		break;
	}
	return 0;
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	return 0;
}

static const struct iio_info ad7292_info = {
	.read_raw = ad7292_read_raw,
	.write_raw = &ad7768_write_raw,
};

static void ad7292_regulator_disable(void *data)
{
	struct ad7292_state *st = data;

	regulator_disable(st->reg);
}

static int ad7292_probe(struct spi_device *spi)
{
	struct ad7292_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	st->reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret) {
			dev_err(&spi->dev,
				"Failed to enable external vref supply\n");
			return ret;
		}

		ret = devm_add_action_or_reset(&spi->dev,
					       ad7292_regulator_disable, st);
		if (ret) {
			regulator_disable(st->reg);
			return ret;
		}

		ret = regulator_get_voltage(st->reg);
		if (ret < 0)
			return ret;

		st->vref_mv = ret / 1000;
	} else {
		st->vref_mv = 1250;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad7292_info;

	ret = of_property_read_bool(spi->dev.of_node, "diff-channels");
	if (ret) {
		indio_dev->num_channels = ARRAY_SIZE(ad7292_channels_diff);
		indio_dev->channels = ad7292_channels_diff;
	} else {
		indio_dev->num_channels = ARRAY_SIZE(ad7292_channels);
		indio_dev->channels = ad7292_channels;
	}

	ret = ad7292_setup(st);
	if (ret)
		return ret;

	ret = ad7292_spi_reg_read(st, AD7292_REG_VENDOR_ID);
	if (ret != ADI_VENDOR_ID) {
		dev_err(&spi->dev, "Wrong vendor id %x\n", ret);
		return -EINVAL;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad7292_id_table[] = {
	{ "ad7292", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7292_id_table);

static const struct of_device_id ad7292_of_match[] = {
	{ .compatible = "adi,ad7292" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7292_of_match);

static struct spi_driver ad7292_driver = {
	.driver = {
		.name = "ad7292",
		.of_match_table = ad7292_of_match,
	},
	.probe = ad7292_probe,
	.id_table = ad7292_id_table,
};
module_spi_driver(ad7292_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt1@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD7292 ADC driver");
MODULE_LICENSE("GPL");
