// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7292 SPI ADC driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define ADI_VENDOR_ID 0x0018

/* AD7292 registers definition */
#define AD7292_REG_VENDOR_ID		0x00
#define AD7292_REG_CONF_BANK		0x05
#define AD7292_REG_CONV_COMM		0x0E
#define AD7292_REG_ADC_CH(x)		(0x10 + (x))

/* AD7292 configuration bank subregisters definition */
#define AD7292_BANK_REG_OUT_DRV		0x01
#define AD7292_BANK_REG_VIN_RNG0	0x10
#define AD7292_BANK_REG_VIN_RNG1	0x11
#define AD7292_BANK_REG_SAMP_MODE	0x12

#define AD7292_RD_FLAG_MSK(x)		(BIT(7) | ((x) & 0x3F))
#define AD7292_WR_FLAG_MSK(x)		(BIT(6) | ((x) & 0x3F))

/* AD7292_REG_ADC_CONVERSION */
#define AD7292_ADC_DATA_MASK		GENMASK(15, 6)
#define AD7292_ADC_DATA(x)		FIELD_GET(AD7292_ADC_DATA_MASK, (x))

/* AD7292_BANK_REG_OUT_DRV */
#define AD7292_OUT_DRV_MASK		GENMASK(11, 0)
#define AD7292_OUT_DRV(x)		FIELD_PREP(AD7292_OUT_DRV_MASK, (x))


/* AD7292_CHANNEL_SAMPLING_MODE */
#define AD7292_CH_SAMP_MODE(reg, ch)	(((reg) >> 8) & BIT(ch))

/* AD7292_CHANNEL_VIN_RANGE */
#define AD7292_CH_VIN_RANGE(reg, ch)	((reg) & BIT(ch))

#define AD7292_VOLTAGE_CHAN(_chan)					\
{									\
	.type = IIO_VOLTAGE,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.indexed = 1,							\
	.channel = _chan,						\
	.scan_index = _chan,						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 10,						\
		.storagebits = 16,					\
		.shift = 6,						\
		.endianness = IIO_LE,					\
	},								\
}

static const struct iio_chan_spec ad7292_channels[] = {
	AD7292_VOLTAGE_CHAN(0),
	AD7292_VOLTAGE_CHAN(1),
	AD7292_VOLTAGE_CHAN(2),
	AD7292_VOLTAGE_CHAN(3),
	AD7292_VOLTAGE_CHAN(4),
	AD7292_VOLTAGE_CHAN(5),
	AD7292_VOLTAGE_CHAN(6),
	AD7292_VOLTAGE_CHAN(7)
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
	AD7292_VOLTAGE_CHAN(2),
	AD7292_VOLTAGE_CHAN(3),
	AD7292_VOLTAGE_CHAN(4),
	AD7292_VOLTAGE_CHAN(5),
	AD7292_VOLTAGE_CHAN(6),
	AD7292_VOLTAGE_CHAN(7)
};

struct ad7292_state {
	struct spi_device *spi;
	struct regulator *reg;
	unsigned short vref_mv;
	struct iio_trigger *trig;
	struct mutex lock;
	int curr_ch;

	__be16 d16 ____cacheline_aligned;
	u8 d8[4];
};

static int ad7292_spi_reg_read(struct ad7292_state *st, unsigned int addr)
{
	int ret;

	st->d8[0] = AD7292_RD_FLAG_MSK(addr);

	ret = spi_write_then_read(st->spi, st->d8, 1, &st->d16, 2);
	if (ret < 0)
		return ret;

	return be16_to_cpu(st->d16);
}

static int ad7292_spi_subreg_read(struct ad7292_state *st, unsigned int addr,
				  unsigned int sub_addr, unsigned int len)
{
	unsigned int shift = 16 - (8 * len);
	int ret;

	st->d8[0] = AD7292_RD_FLAG_MSK(addr);
	st->d8[1] = sub_addr;

	ret = spi_write_then_read(st->spi, st->d8, 2, &st->d16, len);
	if (ret < 0)
		return ret;

	return (be16_to_cpu(st->d16) >> shift);
}

static int ad7292_spi_subreg_write(struct ad7292_state *st, unsigned int addr,
				   unsigned int sub_addr, unsigned int val)
{
	st->d8[0] = AD7292_WR_FLAG_MSK(addr);
	st->d8[1] = sub_addr;
	st->d8[2] = (val & 0xFF00) >> 8;
	st->d8[3] = val & 0x00FF;
	//dev_dbg(&st->spi->dev, "subreg_write: d8 buffer: 0x%x %x %x %x\n",
	//	st->d8[0], st->d8[1], st->d8[2], st->d8[3]);
	return spi_write(st->spi, st->d8, 4);
}

//TODO do not submit this function
static int ad7292_read_out_bank(struct ad7292_state *st)
{
	int regval;

	mutex_lock(&st->lock);
	regval = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					AD7292_BANK_REG_OUT_DRV, 2);
	if (regval < 0)
		goto err_unlock;

	regval = FIELD_GET(AD7292_OUT_DRV_MASK, regval);

err_unlock:
	mutex_unlock(&st->lock);
	if (regval < 0)
		dev_dbg(&st->spi->dev, "ad7292 ERROR read_out_bank\n");
	dev_dbg(&st->spi->dev, "read_out_bank: conf_bank io_driv 0x%03x\n",
		regval);
	return regval;
}

static int ad7292_enable_gpio(struct ad7292_state *st, int gpio_nr, bool enable)
{
	int regval;

	mutex_lock(&st->lock);
	regval = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					AD7292_BANK_REG_OUT_DRV, 2);
	if (regval < 0)
		goto err_unlock;

	//dev_dbg(&st->spi->dev, "enable_gpio: conf_bank io_driv 0x%03x\n", regval);

	regval &= ~BIT(gpio_nr);
	if (enable)
		regval |= AD7292_OUT_DRV(BIT(gpio_nr));

	dev_dbg(&st->spi->dev, "enable_gpio: set conf_bank io_driv to 0x%03x\n",
		regval);

	regval = ad7292_spi_subreg_write(st, AD7292_REG_CONF_BANK,
					 AD7292_BANK_REG_OUT_DRV, regval);

err_unlock:
	mutex_unlock(&st->lock);

	return regval;
}

static int ad7292_single_conversion(struct ad7292_state *st,
				    unsigned int chan_addr)
{
	int ret;

	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d8,
			.len = 4,
			.delay_usecs = 6,
		}, {
			.rx_buf = &st->d16,
			.len = 2,
		},
	};

	st->d8[0] = chan_addr;
	st->d8[1] = AD7292_RD_FLAG_MSK(AD7292_REG_CONV_COMM);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));

	if (ret < 0)
		return ret;

	return be16_to_cpu(st->d16);
}

static int ad7292_vin_range_multiplier(struct ad7292_state *st, int channel)
{
	int samp_mode, range0, range1, factor = 1;

	/*
	 * Every AD7292 ADC channel may have its input range adjusted according
	 * to the settings at the ADC sampling mode and VIN range subregisters.
	 * For a given channel, the minimum input range is equal to Vref, and it
	 * may be increased by a multiplier factor of 2 or 4 according to the
	 * following rule:
	 * If channel is being sampled with respect to AGND:
	 *	factor = 4 if VIN range0 and VIN range1 equal 0
	 *	factor = 2 if only one of VIN ranges equal 1
	 *	factor = 1 if both VIN range0 and VIN range1 equal 1
	 * If channel is being sampled with respect to AVDD:
	 *	factor = 4 if VIN range0 and VIN range1 equal 0
	 *	Behavior is undefined if any of VIN range doesn't equal 0
	 */

	samp_mode = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					   AD7292_BANK_REG_SAMP_MODE, 2);

	if (samp_mode < 0)
		return samp_mode;

	range0 = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					AD7292_BANK_REG_VIN_RNG0, 2);

	if (range0 < 0)
		return range0;

	range1 = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					AD7292_BANK_REG_VIN_RNG1, 2);

	if (range1 < 0)
		return range1;

	if (AD7292_CH_SAMP_MODE(samp_mode, channel)) {
		/* Sampling with respect to AGND */
		if (!AD7292_CH_VIN_RANGE(range0, channel))
			factor *= 2;

		if (!AD7292_CH_VIN_RANGE(range1, channel))
			factor *= 2;

	} else {
		/* Sampling with respect to AVDD */
		if (AD7292_CH_VIN_RANGE(range0, channel) ||
		    AD7292_CH_VIN_RANGE(range1, channel))
			return -EPERM;

		factor = 4;
	}

	return factor;
}

static int ad7292_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7292_state *st = iio_priv(indio_dev);
	unsigned int ch_addr;
	int ret;

	if (indio_dev->currentmode & INDIO_BUFFER_TRIGGERED)
		dev_dbg(&st->spi->dev, "read_raw: trigg mode");
	else
		dev_dbg(&st->spi->dev, "read_raw: direct mode");

	ad7292_read_out_bank(st);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ch_addr = AD7292_REG_ADC_CH(chan->channel);
		ret = ad7292_single_conversion(st, ch_addr);
		if (ret < 0)
			return ret;

		*val = AD7292_ADC_DATA(ret);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/*
		 * To convert a raw value to standard units, the IIO defines
		 * this formula: Scaled value = (raw + offset) * scale.
		 * For the scale to be a correct multiplier for (raw + offset),
		 * it must be calculated as the input range divided by the
		 * number of possible distinct input values. Given the ADC data
		 * is 10 bit long, it may assume 2^10 distinct values.
		 * Hence, scale = range / 2^10. The IIO_VAL_FRACTIONAL_LOG2
		 * return type indicates to the IIO API to divide *val by 2 to
		 * the power of *val2 when returning from read_raw.
		 */

		ret = ad7292_vin_range_multiplier(st, chan->channel);
		if (ret < 0)
			return ret;

		*val = st->vref_mv * ret;
		*val2 = 10;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		break;
	}
	return -EINVAL;
}

static irqreturn_t ad7292_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7292_state *st = iio_priv(indio_dev);
	unsigned short data[5]; /* 16-bit sample + 64-bit timestamp */
	struct spi_transfer t[] = {
		{
			.rx_buf = &st->d16,
			.len = 2,
			.cs_change = 1
		}
	};
	int ret;

	//dev_dbg(&st->spi->dev, "trigger_handler %d: irq  %d", irq, irq);
	dev_dbg(&st->spi->dev, "trigger_handler %d: pf->irq  %d", irq, pf->irq);

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "trigger_handler %d: gpiod_get_raw_value ret: %d",
		irq, ret);

	mutex_lock(&st->lock);

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		goto err_unlock;

	data[0] = AD7292_ADC_DATA(be16_to_cpu(st->d16));

	dev_dbg(&st->spi->dev, "trigger_handler %d: data[0..4]: %u, %u, %u, %u, %u",
		irq, data[0], data[1], data[2], data[3], data[4]);
	dev_dbg(&st->spi->dev, "trigger_handler %d: indio_dev->scan_timestamp: %d\n",
		irq, indio_dev->scan_timestamp);

//    u16 buf[8];
//    int i = 0;
//
//    /* read data for each active channel */
//    for_each_set_bit(bit, active_scan_mask, masklength)
//        buf[i++] = sensor_get_data(bit)
//
//    iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
//
//    iio_trigger_notify_done(trigger);
//    return IRQ_HANDLED;

//	mutex_lock(&st->lock);
//	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
//		val = (sample[1] << 8) + sample[0];
//
//		chan = iio_find_channel_from_si(indio_dev, i);
//		ret = st->write(st, AD5686_CMD_WRITE_INPUT_N_UPDATE_N,
//				chan->address, val << chan->scan_type.shift);
//	}
//	mutex_unlock(&st->lock);


	//iio_push_to_buffers_with_timestamp(indio_dev, data,
	//				   iio_get_time_ns(indio_dev));
	iio_push_to_buffers(indio_dev, data);

err_unlock:
	mutex_unlock(&st->lock);

	iio_trigger_notify_done(indio_dev->trig);

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "trigger_handler %d: gpiod_get_raw_value ret: %d",
		irq, ret);
	if (ret < 0)
		return ret;
	dev_dbg(&st->spi->dev, "trigger_handler %d: return", irq);

	return IRQ_HANDLED;
}

static irqreturn_t ad7292_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad7292_state *st = iio_priv(indio_dev);
	int ret;

	dev_dbg(&st->spi->dev, "interrupt %d:", irq);
	dev_dbg(&st->spi->dev, "interrupt %d: buf_en: %d",
		irq, iio_buffer_enabled(indio_dev));
	dev_dbg(&st->spi->dev, "interrupt %d: trig base: %d",
		irq, st->trig->subirq_base);
	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "interrupt %d: gpiod_get_raw_value ret: %d",
		irq, ret);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);

	return IRQ_HANDLED;
};


static int ad7292_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad7292_state *st = iio_priv(indio_dev);
	int ret;

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "preenable: gpiod_get_raw_value ret: %d", ret);

	ret = ad7292_enable_gpio(st, 6, 1);
	dev_dbg(&st->spi->dev, "preenable: enable BUSY ret: %d", ret);
	//ret = ad7292_read_out_bank(st);

	/* Continuous capture mode keeping CS active */
	dev_dbg(&st->spi->dev, "preenable: activate CS");
	gpiod_set_value(st->spi->cs_gpiod, 1);

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "preenable: gpiod_get_raw_value ret: %d", ret);

	return 0;
}

static int ad7292_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7292_state *st = iio_priv(indio_dev);
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d8,
			.len = 2,
			.cs_change = 1
		}
	};
	int ret;

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "postenable: gpiod_get_raw_value ret: %d", ret);

	iio_triggered_buffer_postenable(indio_dev); //TODO remove - Lars' patch

	dev_dbg(&st->spi->dev, "postenable: Issue conversion command");

	/* AD7292 enters in conversion mode after the issue of a conversion command */
	st->d8[0] = AD7292_REG_ADC_CH(3); // TODO Using analog channel 3 for test
	st->d8[1] = AD7292_RD_FLAG_MSK(AD7292_REG_CONV_COMM);
	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "postenable: gpiod_get_raw_value ret: %d", ret);

	return 0;
}

static int ad7292_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7292_state *st = iio_priv(indio_dev);

	dev_dbg(&st->spi->dev, "predisable: start/end");

	return iio_triggered_buffer_predisable(indio_dev); //TODO return 0 - Lars' patch
}

int ad7292_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad7292_state *st = iio_priv(indio_dev);
	int ret;

	//TODO check BUSY pin before pulling CS high?
	//From datasheet: CS line must remain low while the ADC conversion is
	//in progress to prevent possible corruption of the ADC result.

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "postdisable: gpiod_get_raw_value ret: %d", ret);

	dev_dbg(&st->spi->dev, "postdisable: pull CS up");
	/* Disable CS */
	gpiod_set_value(st->spi->cs_gpiod, 0);

	ret = gpiod_get_raw_value(st->spi->cs_gpiod);
	dev_dbg(&st->spi->dev, "postdisable: gpiod_get_raw_value ret: %d", ret);

	dev_dbg(&st->spi->dev, "postdisable: disable gpio");
	ret = ad7292_enable_gpio(st, 6, 0);

	return 0;
}

/**
 * TODO
 * I'm not sure where/when to enable/disable the gpio for BUSY pin.
 * I've been experimenting with setup_ops but still don't have a clear answer for it.
 */
static const struct iio_buffer_setup_ops ad7292_buffer_ops = {
	.preenable = &ad7292_buffer_preenable,
	.postenable = &ad7292_buffer_postenable,
	.predisable = &ad7292_buffer_predisable,
	.postdisable = &ad7292_buffer_postdisable
};

static int ad7292_trig_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ad7292_state *st = iio_priv(indio_dev);
	int regval;

	regval = ad7292_spi_subreg_read(st, AD7292_REG_CONF_BANK,
					AD7292_BANK_REG_OUT_DRV, 2);
	if (regval < 0)
		return regval;

	return FIELD_GET(BIT(6), regval);
}

static int ad7292_validate_trigger(struct iio_dev *indio_dev,
				   struct iio_trigger *trig)
{
	struct ad7292_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_trigger_ops ad7292_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
	.set_trigger_state = &ad7292_trig_set_state,
};

static const struct iio_info ad7292_info = {
	.validate_trigger = &ad7292_validate_trigger,
	.read_raw = ad7292_read_raw,
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
	struct device_node *child;
	bool diff_channels = 0;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	ret = ad7292_spi_reg_read(st, AD7292_REG_VENDOR_ID);
	if (ret != ADI_VENDOR_ID) {
		dev_err(&spi->dev, "Wrong vendor id 0x%x\n", ret);
		return -EINVAL;
	}

	spi_set_drvdata(spi, indio_dev);
	mutex_init(&st->lock);

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
		/* Use the internal voltage reference. */
		st->vref_mv = 1250;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad7292_info;

	for_each_available_child_of_node(spi->dev.of_node, child) {
		diff_channels = of_property_read_bool(child, "diff-channels");
		if (diff_channels)
			break;
	}

	if (diff_channels) {
		indio_dev->num_channels = ARRAY_SIZE(ad7292_channels_diff);
		indio_dev->channels = ad7292_channels_diff;
	} else {
		indio_dev->num_channels = ARRAY_SIZE(ad7292_channels);
		indio_dev->channels = ad7292_channels;
	}

	st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
					  indio_dev->name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad7292_trigger_ops;
	st->trig->dev.parent = &spi->dev;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(&spi->dev, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	dev_dbg(&spi->dev, "probe spi->irq: %d\n", spi->irq);
	ad7292_read_out_bank(st);
	// I may use iio_trigger_generic_data_rdy_poll istead of ad7292_interrupt
	ret = devm_request_irq(&spi->dev, spi->irq,
			       &ad7292_interrupt,
			       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			       indio_dev->name, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad7292_trigger_handler,
					      &ad7292_buffer_ops);
	if (ret)
		return ret;

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
MODULE_LICENSE("GPL v2");
