/*
 * IMAGEON V4L2 bridge driver
 *
 * Copyright 2015 Analog Devices Inc.
 *  Author: Dragos Bogdan <dragos.bogdan@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/adv7604.h>


#define INPUT_SUBDEV		0
#define OUTPUT_SUBDEV		1

/* HDMI HDL core defines */
#define AXI_HDMI_REG_RESET              0x040
#define AXI_HDMI_REG_CTRL               0x044
#define AXI_HDMI_REG_SOURCE_SEL         0x048
#define AXI_HDMI_REG_COLORPATTERN       0x04c
#define AXI_HDMI_REG_STATUS             0x05c
#define AXI_HDMI_REG_HTIMING1           0x400
#define AXI_HDMI_REG_HTIMING2           0x404
#define AXI_HDMI_REG_HTIMING3           0x408
#define AXI_HDMI_REG_VTIMING1           0x440
#define AXI_HDMI_REG_VTIMING2           0x444
#define AXI_HDMI_REG_VTIMING3           0x448


struct imageon_subdev {
	struct v4l2_async_subdev asd;
	struct v4l2_subdev *subdev;
};

struct hdl_subdev {
	void __iomem *tx_dma_regs;
	void __iomem *rx_dma_regs;
	void __iomem *tx_hdmi_regs;
	void __iomem *rx_hdmi_regs;
};

struct imageon_bridge {
	struct imageon_subdev imageon_subdev[2];
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;

	u8 input_edid_data[256];
	u8 input_edid_blocks;

	int irq;

	struct resource video_ram_buf;
	struct hdl_subdev hdl_subdev;

	u32 num_frames;
	u32 mode;
	u32 distance;
	//resolution[0] width, resolution[1] height
	u32 resolution[2];
	u8 dwidth;
};

enum detailedTimingElement
{
	PIXEL_CLOCK,
	H_ACTIVE_TIME,
	H_BLANKING_TIME,
	H_SYNC_OFFSET,
	H_SYNC_WIDTH_PULSE,
	V_ACTIVE_TIME,
	V_BLANKING_TIME,
	V_SYNC_OFFSET,
	V_SYNC_WIDTH_PULSE
};

static const unsigned long detailedTiming[7][9] =
{
	{25180000, 640, 144, 16, 96, 480, 29, 10, 2},
	{40000000, 800, 256, 40, 128, 600, 28, 1, 4},
	{65000000, 1024, 320, 136, 24, 768, 38, 3, 6},
	{74250000, 1280, 370, 110, 40, 720, 30, 5, 5},
	{84750000, 1360, 416, 136, 72, 768, 30, 3, 5},
	{108000000, 1600, 400, 32, 48, 900, 12, 3, 6},
	{148500000, 1920, 280, 44, 88, 1080, 45, 4, 5}
};

static struct imageon_bridge *
	notifier_to_imageon_bridge(struct v4l2_async_notifier *n)
{
	return container_of(n, struct imageon_bridge, notifier);
}

/*
 * Regsiter related operations
 */
static u32 imageon_bridge_reg_read(void __iomem *base_addr,
					u32 addr)
{
	return ioread32(base_addr + addr);
}

static void imageon_bridge_reg_write(void __iomem *base_addr,
					u32 addr,
					u32 value)
{
	iowrite32(value, base_addr + addr);
}

static void imageon_bridge_clr(void __iomem *base_addr,
					u32 addr,
					u32 clr)
{
	imageon_bridge_reg_write(base_addr,
			addr,
			imageon_bridge_reg_read(base_addr, addr) & ~clr);
}

static void imageon_bridge_set(void __iomem *base_addr,
					u32 addr,
					u32 set)
{
	imageon_bridge_reg_write(base_addr,
			addr,
			imageon_bridge_reg_read(base_addr, addr) | set);
}

static int imageon_bridge_hdmi_cores_init(struct imageon_bridge *bridge,
					  unsigned short horizontalActiveTime,
					  unsigned short horizontalBlankingTime,
					  unsigned short horizontalSyncOffset,
					  unsigned short horizontalSyncPulseWidth,
					  unsigned short verticalActiveTime,
					  unsigned short verticalBlankingTime,
					  unsigned short verticalSyncOffset,
					  unsigned short verticalSyncPulseWidth)
{


	unsigned short horizontalCount = 0;
	unsigned short verticalCount = 0;
	unsigned short horizontalBackPorch = 0;
	unsigned short verticalBackPorch = 0;
	unsigned short horizontalDeMin = 0;
	unsigned short horizontalDeMax = 0;
	unsigned short verticalDeMin = 0;
	unsigned short verticalDeMax = 0;

	bridge->resolution[0] = 1920;
	bridge->resolution[1] = 1080;

	horizontalCount = horizontalActiveTime +
			  horizontalBlankingTime;
	verticalCount = verticalActiveTime +
			verticalBlankingTime;
	horizontalBackPorch = horizontalBlankingTime -
			      horizontalSyncOffset -
			      horizontalSyncPulseWidth;
	verticalBackPorch = verticalBlankingTime -
			    verticalSyncOffset -
			    verticalSyncPulseWidth;
	horizontalDeMin = horizontalSyncPulseWidth +
			  horizontalBackPorch;
	horizontalDeMax = horizontalDeMin +
			  horizontalActiveTime;
	verticalDeMin = verticalSyncPulseWidth +
			verticalBackPorch;
	verticalDeMax = verticalDeMin +
			verticalActiveTime;

	/* Init RX HDMI core */
	imageon_bridge_reg_write(bridge->hdl_subdev.rx_hdmi_regs, AXI_HDMI_REG_RESET, 0x0);

	imageon_bridge_reg_write(bridge->hdl_subdev.rx_hdmi_regs, AXI_HDMI_REG_HTIMING1,
				 ((verticalActiveTime << 16) | horizontalActiveTime));

	/* Disable colorspace conversion on RX side */
	imageon_bridge_reg_write(bridge->hdl_subdev.rx_hdmi_regs, AXI_HDMI_REG_CTRL,
			 	0x01);

	// Enable RX HDMI interface
	imageon_bridge_reg_write(bridge->hdl_subdev.rx_hdmi_regs, AXI_HDMI_REG_RESET, 0x1);



	/* Init TX HDMI core */
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_RESET, 0x0);

	/* Disable colorspace conversion on TX side */
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_CTRL,
			 	0x05);

	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_HTIMING1,
				 ((bridge->resolution[0] << 16) | horizontalCount));
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_HTIMING2,
				 horizontalSyncPulseWidth);
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_HTIMING3,
				 ((horizontalDeMax << 16) | horizontalDeMin));
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_VTIMING1,
				 ((bridge->resolution[1] << 16) | verticalCount));
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_VTIMING2,
				 verticalSyncPulseWidth);
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_VTIMING3,
				 ((verticalDeMax << 16) | verticalDeMin));

	// Enable TX HDMI interface
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_RESET, 0x1);
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_SOURCE_SEL, 0x0);
	imageon_bridge_reg_write(bridge->hdl_subdev.tx_hdmi_regs, AXI_HDMI_REG_SOURCE_SEL, 0x1);

	return 0;
}

static int imageon_bridge_load_input_edid(struct platform_device *pdev,
	struct imageon_bridge *bridge)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "imageon_edid.bin", &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to load firmware: %d\n", ret);
		return ret;
	}

	if (fw->size > 256) {
		dev_err(&pdev->dev, "EDID firmware data too large\n");
		release_firmware(fw);
		bridge->input_edid_blocks = 0;
		return -EINVAL;
	}

	if (fw->size > 128)
		bridge->input_edid_blocks = 2;
	else
		bridge->input_edid_blocks = 1;

	memcpy(bridge->input_edid_data, fw->data, fw->size);

	release_firmware(fw);

	return 0;
}

static irqreturn_t imageon_bridge_hdmiio_int_handler(int irq, void *dev_id)
{
	struct imageon_bridge *bridge = dev_id;

	if (bridge->imageon_subdev[INPUT_SUBDEV].subdev != NULL)
		v4l2_subdev_call(bridge->imageon_subdev[INPUT_SUBDEV].subdev,
				core, interrupt_service_routine, 0, NULL);

	if (bridge->imageon_subdev[OUTPUT_SUBDEV].subdev != NULL)
		v4l2_subdev_call(bridge->imageon_subdev[OUTPUT_SUBDEV].subdev,
				core, interrupt_service_routine, 0, NULL);

	return IRQ_HANDLED;
}

static int imageon_bridge_async_bound(struct v4l2_async_notifier *notifier,
	struct v4l2_subdev *subdev, struct v4l2_async_subdev *asd)
{
	struct imageon_bridge *bridge = notifier_to_imageon_bridge(notifier);
	struct v4l2_subdev_edid edid = {
		.pad = 0,
		.start_block = 0,
		.blocks = bridge->input_edid_blocks,
		.edid = bridge->input_edid_data,
	};
	int ret;

	if (bridge->imageon_subdev[INPUT_SUBDEV].asd.match.fwnode.fwnode
			== of_fwnode_handle(subdev->dev->of_node)) {

		bridge->imageon_subdev[INPUT_SUBDEV].subdev = subdev;

		ret = v4l2_subdev_call(subdev, video, s_routing,
					ADV76XX_PAD_HDMI_PORT_A, 0, 0);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(subdev, pad, set_edid, &edid);
		if (ret)
			return ret;
	}

	if (bridge->imageon_subdev[OUTPUT_SUBDEV].asd.match.fwnode.fwnode
			== of_fwnode_handle(subdev->dev->of_node)) {

		bridge->imageon_subdev[OUTPUT_SUBDEV].subdev = subdev;

		ret = v4l2_subdev_call(bridge->imageon_subdev[OUTPUT_SUBDEV].subdev,
					video, s_stream, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int imageon_bridge_async_complete(struct v4l2_async_notifier *notifier)
{
	struct imageon_bridge *bridge = notifier_to_imageon_bridge(notifier);
	struct v4l2_subdev_format fmt;
	int ret;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = ADV7611_PAD_SOURCE;
	fmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
	ret = v4l2_subdev_call(bridge->imageon_subdev[INPUT_SUBDEV].subdev,
				pad, set_fmt, NULL, &fmt);
	if (ret)
		return ret;

	return 0;
}

static struct imageon_bridge *imageon_bridge_parse_dt(struct device *dev)
{
	struct imageon_bridge *bridge;
	struct device_node *ep = NULL;
	struct device_node *next;
	int index;

	bridge = devm_kzalloc(dev, sizeof(struct imageon_bridge), GFP_KERNEL);
	if (!bridge) {
		dev_err(dev, "could not allocate memory for imageon_bridge\n");
		return NULL;
	}

	for (index = 0; index < 2; index++) {
		next = of_graph_get_next_endpoint(dev->of_node, ep);
		if (!next) {
			return NULL;
		}
		ep = next;

		bridge->imageon_subdev[index].asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
		bridge->imageon_subdev[index].asd.match.fwnode.fwnode =
			of_fwnode_handle(of_graph_get_remote_port_parent(next));
	}

	return bridge;
}

static int imageon_bridge_probe(struct platform_device *pdev)
{
	struct imageon_bridge *bridge;
	struct v4l2_async_subdev **asubdevs;
	int ret;
	struct resource *res;
	struct clk *clk;
	u8 resolution = 6;

	bridge = imageon_bridge_parse_dt(&pdev->dev);
	if (bridge == NULL)
		return -ENOMEM;

	bridge->irq = platform_get_irq(pdev, 0);
	if (bridge->irq > 0) {
		ret = request_threaded_irq(bridge->irq, NULL,
					   imageon_bridge_hdmiio_int_handler,
					   IRQF_ONESHOT | IRQF_TRIGGER_LOW, dev_name(&pdev->dev),
					   bridge);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request irq\n");
			return ret;
		}
	}

	ret = imageon_bridge_load_input_edid(pdev, bridge);
	if (ret < 0)
		goto err;

	/* Get physical address of HDMI cores*/
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tx_hdmi");
	bridge->hdl_subdev.tx_hdmi_regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bridge->hdl_subdev.tx_hdmi_regs))
		dev_info(&pdev->dev, "No HDMI tx addres specified\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rx_hdmi");
	bridge->hdl_subdev.rx_hdmi_regs= devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bridge->hdl_subdev.rx_hdmi_regs))
		dev_info(&pdev->dev, "No HDMI rx addres specified\n");

	imageon_bridge_hdmi_cores_init(bridge, detailedTiming[resolution][H_ACTIVE_TIME],
				       detailedTiming[resolution][H_BLANKING_TIME],
				       detailedTiming[resolution][H_SYNC_OFFSET],
				       detailedTiming[resolution][H_SYNC_WIDTH_PULSE],
				       detailedTiming[resolution][V_ACTIVE_TIME],
				       detailedTiming[resolution][V_BLANKING_TIME],
				       detailedTiming[resolution][V_SYNC_OFFSET],
				       detailedTiming[resolution][V_SYNC_WIDTH_PULSE]);

	/* Get HDMI clock */
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	ret = clk_set_rate(clk, (unsigned long) 148500000);
	if (ret < 0)
		goto err;

	ret = clk_prepare(clk);
	if (ret < 0)
		goto err;

	ret = clk_enable(clk);
	if (ret < 0)
		goto err;

	bridge->media_dev.dev = &pdev->dev;
	strlcpy(bridge->media_dev.model, "IMAGEON V4L2 Bridge",
		sizeof(bridge->media_dev.model));
	bridge->media_dev.hw_revision = 0;

	media_device_init(&bridge->media_dev);

	ret = media_device_register(&bridge->media_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register media_device\n");
		goto err;
	}
	bridge->v4l2_dev.mdev = &bridge->media_dev;

	snprintf(bridge->v4l2_dev.name, sizeof(bridge->v4l2_dev.name),
		 "imageon_v4l2_bridge");

	ret = v4l2_device_register(&pdev->dev, &bridge->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register v4l2_device\n");
		goto err;
	}

	asubdevs = devm_kzalloc(&pdev->dev, sizeof(struct v4l2_async_subdev*) * 2,
				GFP_KERNEL);
	if (bridge == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	asubdevs[INPUT_SUBDEV] = &bridge->imageon_subdev[INPUT_SUBDEV].asd;
	asubdevs[OUTPUT_SUBDEV] = &bridge->imageon_subdev[OUTPUT_SUBDEV].asd;

	bridge->notifier.subdevs = asubdevs;
	bridge->notifier.num_subdevs = 2;
	bridge->notifier.bound = imageon_bridge_async_bound;
	bridge->notifier.complete = imageon_bridge_async_complete;

	ret = v4l2_async_notifier_register(&bridge->v4l2_dev,
					   &bridge->notifier);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device nodes\n");
		goto err;
	}

	ret = v4l2_device_register_subdev_nodes(&bridge->v4l2_dev);
	if (ret < 0)
		goto err;

	return 0;

err:
	if (bridge->irq > 0)
		free_irq(bridge->irq, pdev);
	return ret;
}

static int imageon_bridge_remove(struct platform_device *pdev)
{
	struct imageon_bridge *bridge = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&bridge->notifier);
	v4l2_device_unregister(&bridge->v4l2_dev);
	media_device_unregister(&bridge->media_dev);
	if (bridge->irq > 0)
		free_irq(bridge->irq, pdev);

	return 0;
}

static const struct of_device_id imageon_bridge_of_match[] = {
	{ .compatible = "adi,imageon-v4l2-bridge-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, imageon_bridge_of_match);

static struct platform_driver imageon_bridge_driver = {
	.driver = {
		.name = "imageon-bridge",
		.owner = THIS_MODULE,
		.of_match_table = imageon_bridge_of_match,
	},
	.probe = imageon_bridge_probe,
	.remove = imageon_bridge_remove,
};
module_platform_driver(imageon_bridge_driver);

MODULE_DESCRIPTION("Imageon video bridge");
MODULE_LICENSE("GPL v2");
