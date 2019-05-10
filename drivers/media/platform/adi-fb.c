// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 Analog Devices Inc.
 * ADI Frame Buffer Driver
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <media/media-device.h>

/* DMA defines */
#define DMAC_REG_CTRL 0x400
#define DMAC_REG_START_TRANSFER 0x408
#define DMAC_REG_FLAGS 0x40c
#define DMAC_REG_DEST_ADDRESS 0x410
#define DMAC_REG_SRC_ADDRESS 0x414
#define DMAC_REG_X_LENGTH 0x418
#define DMAC_REG_Y_LENGTH 0x41c
#define DMAC_REG_DEST_STRIDE 0x420
#define DMAC_REG_SRC_STRIDE 0x424
#define DMAC_REG_FRAME_LOCK_CONFIG 0x454
#define DMAC_REG_FRAME_LOCK_STRIDE 0x458

#define DMAC_CTRL_ENABLE (1 << 0)
#define DMAC_TRANSFER_SUBMIT (1 << 0)

#define DMAC_FLAGS_CYCLIC (1 << 0)
#define DMAC_FLAGS_TLAST (1 << 1)
#define DMAC_FLAGS_FLOCK (1 << 3)

struct hdl_subdev {
	void __iomem *tx_dma_regs;
	void __iomem *rx_dma_regs;
};

struct frame_buffer {
	struct media_device media_dev;

	struct resource video_ram_buf;
	struct hdl_subdev hdl_subdev;

	u32 num_frames;
	u32 mode;
	u32 distance;
	//resolution[0] width, resolution[1] height
	u32 resolution[2];
	u32 dwidth;
};

static u32 adi_fb_reg_read(void __iomem *base_addr,
			   u32 addr)
{
	return ioread32(base_addr + addr);
}

static void adi_fb_reg_write(void __iomem *base_addr,
			     u32 addr,
			     u32 value)
{
	iowrite32(value, base_addr + addr);
}

static void adi_fb_reg_clr(void __iomem *base_addr,
			   u32 addr,
			   u32 clr)
{
	adi_fb_reg_write(base_addr,
			 addr,
			 adi_fb_reg_read(base_addr, addr) & ~clr);
}

static void adi_fb_reg_set(void __iomem *base_addr,
			   u32 addr,
			   u32 set)
{
	adi_fb_reg_write(base_addr,
			 addr,
			 adi_fb_reg_read(base_addr, addr) | set);
}

static void adi_fb_init(struct frame_buffer *buff)
{
	// reset tx DMAC
	adi_fb_reg_clr(buff->hdl_subdev.tx_dma_regs, DMAC_REG_CTRL,
		       DMAC_CTRL_ENABLE);
	// reset rx DMAC
	adi_fb_reg_clr(buff->hdl_subdev.rx_dma_regs, DMAC_REG_CTRL,
		       DMAC_CTRL_ENABLE);

	// Init RX DMAC
	adi_fb_reg_set(buff->hdl_subdev.rx_dma_regs, DMAC_REG_CTRL,
		       DMAC_CTRL_ENABLE);           // enable DMAC
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs, DMAC_REG_FLAGS,
			 (
				 DMAC_FLAGS_FLOCK  |
				 DMAC_FLAGS_CYCLIC |
				 DMAC_FLAGS_TLAST  |
				 0));                   // enable circular mode
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs, DMAC_REG_DEST_ADDRESS,
			 buff->video_ram_buf.start);    // start address
	// h size
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs, DMAC_REG_X_LENGTH,
			 ((buff->resolution[0] * buff->dwidth) - 1));
	// h offset
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs, DMAC_REG_DEST_STRIDE,
			 (buff->resolution[0] * buff->dwidth));
	// v size
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs, DMAC_REG_Y_LENGTH,
			 (buff->resolution[1] - 1));

	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs,
			 DMAC_REG_FRAME_LOCK_CONFIG,
			 (buff->num_frames |            // FLOCK_NUMFRAMES
			  ((buff->distance) << 16))     // FLOCK_FRAMEDISTANCE-1
			 );
	adi_fb_reg_write(buff->hdl_subdev.rx_dma_regs,
			 DMAC_REG_FRAME_LOCK_STRIDE,
			 (buff->resolution[0] * buff->resolution[1] *
			  buff->dwidth));
	// total active
	adi_fb_reg_set(buff->hdl_subdev.rx_dma_regs, DMAC_REG_START_TRANSFER,
		       DMAC_TRANSFER_SUBMIT);           // submit transfer

	// Init TX DMAC
	adi_fb_reg_set(buff->hdl_subdev.tx_dma_regs, DMAC_REG_CTRL,
		       DMAC_CTRL_ENABLE);           // enable DMAC
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs, DMAC_REG_FLAGS,
			 (
				 DMAC_FLAGS_FLOCK  |
				 DMAC_FLAGS_CYCLIC |
				 DMAC_FLAGS_TLAST  |
				 0));         // enable circular mode

	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs, DMAC_REG_SRC_ADDRESS,
			 buff->video_ram_buf.start);         // start address
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs, DMAC_REG_X_LENGTH,
			 ((buff->resolution[0] * buff->dwidth) - 1));
	// h size
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs, DMAC_REG_SRC_STRIDE,
			 (buff->resolution[0] * buff->dwidth));
	// h offset
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs, DMAC_REG_Y_LENGTH,
			 (buff->resolution[1] - 1));
	// v size
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs,
			 DMAC_REG_FRAME_LOCK_CONFIG,
			 (buff->num_frames |            // FLOCK_NUMFRAMES
			  (1 << 9) |
			  ((buff->distance) << 16))     // FLOCK_FRAMEDISTANCE-1
			 );
	// total active
	adi_fb_reg_write(buff->hdl_subdev.tx_dma_regs,
			 DMAC_REG_FRAME_LOCK_STRIDE,
			 (buff->resolution[0] * buff->resolution[1] *
			  buff->dwidth));
	// submit transfer
	adi_fb_reg_set(buff->hdl_subdev.tx_dma_regs, DMAC_REG_START_TRANSFER,
		       DMAC_TRANSFER_SUBMIT);
}

static int frame_buffer_probe(struct platform_device *pdev)
{
	struct frame_buffer *frm_buff;
	struct device_node *np;
	struct resource *res;
	int ret;

	frm_buff = devm_kzalloc(&pdev->dev, sizeof(struct frame_buffer),
				GFP_KERNEL);
	if (!frm_buff) {
		ret = ENOMEM;
		goto err;
	}

	/* Get reserved memory region from Device-tree */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np)
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");

	ret = of_address_to_resource(np, 0, &frm_buff->video_ram_buf);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		goto err;
	}
	dev_info(&pdev->dev, "Allocated reserved memory, paddr: 0x%0X\n",
		 frm_buff->video_ram_buf.start);

	/* Get physical address of DMAs*/
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "tx_dma");
	frm_buff->hdl_subdev.tx_dma_regs =
		devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(frm_buff->hdl_subdev.tx_dma_regs))
		return PTR_ERR(frm_buff->hdl_subdev.tx_dma_regs);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rx_dma");
	frm_buff->hdl_subdev.rx_dma_regs =
		devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(frm_buff->hdl_subdev.rx_dma_regs))
		return PTR_ERR(frm_buff->hdl_subdev.rx_dma_regs);

	/* Get frames number */
	if (device_property_read_u32(&pdev->dev, "flock,frm-buf-nr",
				     &frm_buff->num_frames)) {
		frm_buff->num_frames = 3;
		dev_info(&pdev->dev, "No frames number specified. Using %d\n",
			 frm_buff->num_frames);

	}

	/* Get frame distance */
	if (device_property_read_u32(&pdev->dev, "flock,distance",
				     &frm_buff->distance)) {
		frm_buff->distance = 0;
		dev_info(&pdev->dev, "No distance specified. Using %d\n",
			 frm_buff->distance);
	}

	/* Get operating mode */
	if (device_property_read_u32(&pdev->dev, "flock,mode",
				     &frm_buff->mode))
		dev_info(&pdev->dev,
			 "No operating mode specified. Using framelock\n");

	/* Get data width */
	if (device_property_read_u32(&pdev->dev, "flock,dwidth",
				     &frm_buff->dwidth)) {
		frm_buff->dwidth = 4;
		dev_info(&pdev->dev, "No data width specified. Using %d byte\n",
			 frm_buff->dwidth);
	}

	/* Get resolution mode */
	if (device_property_read_u32_array(&pdev->dev, "flock,resolution",
					   frm_buff->resolution, 2)) {
		frm_buff->resolution[0] = 1920;
		frm_buff->resolution[1] = 1080;
		dev_info(&pdev->dev,
			 "No resolution specified. Using default %d x %d\n",
			 frm_buff->resolution[0], frm_buff->resolution[1]);
	}

	adi_fb_init(frm_buff);

	frm_buff->media_dev.dev = &pdev->dev;

	strlcpy(frm_buff->media_dev.model, "ADI Frame Buffer",
		sizeof(frm_buff->media_dev.model));

	frm_buff->media_dev.hw_revision = 0;

	media_device_init(&frm_buff->media_dev);

	ret = media_device_register(&frm_buff->media_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register media_device\n");
		goto err;
	}

	return 0;

err:
	return ret;
}

static int frame_buffer_remove(struct platform_device *pdev)
{
	struct frame_buffer *frm_buff = platform_get_drvdata(pdev);

	media_device_unregister(&frm_buff->media_dev);

	return 0;
}

static const struct of_device_id frame_buffer_of_match[] = {
	{ .compatible = "adi,framebuffer-0.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, frame_buffer_of_match);

static struct platform_driver frame_buffer_driver = {
	.driver			= {
		.name		= "frame-buffer",
		.of_match_table = frame_buffer_of_match,
	},
	.probe			= frame_buffer_probe,
	.remove			= frame_buffer_remove,
};
module_platform_driver(frame_buffer_driver);

MODULE_AUTHOR("Bogdan Togoean <bogdan.togorean@analog.com>");
MODULE_DESCRIPTION("Analog Devices Frame Buffer");
MODULE_LICENSE("GPL");
