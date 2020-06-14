/* linux/arch/arm/mach-s3c24xx/mach-mini2451.c
 *
 * Copyright (c) 2012 FriendlyARM (www.arm9.net)
 *
 * Copyright (c) 2009 Yauhen Kharuzhy <jekhor@gmail.com>,
 *	as part of OpenInkpot project
 * Copyright (c) 2009 Promwad Innovation Company
 *	Yauhen Kharuzhy <yauhen.kharuzhy@promwad.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>

#include <mach/idle.h>
#include <mach/leds-gpio.h>
#include <plat/iic.h>

#include <plat/s3c2416.h>
#include <plat/gpio-cfg.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/nand.h>
#include <plat/sdhci.h>
#include <plat/udc.h>
#include <linux/platform_data/s3c-hsudc.h>

#include <plat/regs-fb-v4.h>
#include <plat/fb.h>
#include <mach/s3cfb.h>
#include <mach/board-revision.h>

#include <plat/common-smdk.h>

static struct map_desc mini2451_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */

	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}
};

#define UCON (S3C2410_UCON_DEFAULT	| \
		S3C2440_UCON_PCLK	| \
		S3C2443_UCON_RXERR_IRQEN)

#define ULCON (S3C2410_LCON_CS8 | S3C2410_LCON_PNONE)

#define UFCON (S3C2410_UFCON_RXTRIG8	| \
		S3C2410_UFCON_FIFOMODE	| \
		S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg mini2451_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	/* IR port */
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
#ifdef CONFIG_IRDA
		.ulcon	     = ULCON | 0x50,
#else
		.ulcon	     = ULCON,
#endif
		.ufcon	     = UFCON,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	}
};

static void mini2451_hsudc_gpio_init(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 0);
}

static void mini2451_hsudc_gpio_uninit(void)
{
	s3c2410_modify_misccr(S3C2416_MISCCR_SEL_SUSPND, 1);
}

static struct s3c24xx_hsudc_platdata mini2451_hsudc_platdata = {
	.epnum = 9,
	.gpio_init = mini2451_hsudc_gpio_init,
	.gpio_uninit = mini2451_hsudc_gpio_uninit,
};

/* HW revision */
static int mini2451_hw_rev;

static void __init mini2451_init_hw_rev(void)
{
	struct gpio hw_rev_gpios[] = {
		{ S3C2410_GPG(13), GPIOF_IN, "hw_rev0" },
		{ S3C2410_GPG(14), GPIOF_IN, "hw_rev1" },
		{ S3C2410_GPG(15), GPIOF_IN, "hw_rev2" },
	};
	int i, ret;

	ret = gpio_request_array(hw_rev_gpios,
			ARRAY_SIZE(hw_rev_gpios));
	BUG_ON(ret);

	for (i = 0; i < ARRAY_SIZE(hw_rev_gpios); i++)
		mini2451_hw_rev |= gpio_get_value(hw_rev_gpios[i].gpio) << i;

	printk("HW revision: %d\n", mini2451_hw_rev);
}

int board_get_revision(void)
{
	return mini2451_hw_rev;
}

static struct s3c_fb_pd_win mini2451_fb_win[] = {
	[0] = {
		.default_bpp= 16,
		.max_bpp	= 32,
		.xres		= 800,
		.yres		= 480,
	},
};

static struct fb_videomode mini2451_lcd_timing = {
	.pixclock		= 41094,
	.left_margin	= 8,
	.right_margin	= 13,
	.upper_margin	= 7,
	.lower_margin	= 5,
	.hsync_len		= 3,
	.vsync_len		= 1,
	.xres			= 800,
	.yres			= 480,
};

static void s3c2416_fb_gpio_setup_24bpp(void)
{
	unsigned int gpio;

	for (gpio = S3C2410_GPC(1); gpio <= S3C2410_GPC(4); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPC(8); gpio <= S3C2410_GPC(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	for (gpio = S3C2410_GPD(0); gpio <= S3C2410_GPD(15); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}

static struct s3c_fb_platdata mini2451_fb_platdata = {
	.win[0]		= &mini2451_fb_win[0],
	.vtiming	= &mini2451_lcd_timing,
	.setup_gpio	= s3c2416_fb_gpio_setup_24bpp,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
};

static void __init mini2451_fb_init_pdata(struct s3c_fb_platdata *pd) {
	struct s3cfb_lcd *lcd;
	struct s3c_fb_pd_win *win = pd->win[0];
	struct fb_videomode *mode = pd->vtiming;
	unsigned long val = 0;
	u64 pixclk = 1000000000000ULL;
	u32 div;

	lcd = mini2451_get_lcd();

	win->default_bpp	= lcd->bpp < 25 ? lcd->bpp : 24;
	win->xres			= lcd->width;
	win->yres			= lcd->height;

	mode->left_margin	= lcd->timing.h_bp;
	mode->right_margin	= lcd->timing.h_fp;
	mode->upper_margin	= lcd->timing.v_bp;
	mode->lower_margin	= lcd->timing.v_fp;
	mode->hsync_len		= lcd->timing.h_sw;
	mode->vsync_len		= lcd->timing.v_sw;
	mode->xres			= lcd->width;
	mode->yres			= lcd->height;

	/* calculates pixel clock */
	div  = mode->left_margin + mode->hsync_len + mode->right_margin +
		mode->xres;
	div *= mode->upper_margin + mode->vsync_len + mode->lower_margin +
		mode->yres;
	div *= lcd->freq ? : 60;

	do_div(pixclk, div);

	mode->pixclock		= pixclk;

	/* initialize signal polarity of RGB interface */
	if (lcd->polarity.rise_vclk)
		val |= VIDCON1_INV_VCLK;
	if (lcd->polarity.inv_hsync)
		val |= VIDCON1_INV_HSYNC;
	if (lcd->polarity.inv_vsync)
		val |= VIDCON1_INV_VSYNC;
	if (lcd->polarity.inv_vden)
		val |= VIDCON1_INV_VDEN;

	pd->vidcon1 = val;
}

/* Nand flash */
struct mtd_partition mini2451_nand_part[] = {
	{
		.name		= "Bootloader",
		.offset		= 0,
		.size		= (4 * 128 *SZ_1K),
	}, {
		.name		= "Kernel",
		.offset		= (4 * 128 *SZ_1K),
		.size		= (5*SZ_1M) ,
	}, {
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	}
};

static struct s3c2410_nand_set mini2451_nand_sets[] = {
	[0] = {
		.name       = "nand",
		.nr_chips   = 1,
		.nr_partitions  = ARRAY_SIZE(mini2451_nand_part),
		.partitions = mini2451_nand_part,
	},
};

static struct s3c2410_platform_nand mini2451_nand_info = {
	.tacls      = 25,
	.twrph0     = 55,
	.twrph1     = 40,
	.nr_sets    = ARRAY_SIZE(mini2451_nand_sets),
	.sets       = mini2451_nand_sets,
};

static struct s3c_sdhci_platdata mini2451_hsmmc0_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio	= S3C2410_GPF(6),
	.ext_cd_gpio_invert	= 1,
	.disable_acmd12	= 1,
};

static int gpio_hsmmc1_ro = S3C2410_GPG(1);

static int mini2451_hsmmc1_get_ro(struct mmc_host *mmc) {
	return gpio_get_value(gpio_hsmmc1_ro);
}

static struct s3c_sdhci_platdata mini2451_hsmmc1_pdata __initdata = {
	.max_width		= 4,
	.cd_type		= S3C_SDHCI_CD_NONE,
	.get_ro			= mini2451_hsmmc1_get_ro,
};

static void __init mini2451_hsmmc_gpio_setup(void)
{
	s3c_gpio_cfgrange_nopull(S3C2410_GPJ(13), 3, S3C_GPIO_SFN(2));

	gpio_request(gpio_hsmmc1_ro, "sdwp#1");
	gpio_direction_input(gpio_hsmmc1_ro);
	s3c_gpio_setpull(gpio_hsmmc1_ro, S3C_GPIO_PULL_UP);
}

static void __init mini2451_wifi_init(void)
{
	/* WIFI 1 (external): PDn --> RESETn */
	gpio_request(S3C2410_GPB(3), "WIFI_PD");
	gpio_direction_output(S3C2410_GPB(3), 1);
	udelay(50);
	gpio_free(S3C2410_GPB(3));

	gpio_request(S3C2410_GPB(4), "WIFI_RESET");
	gpio_direction_output(S3C2410_GPB(4), 0);
	udelay(100);
	gpio_set_value(S3C2410_GPB(4), 1);
	gpio_free(S3C2410_GPB(4));
}

#if defined(CONFIG_DM9000)
#include <linux/dm9000.h>

#define MINI2451_DM9K_A		(0x08001000)
#define MINI2451_DM9K_F		(MINI2451_DM9K_A + 0x300C)

static struct resource mini2451_dm9000_resources[] = {
	[0] = DEFINE_RES_MEM(MINI2451_DM9K_A, 4),
	[1] = DEFINE_RES_MEM(MINI2451_DM9K_F, 4),
	[2] = DEFINE_RES_NAMED(IRQ_EINT7, 1, NULL,
					IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
};

static struct dm9000_plat_data mini2451_dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
	.dev_addr	= { 0x08, 0x90, 0x00, 0xa0, 0x02, 0x10 },
};

struct platform_device mini2451_dm9000 = {
	.name		= "dm9000",
	.id			= -1,
	.num_resources	= ARRAY_SIZE(mini2451_dm9000_resources),
	.resource	= mini2451_dm9000_resources,
	.dev		= {
		.platform_data	= &mini2451_dm9000_platdata,
	},
};

static void __init mini2451_dm9000_update(unsigned int addr)
{
	addr += 0x0300;
	mini2451_dm9000_resources[0].start = addr;
	mini2451_dm9000_resources[0].end = addr + 4 - 1;

	addr += 0x000C;
	mini2451_dm9000_resources[1].start = addr;
	mini2451_dm9000_resources[1].end = addr + 4 - 1;
}
#endif

#ifdef CONFIG_SND_SOC_WM8960_MINI2451
#include <sound/wm8960.h>
static struct wm8960_data wm8960_pdata = {
	.capless	= 0,
	.dres		= WM8960_DRES_400R,
};
#endif

static struct i2c_board_info mini2451_i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8960_MINI2451
	{
		I2C_BOARD_INFO("wm8960", 0x1a),
		.platform_data = &wm8960_pdata,
	},
#endif
};

struct platform_device mini2451_audio = {
	.name		= "mini2451-audio",
	.id			= -1,
};

static struct platform_device mini2451_1wire = {
	.name		= "mini2451_1wire",
	.id			= -1,
};

static struct platform_device mini2451_adc = {
	.name		= "mini2451_adc",
	.id			= -1,
};

static struct platform_device *mini2451_devices[] __initdata = {
	&s3c_device_fb,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_nand,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_usb_hsudc,
	&s3c_device_ohci,
	&s3c_device_iis,
	&s3c_device_adc,
	&s3c_device_rtc,
	&s3c_device_timer[0],
	&samsung_asoc_dma,
#if defined(CONFIG_DM9000)
	&mini2451_dm9000,
#endif
	&mini2451_audio,
	&mini2451_1wire,
	&mini2451_adc,
};

static void __init mini2451_map_io(void)
{
	s3c24xx_init_io(mini2451_iodesc, ARRAY_SIZE(mini2451_iodesc));
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(mini2451_uartcfgs, ARRAY_SIZE(mini2451_uartcfgs));
}

static void __init mini2451_machine_init(void)
{
	mini2451_init_hw_rev();
	if (is_board_rev_B()) {
		gpio_hsmmc1_ro = S3C2410_GPJ(15);
		mini2451_dm9000_update(S3C2410_CS4);
	}

	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, mini2451_i2c_devs0,
			ARRAY_SIZE(mini2451_i2c_devs0));

	mini2451_fb_init_pdata(&mini2451_fb_platdata);
	s3c_fb_set_platdata(&mini2451_fb_platdata);

#ifdef CONFIG_MTD_NAND_S3C
	s3c_device_nand.name = "s3c2450-nand";
#endif
	s3c_nand_set_platdata(&mini2451_nand_info);

	s3c_sdhci0_set_platdata(&mini2451_hsmmc0_pdata);
	s3c_sdhci1_set_platdata(&mini2451_hsmmc1_pdata);

	mini2451_hsmmc_gpio_setup();
	mini2451_wifi_init();

	s3c24xx_hsudc_set_platdata(&mini2451_hsudc_platdata);

	gpio_request(S3C2410_GPB(1), "Display Power");
	gpio_direction_output(S3C2410_GPB(1), 1);
	gpio_free(S3C2410_GPB(1));

	platform_add_devices(mini2451_devices, ARRAY_SIZE(mini2451_devices));

#if defined(CONFIG_S3C24XX_SMDK)
	smdk_machine_init();
#endif
}

void mini2451_hsudc_init_phy(void)
{
#define HSUDC_RESET		((1 << 2) | (1 << 0))
	u32 cfg;

	cfg = readl(S3C2443_PWRCFG) | S3C2443_PWRCFG_USBPHY;
	writel(cfg, S3C2443_PWRCFG);
	udelay(5);

	writel(0, S3C2443_PHYCTRL);

	cfg = readl(S3C2443_PHYPWR) | (1 << 31);
	writel(cfg, S3C2443_PHYPWR);

	cfg = readl(S3C2443_UCLKCON);
	cfg |= (1 << 31) | (1 << 2);
	writel(cfg, S3C2443_UCLKCON);
	udelay(5);

	cfg = readl(S3C2443_URSTCON);
	cfg |= HSUDC_RESET;
	writel(cfg, S3C2443_URSTCON);
	mdelay(1);

	cfg = readl(S3C2443_URSTCON);
	cfg &= ~HSUDC_RESET;
	writel(cfg, S3C2443_URSTCON);
	udelay(5);

#if 0
	printk("usb-phy: PWRCFG=%08x, PHYCTRL=%08x, PHYPWR=%08x, UCLKCON=%08x\n",
			readl(S3C2443_PWRCFG), readl(S3C2443_PHYCTRL),
			readl(S3C2443_PHYPWR), readl(S3C2443_UCLKCON));
#endif
}
EXPORT_SYMBOL(mini2451_hsudc_init_phy);

MACHINE_START(SMDK2416, "MINI2451")
	/* Maintainer: FriendlyARM (www.arm9.net) */
	.atag_offset	= 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= mini2451_map_io,
	.init_machine	= mini2451_machine_init,
	.timer		= &s3c24xx_timer,
	.restart	= s3c2416_restart,
MACHINE_END

