/*
 * MMC definitions for OMAP2
 *
 * Copyright (C) 2006 Nokia Corporation
 * embedded_sdio_data contribution from San Mehat
 *
 * Support for embedded_sdio
 * 	San Mehat		<san@google.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __OMAP2_MMC_H
#define __OMAP2_MMC_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#ifdef CONFIG_MMC_EMBEDDED_SDIO
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#endif

#include <plat/board.h>

#define OMAP15XX_NR_MMC		1
#define OMAP16XX_NR_MMC		2
#define OMAP1_MMC_SIZE		0x080
#define OMAP1_MMC1_BASE		0xfffb7800
#define OMAP1_MMC2_BASE		0xfffb7c00	/* omap16xx only */

#define OMAP24XX_NR_MMC		2
#define OMAP34XX_NR_MMC		3
#define OMAP44XX_NR_MMC		5
#define OMAP2420_MMC_SIZE	OMAP1_MMC_SIZE
#define OMAP3_HSMMC_SIZE	0x200
#define OMAP4_HSMMC_SIZE	0x1000
#define OMAP2_MMC1_BASE		0x4809c000
#define OMAP2_MMC2_BASE		0x480b4000
#define OMAP3_MMC3_BASE		0x480ad000
#define OMAP4_MMC4_BASE		0x480d1000
#define OMAP4_MMC5_BASE		0x480d5000
#define OMAP4_MMC_REG_OFFSET	0x100
#define HSMMC5			(1 << 4)
#define HSMMC4			(1 << 3)
#define HSMMC3			(1 << 2)
#define HSMMC2			(1 << 1)
#define HSMMC1			(1 << 0)

#define OMAP_MMC_MAX_SLOTS	2

#ifdef CONFIG_MMC_EMBEDDED_SDIO
struct embedded_sdio_data {
	struct sdio_cis cis;
	struct sdio_cccr cccr;
	struct sdio_embedded_func *funcs;
	int num_funcs;
	unsigned int quirks;
};
#endif

struct omap_mmc_platform_data {
	/* back-link to device */
	struct device *dev;

	/* number of slots per controller */
	unsigned nr_slots:2;

	/* set if your board has components or wiring that limits the
	 * maximum frequency on the MMC bus */
	unsigned int max_freq;

	/* switch the bus to a new slot */
	int (* switch_slot)(struct device *dev, int slot);
	/* initialize board-specific MMC functionality, can be NULL if
	 * not supported */
	int (* init)(struct device *dev);
	void (* cleanup)(struct device *dev);
	void (* shutdown)(struct device *dev);

	/* To handle board related suspend/resume functionality for MMC */
	int (*suspend)(struct device *dev, int slot);
	int (*resume)(struct device *dev, int slot);

	/* Return context loss count due to PM states changing */
	int (*get_context_loss_count)(struct device *dev);

	u64 dma_mask;

	struct omap_mmc_slot_data {

		/* 4 wire signaling is optional, and is used for SD/SDIO/HSMMC;
		 * 8 wire signaling is also optional, and is used with HSMMC
		 */
		u8 wires;

		/*
		 * nomux means "standard" muxing is wrong on this board, and
		 * that board-specific code handled it before common init logic.
		 */
		unsigned nomux:1;

		/* switch pin can be for card detect (default) or card cover */
		unsigned cover:1;

		/* use the internal clock */
		unsigned internal_clock:1;

		/* nonremovable e.g. eMMC */
		unsigned nonremovable:1;

		/* Try to sleep or power off when possible */
		unsigned power_saving:1;

		int switch_pin;			/* gpio (card detect) */
		int gpio_wp;			/* gpio (write protect) */

		int (* set_bus_mode)(struct device *dev, int slot, int bus_mode);
		int (* set_power)(struct device *dev, int slot, int power_on, int vdd);
		int (* get_ro)(struct device *dev, int slot);
		int (*set_sleep)(struct device *dev, int slot, int sleep,
				 int vdd, int cardsleep);

		/* return MMC cover switch state, can be NULL if not supported.
		 *
		 * possible return values:
		 *   0 - closed
		 *   1 - open
		 */
		int (* get_cover_state)(struct device *dev, int slot);

		const char *name;
		u32 ocr_mask;

		/* Card detection IRQs */
		int card_detect_irq;
		int (* card_detect)(int irq);

		unsigned int ban_openended:1;

#ifdef CONFIG_MMC_EMBEDDED_SDIO
		struct embedded_sdio_data *embedded_sdio;
		int (*register_status_notify)
			(void (*callback)(int , void *), void *);
#endif

	} slots[OMAP_MMC_MAX_SLOTS];
};

/* called from board-specific card detection service routine */
extern void omap_mmc_notify_cover_event(struct device *dev, int slot, int is_closed);

#if	defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)
void omap1_init_mmc(struct omap_mmc_platform_data **mmc_data,
				int nr_controllers);
void omap2_init_mmc(struct omap_mmc_platform_data **mmc_data,
				int nr_controllers);
int omap_mmc_add(const char *name, int id, unsigned long base,
				unsigned long size, unsigned int irq,
				struct omap_mmc_platform_data *data);
#else
static inline void omap1_init_mmc(struct omap_mmc_platform_data **mmc_data,
				int nr_controllers)
{
}
static inline void omap2_init_mmc(struct omap_mmc_platform_data **mmc_data,
				int nr_controllers)
{
}
static inline int omap_mmc_add(const char *name, int id, unsigned long base,
				unsigned long size, unsigned int irq,
				struct omap_mmc_platform_data *data)
{
	return 0;
}

#endif
#endif
