/*
 * linux/arch/arm/mach-omap2/board-oma3logic-product-id.c
 *
 * Copyright (C) 2009 Logic Product Development, Inc.
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <plat/hardware.h>

#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach-types.h>
#include <plat/control.h>
#include <plat/sram.h>

#include <plat/omap3logic-productid.h>
#include "mux.h"

static struct extracted_product_id_data {
	u32 part_number;
	u32 part_number_valid;
} extracted_product_id_data;

/* GPIO i2c code to access at88 chip */
enum {
	RX_MODE_FIRST_BYTE,
	RX_MODE_MIDDLE_BYTE,
	RX_MODE_NEXT_TO_LAST_BYTE,
	RX_MODE_LAST_BYTE,
	RX_MODE_ONE_BYTE
} I2C_RX_MODE;

typedef enum {
	GPIO_I2C_SDATA,
	GPIO_I2C_SCLK,
} GPIO_I2C_PIN;

static enum {
	GPIO_I2C_UNINIT,
	GPIO_I2C_STOPPED,
	GPIO_I2C_STARTED,
} gpio_i2c_bus_state;

typedef enum {
	GPIO_I2C_INPUT,
	GPIO_I2C_OUTPUT,
} GPIO_I2C_DIRECTION;

static int gpio_i2c_clock_high_width, gpio_i2c_clock_low_width;
static int gpio_i2c_coarse_delay;

#define DEBUG_PRODUCTION_DATA 0  // !0 to dump extraction data
#define DEBUG_PRODUCTION_DATA_BUF 0 // !0 to dump data read


#define 	MUX_VAL(OFFSET,VALUE)\
		__raw_writew((VALUE), OMAP34XX_CTRL_BASE + (OFFSET))

#define		CP(x)	(CONTROL_PADCONF_##x)
/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */

#define GPIO_I2C_GPIO_SCLK  184
#define GPIO_I2C_GPIO_SDATA 185

/* Put SCLK/SDA pins connected to the product ID into GPIO mode */
static void gpio_i2c_config_pins(void)
{

	/* Claim the I2C3 pins as GPIO */
	
	omap_mux_init_gpio(GPIO_I2C_GPIO_SCLK, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(GPIO_I2C_GPIO_SDATA, OMAP_PIN_INPUT_PULLUP);

	if (gpio_request(GPIO_I2C_GPIO_SCLK, "gpio_i2c4_sclk"))
		printk("%s: gpio_request(gpio_i2ce_sclk) failed\n", __FUNCTION__);
	if (gpio_request(GPIO_I2C_GPIO_SDATA, "gpio_i2c4_sdata"))
		printk("%s: gpio_request(gpio_i2ce_sdata) failed\n", __FUNCTION__);

	printk("%s: VAUX1 must be on to access product_id data.\n", __FUNCTION__);
}

/* Restore SCLK/SDA pins connected to the product ID back to I2C mode */
static void gpio_i2c_restore_pins(void)
{
	gpio_free(GPIO_I2C_GPIO_SCLK);
	gpio_free(GPIO_I2C_GPIO_SDATA);
	omap_mux_init_signal("i2c3_scl", OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("i2c3_sda", OMAP_PIN_INPUT_PULLUP);
}

static void gpio_i2c_config_pin(GPIO_I2C_PIN pin, GPIO_I2C_DIRECTION dir, int level)
{
	if (dir == GPIO_I2C_INPUT) {
		if (pin == GPIO_I2C_SCLK)
			gpio_direction_input(GPIO_I2C_GPIO_SCLK);
		else
			gpio_direction_input(GPIO_I2C_GPIO_SDATA);
	} else if (dir == GPIO_I2C_OUTPUT) {
		if (pin == GPIO_I2C_SCLK)
			gpio_direction_output(GPIO_I2C_GPIO_SCLK, level);
		else
			gpio_direction_output(GPIO_I2C_GPIO_SDATA, level);
	}
}

static int gpio_i2c_read_pin(GPIO_I2C_PIN pin)
{
	if (pin == GPIO_I2C_SCLK)
		return gpio_get_value(GPIO_I2C_GPIO_SCLK);
	else
		return gpio_get_value(GPIO_I2C_GPIO_SDATA);
	return 0;
}

static void gpio_i2c_set_pin_level(GPIO_I2C_PIN pin, int level)
{
	uint8_t pin_level;

	if (pin == GPIO_I2C_SCLK) {
		pin_level = gpio_get_value(GPIO_I2C_GPIO_SCLK);
		if (((level == 1) && (pin_level == 0)) ||
		    ((level == 0) && (pin_level == 1)))
			gpio_set_value(GPIO_I2C_GPIO_SCLK, level);
	} else if (pin == GPIO_I2C_SDATA) {
		if (level == 0) {
			gpio_i2c_config_pin(pin, GPIO_I2C_OUTPUT, 0);
		} else {
			gpio_i2c_config_pin(pin, GPIO_I2C_INPUT, 0);
		}
	}
}


static void gpio_i2c_init(int bps)
{
	gpio_i2c_bus_state = GPIO_I2C_UNINIT;

	gpio_i2c_config_pins();

	/* Config SCLK, SDATA pins */
	gpio_i2c_config_pin(GPIO_I2C_SCLK, GPIO_I2C_OUTPUT, 1);

	gpio_i2c_config_pin(GPIO_I2C_SDATA, GPIO_I2C_INPUT, 0);

	/* Assume 1:1 clock duty cycle */
	gpio_i2c_clock_high_width = gpio_i2c_clock_low_width
	  = 1000000 / bps / 2;

	gpio_i2c_coarse_delay = gpio_i2c_clock_high_width;
}

static int gpio_i2c_busy(void)
{
	return (gpio_i2c_bus_state == GPIO_I2C_STARTED);
}

static void gpio_i2c_tx_stop(void)
{
	if (gpio_i2c_bus_state == GPIO_I2C_STARTED) {
		udelay(gpio_i2c_coarse_delay);

		/* Pull SDATA low */
		gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 0);
		udelay(gpio_i2c_coarse_delay);

		/* Push SCLK high */
		gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
		udelay(gpio_i2c_coarse_delay);

		/* Now drive SDATA high - thats a STOP */
		gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 1);
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_bus_state = GPIO_I2C_STOPPED;
	}
}

static void gpio_i2c_tx_start(void)
{
	if (gpio_i2c_bus_state == GPIO_I2C_UNINIT || gpio_i2c_bus_state == GPIO_I2C_STOPPED) {
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 1);
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 0);
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 0);
		udelay(gpio_i2c_coarse_delay);
		gpio_i2c_bus_state = GPIO_I2C_STARTED;
	}
}

/* Return !0 if NACK */
static int gpio_i2c_tx_byte(uint8_t data)
{
	uint8_t clock, tx_bit_mask=0x80, nack;

	if (gpio_i2c_bus_state != GPIO_I2C_STARTED)
		printk("%s: Unexpected I2C bus state!\n", __FUNCTION__);

	udelay(gpio_i2c_coarse_delay);

	for (clock=0; clock <= 7; ++clock) {
		if (data & tx_bit_mask)
			gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 1);
		else
			gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 0);
		udelay(gpio_i2c_clock_low_width);
		gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
		udelay(gpio_i2c_clock_high_width);
		gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 0);
		tx_bit_mask >>= 1;
	}
	gpio_i2c_config_pin(GPIO_I2C_SDATA, GPIO_I2C_INPUT, 0);
	udelay(gpio_i2c_clock_low_width);
	gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
	udelay(gpio_i2c_clock_high_width);
	nack = gpio_i2c_read_pin(GPIO_I2C_SDATA);
	gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 0);
	return (nack != 0);
}

static int gpio_i2c_rx_byte(uint8_t *data, int rx_mode)
{
	uint8_t clock, data_bit;

	*data = 0;

	gpio_i2c_config_pin(GPIO_I2C_SDATA, GPIO_I2C_INPUT, 0);

	udelay(gpio_i2c_coarse_delay);

	for (clock=0; clock<=8; ++clock) {
		if (clock < 8) {
			gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
			udelay(gpio_i2c_clock_high_width);
			data_bit = gpio_i2c_read_pin(GPIO_I2C_SDATA);
			gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 0);
			*data = (*data << 1) | data_bit;
		} else {
			if ((rx_mode == RX_MODE_LAST_BYTE) || (rx_mode == RX_MODE_ONE_BYTE))
				gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 1);
			else
				gpio_i2c_set_pin_level(GPIO_I2C_SDATA, 0);

			gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 1);
			udelay(gpio_i2c_clock_high_width);
			gpio_i2c_set_pin_level(GPIO_I2C_SCLK, 0);
		}
		udelay(gpio_i2c_clock_low_width);
	}

	return 0;
}


static int send_packet(uint8_t *data, int len, uint8_t *rxbuf, int rxlen)
{
	int timeout = 1000;
	int retry;
	int rx_mode;
	int tick, err, idx;

	if (DEBUG_PRODUCTION_DATA) {
		char buf[128];
		int i, offset;
		for (offset = 0, i=0; i<len; ++i) {
			if (!i)
				offset = sprintf(buf, "%02x", data[i]);
			else
				offset += sprintf(&buf[offset], " %02x", data[i]);
		}
		printk("%s: %s\n", __FUNCTION__, buf);
	}

	/* Wait for bus */
	while (gpio_i2c_busy() && timeout--)
		udelay(100);

	if (!timeout)
		printk("%s i2c_busy never return zero!\n", __FUNCTION__);

	retry = 0;
	do {
		tick = 0;
		do {
			gpio_i2c_tx_stop();
			gpio_i2c_tx_start();

			/* send cmd */
			err = gpio_i2c_tx_byte(data[0]);
			tick++;
		} while (err && tick < 100);

		if (tick > 3)
			printk("I2C ACK polling tick %d!\n", tick);

		for (idx = 1; idx<len; ++idx) {
			err = gpio_i2c_tx_byte(data[idx]);
			if (err) {
				printk("%s: NACK idx %d\n", __FUNCTION__, idx);
			}
		}
	} while (err && (retry++ < 5));

	if (err)
		return err;

	/* Are we expecting a response? */
	if (rxbuf) {
		for (idx = 0; idx < rxlen; ++idx) {
			if (rxlen == 1)
				rx_mode = RX_MODE_ONE_BYTE;
			else if (idx == (rxlen - 1))
				rx_mode = RX_MODE_LAST_BYTE;
			else if (idx == (rxlen - 2))
				rx_mode = RX_MODE_NEXT_TO_LAST_BYTE;
			else if (idx == 0)
				rx_mode = RX_MODE_FIRST_BYTE;
			else
				rx_mode = RX_MODE_MIDDLE_BYTE;

			err = gpio_i2c_rx_byte(&rxbuf[idx], rx_mode);
			if (DEBUG_PRODUCTION_DATA) {
				if (err)
					printk("%s err idx %d\n", __FUNCTION__, idx);
			}
		}
	}

	gpio_i2c_tx_stop();
	return err;
}

#define CMD_SYSTEM_WRITE    0xB4

static int
set_user_zone(int zone)
{
	unsigned char cmd[] = { CMD_SYSTEM_WRITE, 0x03, 0x00, 0x00 };
	if (DEBUG_PRODUCTION_DATA)
		printk("%s: zone %d\n", __FUNCTION__, zone);
	cmd[2] = zone;
	return send_packet(cmd, sizeof(cmd), NULL, 0);
}

#define CMD_READ_USER_ZONE  0xB2

static int
read_user_zone(unsigned char *buf, int len, int startoff)
{
	unsigned char cmd[] = { CMD_READ_USER_ZONE, 0x00, 0x00, 0x00 };
	int ret;
	cmd[2] = startoff;
	cmd[3] = len;
	ret = send_packet(cmd, sizeof(cmd), buf, len);

	if (DEBUG_PRODUCTION_DATA_BUF) {
		char obuf[128];
		int i,j,offset;
		for (i = 0, offset=0; i<len; i+=16) {
			for (j = 0; j<16 && i+j<len; ++j)
				if (!j)
					offset = sprintf(obuf, "%02x", buf[i+j]);
				else
					offset += sprintf(&obuf[offset], " %02x", buf[i+j]);
			printk("%s\n", obuf);
		}
	}
	return ret;
}

static struct product_id_data product_id_data;
static int valid_product_id_lan_ethaddr;  // !0 if LAN ethaddr is good
static int valid_product_id_wifi_ethaddr;  // !0 if LAN ethaddr is good

static int header_version = -1;

/* Set to non-zero if product_id_data is valid, either from being
   passed through the SRAM, or by extracting from the product_id part. */
static int product_id_data_valid;

static inline int omap3logic_is_product_data_valid(void)
{
	return product_id_data_valid;
}

static int omap3logic_extract_header_version(struct product_id_data *p, int *header_version)
{
	if (p->d.u_zone0.pz_0r0.header_version == LOGIC_HEADER_VERSION_0) {
		*header_version = p->d.u_zone0.pz_0r0.header_version;
		return 0;
	}

	if (p->d.u_zone0.pz_0r1.header_version == LOGIC_HEADER_VERSION_1) {
		*header_version = p->d.u_zone0.pz_0r1.header_version;
		return 0;
	}

	if (p->d.u_zone0.pz_0r2.header_version == LOGIC_HEADER_VERSION_2
		|| p->d.u_zone0.pz_0r2.header_version == LOGIC_HEADER_VERSION_3) {
		*header_version = p->d.u_zone0.pz_0r2.header_version;
		return 0;
	}

	*header_version = -1;
	return -1;
}

int omap3logic_extract_product_id_part_number(struct product_id_data *p, char *buf, int buflen)
{
	int size;

	buf[0] = '\0';
	if (p->d.u_zone0.pz_0r0.header_version == LOGIC_HEADER_VERSION_0) {
		size = sizeof(p->d.u_zone0.pz_0r0.part_number);
		if (buflen < sizeof(p->d.u_zone0.pz_0r0.part_number))
			size = buflen;
		strncpy(buf, p->d.u_zone0.pz_0r0.part_number, sizeof(p->d.u_zone0.pz_0r0.part_number));
		buf[sizeof(p->d.u_zone0.pz_0r0.part_number)] = '\0';
		return 0;
	}

	if (p->d.u_zone0.pz_0r0.header_version == LOGIC_HEADER_VERSION_1) {
		size = sizeof(p->d.u_zone0.pz_0r1.part_number);
		if (buflen < sizeof(p->d.u_zone0.pz_0r1.part_number))
			size = buflen;
		strncpy(buf, p->d.u_zone0.pz_0r1.part_number, sizeof(p->d.u_zone0.pz_0r1.part_number));
		buf[sizeof(p->d.u_zone0.pz_0r1.part_number)] = '\0';
		return 0;
	}

	return -1;
}

static void extract_part_number(void)
{
	char buf[32];
	int ret;
	u32 part_number;

	ret = omap3logic_extract_product_id_part_number(&product_id_data, buf, sizeof(buf));
	if (ret)
		return;

	part_number = simple_strtoul(buf, NULL, 0);

	extracted_product_id_data.part_number_valid = 1;
	extracted_product_id_data.part_number = part_number;
}

int omap3logic_get_product_id_part_number(u32 *part_number)
{
	if (!extracted_product_id_data.part_number_valid)
		return -EINVAL;

	*part_number = extracted_product_id_data.part_number;
	return 0;
}

int omap3logic_extract_serial_number(struct product_id_data *p, char *buf, int buflen)
{
	buf[0] = '\0';

	if (!omap3logic_is_product_data_valid())
		return -1;

	if (header_version == LOGIC_HEADER_VERSION_0) {
		sprintf(buf, "%02d%02d%c%05d", p->d.u_zone0.pz_0r0.sn_week,
			 p->d.u_zone0.pz_0r0.sn_year, p->d.u_zone0.pz_0r0.sn_site,
			 p->d.u_zone0.pz_0r0.sn_cnt);
		return 0;
	}
	if (header_version == LOGIC_HEADER_VERSION_1) {
		sprintf(buf, "%02d%02d%c%05d", p->d.u_zone0.pz_0r1.sn_week,
			 p->d.u_zone0.pz_0r1.sn_year, p->d.u_zone0.pz_0r1.sn_site,
			 p->d.u_zone0.pz_0r1.sn_cnt);
		return 0;
	}
	if (header_version == LOGIC_HEADER_VERSION_2
		|| header_version == LOGIC_HEADER_VERSION_3) {
		sprintf(buf, "%02d%02d%c%05d", p->d.u_zone0.pz_0r2.sn_week,
			 p->d.u_zone0.pz_0r2.sn_year, p->d.u_zone0.pz_0r2.sn_site,
			 p->d.u_zone0.pz_0r2.sn_cnt);
		return 0;
	}
	return -1;
}

void omap3logic_extract_model_number_revision(struct product_id_data *p, char *buf, int buflen)
{
	int i;

	strncpy(buf, product_id_data.d.zone1.model_number, buflen);
	buf[buflen-1] = '\0';
	i = strlen(buf);
	if (i + 3 < buflen) {
		buf[i] = '-';
		buf[i+1] = product_id_data.d.zone1.model_revision;
		buf[i+2] = '\0';
	}
}


/* Only calculate across the data to checksum, compare to stored
   value(outside of checksummed range) */
static unsigned int calculate_product_id_checksum(void *p, int len)
{
	unsigned char *buf = p;
	unsigned int xsum = 0;
	int i;

	for (i=0; i<len; ++i)
		xsum = ((xsum << 3) || (xsum >> 29)) ^ buf[i];

	return xsum;
}

/* return !0 if three-byte address is valid */
static int valid_mac_address(unsigned char mac[3])
{
	if (mac[0] == 0xff && mac[1] == 0xff && mac[2] == 0xff)
		return 0;
	if (mac[0] == 0x00 && mac[1] == 0x00 && mac[2] == 0x00)
		return 0;
	return !0;
}

static int valid_full_mac_address(unsigned char mac[6])
{
	return (valid_mac_address(&mac[0]) && valid_mac_address(&mac[3]));
}

static int extract_product_id_mac_address(struct product_id_data *p, int position, unsigned char mac[6])
{
	unsigned char *m = NULL;

	if (!omap3logic_is_product_data_valid())
		return -1;

	switch(position) {
	case 0:
		if (header_version >= 2) {
			if (valid_full_mac_address(p->d.u_zone0.pz_0r2.full_mac)) {
				memcpy(mac, p->d.u_zone0.pz_0r2.full_mac, 6);
				return 0;
			}
		}
		m = p->d.zone2.pz_2r0.mac0;
		break;
	case 1:
		m = p->d.zone2.pz_2r0.mac1;
		break;
	case 2:
		m = p->d.zone2.pz_2r0.mac2;
		break;
	case 3:
		m = p->d.zone2.pz_2r0.mac3;
		break;
	}
	if (valid_mac_address(m)) {
		mac[0] = 0x00;
		mac[1] = 0x08;
		mac[2] = 0xee;
		memcpy(&mac[3], m, 3);
		return 0;
	}
	return -1;
}

/* Extract the ethaddr, and return !0 if its valid */
int omap3logic_extract_lan_ethaddr(u8 *ethaddr)
{
	int ret;
	ret = extract_product_id_mac_address(&product_id_data, 0, ethaddr);
	return !ret;
}
EXPORT_SYMBOL(omap3logic_extract_lan_ethaddr);

/* Extract the WiFi ethaddr, and return !0 if its valid */
int omap3logic_extract_wifi_ethaddr(u8 *ethaddr)
{
	int ret;
	ret = extract_product_id_mac_address(&product_id_data, 1, ethaddr);
	return !ret;
}
EXPORT_SYMBOL(omap3logic_extract_wifi_ethaddr);

/* Data is valid, extract/print it.  Validate ethaddrs for LAN/WiFi */
void valid_data_extract_dump(struct product_id_data *p)
{
	char buf[36];
	unsigned char mac[6];
	int ret;

	/* Code path only gets here if production data is valid */
	product_id_data_valid = 1;

	omap3logic_extract_product_id_part_number(p, buf, sizeof(buf));
	printk(KERN_INFO "Part Number  : %s\n", buf);
	omap3logic_extract_model_number_revision(p, buf, sizeof(buf));
	printk(KERN_INFO "Model Name   : %s\n", buf);
	omap3logic_extract_serial_number(p, buf, sizeof(buf));
	printk(KERN_INFO "Serial Number: %s\n", buf);

	ret = extract_product_id_mac_address(p, 0, mac);
	if (!ret) {
		printk(KERN_INFO "LAN ethaddr  : %02x:%02x:%02x:%02x:%02x:%02x\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}

	ret = extract_product_id_mac_address(p, 1, mac);
	if (!ret) {
		printk(KERN_INFO "WLAN ethaddr  : %02x:%02x:%02x:%02x:%02x:%02x\n",
			mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}

}

/* Check the product data passed in from u-boot at the start of SRAM. */
void omap3logic_fetch_sram_product_id_data(void)
{
	struct product_id_data *p;
	unsigned int checksum;

	/* U-boot passed in the production information at the start of
	   SRAM - should get LoLo to do the same... */
	
	p = sram_get_base_va();
	product_id_data = *p;

	/* Checksum the data, is it good? */

	checksum = calculate_product_id_checksum(&product_id_data.d, sizeof(product_id_data.d));

	if (checksum != product_id_data.checksum) {
		printk(KERN_INFO "U-boot provided product_id data is invalid\n");
		return;
	}

	/* If the header doesn't match, we can't map any of the data */
	if (omap3logic_extract_header_version(&product_id_data, &header_version)) {
		printk(KERN_ERR "U-boot provided product_id data has invalid header version %d!\n", header_version);
		return;
	}

	printk(KERN_INFO "U-boot provided product_id data is valid\n");

	valid_data_extract_dump(&product_id_data);

	/* If the data is not valid, then later fetch_product_data()
	 * is invoked(after the twl4030 comes up) and it will fetch
	 * the data for us. */
}


/* if the data isn't already in SRAM, then read the data out of the chip */
static int fetch_product_id_data(void)
{
	int err = 0;
	unsigned int checksum;
	
	/* If we already have the data, then punt... */
	if (omap3logic_is_product_data_valid())
	  return 0;

	printk(KERN_INFO "Extract product_id data from ID chip\n");

	printk(KERN_WARNING "%s: VAUX1 must be on by now to access product id data\n", __FUNCTION__);

	gpio_i2c_init(50000);

	printk("Read production data: ");

	if (set_user_zone(0) || read_user_zone((unsigned char *)&product_id_data.d.u_zone0, sizeof(product_id_data.d.u_zone0), 0)) {
		printk(KERN_ERR "failed!\n");
		err = -1;
		goto out;
	}

	if (set_user_zone(1) || read_user_zone((unsigned char *)&product_id_data.d.zone1, sizeof(product_id_data.d.zone1), 0)) {
		printk(KERN_ERR "failed!\n");
		err = -3;
		goto out;
	}

	if (set_user_zone(2) || read_user_zone((unsigned char *)&product_id_data.d.zone2, sizeof(product_id_data.d.zone2), 0)) {
		printk(KERN_ERR"failed!\n");
		err = -4;
		goto out;
	}

	printk("done\n");

	/* Checksum the data, is it good? */

	checksum = calculate_product_id_checksum(&product_id_data.d, sizeof(product_id_data.d));

	if (checksum != product_id_data.checksum) {
		printk(KERN_INFO "Production Data is invalid\n");
		err = -2;
		goto out;
	}

	/* If the header doesn't match, we can't map any of the data */
	if (omap3logic_extract_header_version(&product_id_data, &header_version)) {
		err = -2;
		printk(KERN_ERR "Production Data has invalid header version %d!\n", header_version);
		goto out;
	}

	printk(KERN_INFO "Production Data is valid\n");

	valid_data_extract_dump(&product_id_data);

	/* Correct endianess issues */
	if (header_version < LOGIC_HEADER_VERSION_2) {
		product_id_data.d.zone2.pz_2r0.processor_type = le16_to_cpu(product_id_data.d.zone2.pz_2r0.processor_type);
		product_id_data.d.zone2.pz_2r0.platform_bits = le32_to_cpu(product_id_data.d.zone2.pz_2r0.platform_bits);
	} else if (header_version == LOGIC_HEADER_VERSION_2) {
		product_id_data.d.zone2.pz_2r3.processor_type = le16_to_cpu(product_id_data.d.zone2.pz_2r2.processor_type);
		product_id_data.d.zone2.pz_2r2.platform_bits = le32_to_cpu(product_id_data.d.zone2.pz_2r2.platform_bits);
	} else {
		/* For version three and up */
		product_id_data.d.zone2.pz_2r3.processor_type = le16_to_cpu(product_id_data.d.zone2.pz_2r3.processor_type);
		product_id_data.d.zone2.pz_2r3.platform_bits = le32_to_cpu(product_id_data.d.zone2.pz_2r3.platform_bits);
	}

	/* Feature bits went away in verison 2 */
	if (header_version < LOGIC_HEADER_VERSION_2)
		product_id_data.d.zone2.pz_2r0.feature_bits = le32_to_cpu(product_id_data.d.zone2.pz_2r0.feature_bits);


 out:

	/* Restore pins back to their intended use */
	gpio_i2c_restore_pins();


	return err;
}

ssize_t product_id_show_lan_macaddr(struct class *class, char *buf)
{
	return sprintf(buf, "00:08:0e:%02x:%02x:%02x\n",
		       product_id_data.d.zone2.pz_2r0.mac0[0],
		       product_id_data.d.zone2.pz_2r0.mac0[1],
		       product_id_data.d.zone2.pz_2r0.mac0[2]);
}

ssize_t product_id_show_wifi_macaddr(struct class *class, char *buf)
{
	return sprintf(buf, "00:08:0e:%02x:%02x:%02x\n",
		       product_id_data.d.zone2.pz_2r0.mac1[0],
		       product_id_data.d.zone2.pz_2r0.mac1[1],
		       product_id_data.d.zone2.pz_2r0.mac1[2]);
}

ssize_t product_id_show_part_number(struct class *class, char *buf)
{
	int len;

	omap3logic_extract_product_id_part_number(&product_id_data, buf, 128);
	len = strlen(buf);
	buf[len++] = '\n';
	return len;
}

ssize_t product_id_show_model_name(struct class *class, char *buf)
{
	int len;

	omap3logic_extract_model_number_revision(&product_id_data, buf, 128);
	len = strlen(buf);
	buf[len++] = '\n';
	return len;
}

ssize_t product_id_show_serial_number(struct class *class, char *buf)
{
	int len;

	omap3logic_extract_serial_number(&product_id_data, buf, 128);
	len = strlen(buf);
	buf[len++] = '\n';
	return len;
}

ssize_t product_id_show_zone2_data(struct class *class, char *buf)
{
	int len = 0;
	struct product_id_data *p = &product_id_data;

	if (!omap3logic_is_product_data_valid()) {
		len = sprintf(buf, "Product ID data is invalid\n");
		return len;
	}

	switch(header_version) {
	case LOGIC_HEADER_VERSION_0:
	case LOGIC_HEADER_VERSION_1:
		len = sprintf(buf, "nor: %02x%02x nand: %02x%02x sdram: %02x%02x\n"
			"processor: %04x features: %08x platform_bits: %08x\n",
			p->d.zone2.pz_2r0.nor0_size, p->d.zone2.pz_2r0.nor1_size,
			p->d.zone2.pz_2r0.nand0_size, p->d.zone2.pz_2r0.nand1_size,
			p->d.zone2.pz_2r0.sdram0_size, p->d.zone2.pz_2r0.sdram1_size,
			p->d.zone2.pz_2r0.processor_type, p->d.zone2.pz_2r0.feature_bits,
			p->d.zone2.pz_2r0.platform_bits);
		break;
		break;
	case LOGIC_HEADER_VERSION_2:
		len = sprintf(buf, "nor: %02x%02x nand: %02x%02x sdram: %02x%02x\n"
			"processor: %04x platform_bits: %08x\n",
			p->d.zone2.pz_2r2.nor0_size, p->d.zone2.pz_2r2.nor1_size,
			p->d.zone2.pz_2r2.nand0_size, p->d.zone2.pz_2r2.nand1_size,
			p->d.zone2.pz_2r2.sdram0_size, p->d.zone2.pz_2r2.sdram1_size,
			p->d.zone2.pz_2r2.processor_type,
			p->d.zone2.pz_2r2.platform_bits);
		break;
	case LOGIC_HEADER_VERSION_3:
	default:
		len = sprintf(buf, "nor: %02x%02x nand: %02x%02x sdram: %02x%02x\n"
			"processor: %04x platform_bits: %08x hardware_revision %08x\n",
			p->d.zone2.pz_2r3.nor0_size, p->d.zone2.pz_2r3.nor1_size,
			p->d.zone2.pz_2r3.nand0_size, p->d.zone2.pz_2r3.nand1_size,
			p->d.zone2.pz_2r3.sdram0_size, p->d.zone2.pz_2r3.sdram1_size,
			p->d.zone2.pz_2r3.processor_type,
			p->d.zone2.pz_2r3.platform_bits, p->d.zone2.pz_2r3.hardware_revision);
		break;
	}
	return len;
}

ssize_t product_id_show_wifi_config_data(struct class *class, char *buf)
{
	int len;

	len = sizeof(product_id_data.d.wifi_config_data);
	memcpy(buf, product_id_data.d.wifi_config_data, len);
	return len;
}


#define DECLARE_CLASS_ATTR(_name,_mode,_show,_store)                  \
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode },	\
		.show   = _show,                                        \
		.store  = _store,				\
}
static struct {
	struct class_attribute attr;
	int *test_value;
} product_id_class_attributes[] = {
	{
		__ATTR(lan_macaddr, S_IRUGO, product_id_show_lan_macaddr, NULL),
		&valid_product_id_lan_ethaddr,
	},
	{
		__ATTR(wifi_macaddr, S_IRUGO, product_id_show_wifi_macaddr, NULL),
		&valid_product_id_wifi_ethaddr,
	},
	{
		__ATTR(part_number, S_IRUGO, product_id_show_part_number, NULL),
		NULL,
	},
	{
		__ATTR(model_name, S_IRUGO, product_id_show_model_name, NULL),
		NULL,
	},
	{
		__ATTR(serial_number, S_IRUGO, product_id_show_serial_number, NULL),
		NULL,
	},
	{
		__ATTR(zone2_data, S_IRUGO, product_id_show_zone2_data, NULL),
		NULL,
	},
	{
		__ATTR(wifi_config_data, S_IRUGO, product_id_show_wifi_config_data, NULL),
		NULL,
	},
};

static void product_id_dev_release(struct device *dev)
{
}

struct class product_id_class = {
	.name = "product_id",
	.dev_release = product_id_dev_release,
};

static int create_sysfs_files(void)
{
	int i, rc;

	rc = class_register(&product_id_class);
	if (rc != 0) {
		printk("%s: failed to register product_id class\n", __FUNCTION__);
		return rc;
	}

	for (i=0; i<ARRAY_SIZE(product_id_class_attributes); ++i) {
		if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
			rc = class_create_file(&product_id_class, &product_id_class_attributes[i].attr);
			if (unlikely(rc)) {
				printk("%s: failed to create product_id class file\n", __FUNCTION__);
				while (--i >= 0) { 
					if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
						class_remove_file(&product_id_class, &product_id_class_attributes[i].attr);
					}
				}
				class_unregister(&product_id_class);
				return -EPERM;
			}
		}
	}

	return 0;
}

#if 0
static void remove_sysfs_files(void)
{
	int i;
	for (i=0; i<ARRAY_SIZE(product_id_class_attributes); ++i) {
		if (!product_id_class_attributes[i].test_value || *product_id_class_attributes[i].test_value) {
			class_remove_file(&product_id_class, &product_id_class_attributes[i].attr);
		}
	}
	class_unregister(&product_id_class);

}
#endif

/* Starting with the "B" rev (1011880) an external pair of FET's
 * controlled by GPIO177. */
int omap3logic_has_external_mute(void)
{
	if (product_id_data.d.zone1.model_revision >= 'B')
		return !0;

	/* If we're a torpedo, we have mute as well */
	if (machine_is_omap3_torpedo())
		return !0;

	return 0;
}

/* Return positive non-zero if productID indicates there's
 * the first NOR flash on the device - return size of flash
 * as log2 in bytes, or negative for none(or size unknown) */
int omap3logic_NOR0_size(void)
{
	char nor0_size;

	if (!omap3logic_is_product_data_valid())
		return -1;

	if (header_version <= LOGIC_HEADER_VERSION_1) {
		nor0_size = product_id_data.d.zone2.pz_2r0.nor0_size;
	} else if (header_version <= LOGIC_HEADER_VERSION_2) {
		nor0_size = product_id_data.d.zone2.pz_2r2.nor0_size;
	} else 
		nor0_size = product_id_data.d.zone2.pz_2r3.nor0_size;

	/* Flash exists if its size is non-zero, but 0xff is known to be
	 * a non-programmed value */
	if (nor0_size == 0x00
		|| nor0_size == 0xff)
		return 0;

	return nor0_size;
}

static int __init omap3logic_fetch_production_data(void)
{
	int valid;
	extern void omap3logic_init_productid_specifics(void);


	valid = !fetch_product_id_data();
	if (valid) {
		create_sysfs_files();
	}
	/* Extract the product_id part_number */
	extract_part_number();

	/* Now that we *know* we have the product_id data, we can
	 * configure those bits that are specific to certain
	 * revisions of the board. */
	omap3logic_init_productid_specifics();

	return valid;
}
subsys_initcall_sync(omap3logic_fetch_production_data);
