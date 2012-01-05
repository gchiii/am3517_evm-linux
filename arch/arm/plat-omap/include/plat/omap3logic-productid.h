/*
 * (C) Copyright 2008
 * Logic Produc Development, <www.logicpd.com>
 * Peter Barada <peter.barada@logicpd.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef OMAP3LOGIC_PRODUCTID_H
#define OMAP3LOGIC_PRODUCTID_H

#define LOGIC_HEADER_VERSION_0 0
#define LOGIC_HEADER_VERSION_1 1
#define LOGIC_HEADER_VERSION_2 2
#define LOGIC_HEADER_VERSION_3 3

struct product_zone_0_rev_0 {
	unsigned char header_version;	// offset 0
	char part_number[11];		// offset 1
	char revision;			// offset 12
	char sn_week;			// offset 13
	char sn_year;			// offset 14
	char sn_site;			// offset 15
	int sn_cnt;			// offset 16
	char maturity;			// offset 20
};

struct product_zone_0_rev_1 {
	unsigned char header_version;	// offset 0
	char part_number[11];		// offset 1
	char reserved;			// revision removed after version 0
	char sn_week;			// offset 13
	char sn_year;			// offset 14
	char sn_site;			// offset 15
	int sn_cnt;			// offset 16
	char maturity;			// offset 20
	char wifi_trim;			// offset 21
};


struct product_zone_0_rev_2 {
	unsigned char header_version;	// offset 0
	char part_number[11];		// offset 1
	char revision;			// offset 12
	char sn_week;			// offset 13
	char sn_year;			// offset 14
	char sn_site;			// offset 15
	int sn_cnt;			// offset 16
	char maturity;			// offset 20
	char wifi_trim;			// offset 21
	unsigned char full_mac[6];	// offset 22
};

struct product_zone_2_rev_0 {
	unsigned char mac0[3];		// offset 0
	unsigned char mac1[3];		// offset 3
	unsigned char mac2[3];		// offset 6
	unsigned char mac3[3];		// offset 9
	unsigned char nor0_size;	// offset 12
	unsigned char nor1_size;	// offset 13
	unsigned char nand0_size;	// offset 14
	unsigned char nand1_size;	// offset 15
	unsigned char sdram0_size;	// offset 16
	unsigned char sdram1_size;	// offset 17
	short processor_type;		// offset 18
	int feature_bits;		// offset 20
	int platform_bits;		// offset 24
};

struct product_zone_2_rev_2 {
	unsigned char mac0[3];		// offset 0
	unsigned char mac1[3];		// offset 3
	unsigned char mac2[3];		// offset 6
	unsigned char mac3[3];		// offset 9
	unsigned char nor0_size;	// offset 12
	unsigned char nor1_size;	// offset 13
	unsigned char nand0_size;	// offset 14
	unsigned char nand1_size;	// offset 15
	unsigned char sdram0_size;	// offset 16
	unsigned char sdram1_size;	// offset 17
	short processor_type;		// offset 18
	char reserved[4];		// offset 20
	int platform_bits;		// offset 24
};

struct product_zone_2_rev_3 {
	unsigned char mac0[3];		// offset 0
	unsigned char mac1[3];		// offset 3
	unsigned char mac2[3];		// offset 6
	unsigned char mac3[3];		// offset 9
	unsigned char nor0_size;	// offset 12
	unsigned char nor1_size;	// offset 13
	unsigned char nand0_size;	// offset 14
	unsigned char nand1_size;	// offset 15
	unsigned char sdram0_size;	// offset 16
	unsigned char sdram1_size;	// offset 17
	short processor_type;		// offset 18
	char reserved[4];		// offset 20
	int platform_bits;		// offset 24
	int hardware_revision;		// offset 28
};


struct product_id_data {
	struct {
		union {
			struct product_zone_0_rev_0 pz_0r0;
			struct product_zone_0_rev_1 pz_0r1;
			struct product_zone_0_rev_2 pz_0r2;
		} u_zone0;

		struct {
			char model_revision;
			char model_number[31];
		} zone1;

		union {
			struct product_zone_2_rev_0 pz_2r0;
			struct product_zone_2_rev_2 pz_2r2;
			struct product_zone_2_rev_3 pz_2r3;
		} zone2;

		/* WiFi configuation data for the Murata module */
		unsigned char wifi_config_data[468];
	} d;
	unsigned int checksum;
};

extern int omap3logic_get_product_id_part_number(u32 *part_number);

extern int omap3logic_extract_lan_ethaddr(u8 *ethaddr);
extern int omap3logic_extract_wifi_ethaddr(u8 *ethaddr);
extern void omap3logic_fetch_sram_product_id_data(void);
int omap3logic_extract_product_id_part_number(struct product_id_data *p, char *buf, int buflen);
// extern int omap3logic_extract_header_version(struct product_id_data *p, int *header_version);
extern int omap3logic_extract_serial_number(struct product_id_data *p, char *buf, int buflen);
extern void omap3logic_extract_model_number_revision(struct product_id_data *p, char *buf, int buflen);

extern int omap3logic_has_external_mute(void);

// extern int omap3logic_is_product_data_valid(void);
extern void omap3logic_set_product_id_data(struct product_id_data *p);

extern int omap3logic_NOR0_size(void);

#endif
