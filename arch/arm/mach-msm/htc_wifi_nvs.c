/* arch/arm/mach-msm/htc_wifi_nvs.c
 *
 * Code to extract WiFi calibration information from ATAG set up 
 * by the bootloader.
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Dmitry Shmidt <dimitrysh@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <mach/msm_iomap.h>
#include <linux/crc32.h>
#include <linux/io.h>

#include "board-photon.h"

#define NVS_MAX_SIZE		0x800U
#define NVS_MACADDR_SIZE	0x1AU

static struct proc_dir_entry *wifi_calibration;
static unsigned char nvs_mac_addr[NVS_MACADDR_SIZE];
static unsigned char *hardcoded_nvs = 
	"sromrev=3\n"\
	"vendid=0x14e4\n"\
	"devid=0x432f\n"\
	"boardtype=0x4b9\n"\
	"boardrev=0x32\n"\
	"boardflags=0x200\n"\
	"xtalfreq=37400\n"\
	"aa2g=1\n"\
	"aa5g=0\n"\
	"ag0=255\n"\
	"pa0b0=5747\n"\
	"pa0b1=64128\n"\
	"pa0b2=65195\n"\
	"pa0itssit=62\n"\
	"pa0maxpwr=72\n"\
	"opo=20\n"\
	"mcs2gpo0=0x6666\n"\
	"mcs2gpo1=0x6666\n"\
	"rssismf2g=0xa\n"\
	"rssismc2g=0xb\n"\
	"rssisav2g=0x3\n"\
	"bxa2g=0\n"\
	"ccode=ALL\n"\
	"cctl=0x0\n"\
	"cckdigfilttype=0\n"\
	"ofdmdigfilttype=1\n"\
	"rxpo2g=2\n"\
	"boardnum=1\n"\
	"nocrc=1\n"\
	"otpimagesize=182\n"\
	"hwhdr=0x05ffff031030031003100000\n"\
	"sd_gpout=0\n"\
	"sd_gpval=1\n"\
	"sd_gpdc=0\n"\
	"sd_oobonly=1\n"\
	"RAW1=80 32 fe 21 02 0c 00 22 2a 01 01 00 00 c5 0 e6 00 00 00 00 00 40 00 00 ff ff 80 00 00 00 00 00 00 00 00 00 00 c8 00 00 00 00 00 00 00 00 00 00 00 00 00 ff 20 04 D0 2 29 43 21 02 0c 00 22 04 00 20 00 5A\n"\
	"sd_gpout=0\n";

unsigned char *get_wifi_nvs_ram( void )
{
	return hardcoded_nvs;
}
EXPORT_SYMBOL(get_wifi_nvs_ram);

static int parse_tag_msm_wifi(void)
{
	uint32_t id1, id2, id3, sid1, sid2, sid3;
	uint32_t id_base = 0xEF260;
	id1 = readl(MSM_SHARED_RAM_BASE + id_base + 0x0);
	id2 = readl(MSM_SHARED_RAM_BASE + id_base + 0x4);	
	id3 = readl(MSM_SHARED_RAM_BASE + id_base + 0x8);
	sid1 = crc32(~0, &id1, 4);
	sid2 = crc32(~0, &id2, 4);
	sid3 = crc32(~0, &id3, 4);
	sprintf(nvs_mac_addr, "macaddr=00:23:76:%2x:%2x:%2x\n", sid1 % 0xff, sid2 % 0xff, sid3 % 0xff);
	pr_info("[WIFI] Device Wifi Mac Address: %s\n", nvs_mac_addr);
	return 0;
}

static int parse_tag_msm_wifi_from_spl(void)
{
	int n;
	uint32_t id[10];
	uint32_t id_base = 0xFC028;
	for (n=0;n<6;n++)
		id[n] = 0xff & readl(MSM_SPLHOOD_BASE + id_base + n);
	sprintf(nvs_mac_addr, "macaddr=%2x:%2x:%2x:%2x:%2x:%2x\n", id[0], id[1], id[2], id[3], id[4], id[5]);
	pr_info("[WIFI] Device Real Wifi Mac Address: %s\n", nvs_mac_addr);
	return 0;
}

static unsigned wifi_get_nvs_size( void )
{
	unsigned len;

	len = strlen(hardcoded_nvs)+NVS_MACADDR_SIZE;
	return len;
}

static int wifi_calibration_read_proc(char *page, char **start, off_t off,
					int count, int *eof, void *data)
{
	unsigned char *ptr;
	unsigned len;
#ifdef NVS_MSM_WIFI_DEBUG
	unsigned i;
#endif

	memcpy(page, nvs_mac_addr, NVS_MACADDR_SIZE);
	ptr = get_wifi_nvs_ram();
	len = min(wifi_get_nvs_size(), (unsigned)count);
	memcpy(page+NVS_MACADDR_SIZE, ptr, strlen(hardcoded_nvs));
#ifdef NVS_MSM_WIFI_DEBUG
	printk("[WIFI] WiFi Data len = %d \n", len);
	for(i=0;( i < len );i++) {
		printk("%c", *page++);
	}
#endif	
	return len;
}

static int __init wifi_nvs_init(void)
{
	pr_info("[WIFI] %s\n", __func__);
	if (photon_is_nand_boot()) {
		parse_tag_msm_wifi();
	} else {
		parse_tag_msm_wifi_from_spl();
	}
	wifi_calibration = create_proc_entry("calibration", 0444, NULL);
	if (wifi_calibration != NULL) {
		wifi_calibration->size = wifi_get_nvs_size();
		wifi_calibration->read_proc = wifi_calibration_read_proc;
		wifi_calibration->write_proc = NULL;
	}
	return 0;
}

late_initcall(wifi_nvs_init);
