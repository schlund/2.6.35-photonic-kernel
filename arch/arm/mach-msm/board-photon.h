/* linux/arch/arm/mach-msm/board-photon.h
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_PHOTON_H
#define __ARCH_ARM_MACH_MSM_BOARD_PHOTON_H

#include <mach/board.h>

#define MSM_MEM1_BASE		0x00000000
#define MSM_MEM1_SIZE		0x0CC00000

#define MSM_MEM2_BASE		0x20000000
#define MSM_MEM2_SIZE		0x08000000

#define MSM_LINUX_BASE_OFFSET	0x00200000

#define MSM_MM_HEAP_SIZE        0x02A00000

#define MSM_PHOTON_LINUX1_BASE          (MSM_MEM1_BASE + MSM_LINUX_BASE_OFFSET)
#define MSM_PHOTON_LINUX1_SIZE          (MSM_MEM1_SIZE - MSM_LINUX_BASE_OFFSET)

#define MSM_PHOTON_LINUX2_BASE           MSM_MEM2_BASE
#define MSM_PHOTON_LINUX2_SIZE          (MSM_MEM2_SIZE - MSM_MM_HEAP_SIZE)


#define MSM_FB_BASE             MSM_MEM2_BASE + MSM_MEM2_SIZE - MSM_MM_HEAP_SIZE
#define MSM_FB_SIZE             0x00200000

#define MSM_GPU_MEM_BASE        MSM_FB_BASE + MSM_FB_SIZE
#define MSM_GPU_MEM_SIZE        0x00300000

#define MSM_PMEM_MDP_BASE       MSM_GPU_MEM_BASE + MSM_GPU_MEM_SIZE
#define MSM_PMEM_MDP_SIZE       0x01000000

#define MSM_PMEM_ADSP_BASE      MSM_PMEM_MDP_BASE + MSM_PMEM_MDP_SIZE
#define MSM_PMEM_ADSP_SIZE      0x00C1B000

#define MSM_PMEM_CAMERA_BASE    MSM_PMEM_ADSP_BASE + MSM_PMEM_ADSP_SIZE
#define MSM_PMEM_CAMERA_SIZE    0x00800000

/* TODO: To save space, we can move RAM_CONSOLE to 0x00000000 */
#define MSM_RAM_CONSOLE_BASE    0x00100000
#define MSM_RAM_CONSOLE_SIZE    SZ_1M

#define PHOTON_GPIO_TO_INT(x)           (x+64) /* from gpio_to_irq */

#define PHOTON_GPIO_UP_INT2_N           (18)
#define PHOTON_GPIO_USB_ID_PIN          (19)
#define PHOTON_POWER_KEY                (20)
#define PHOTON_GPIO_PS_HOLD             (25)
#define PHOTON_GPIO_CAM_I2C_SDA         (27)
#define PHOTON_GPIO_WIFI_IRQ1           (29)
#define PHOTON_GPIO_USB_ISET            (30)
#define PHOTON_GPIO_RESET_BTN_N         (36)
#define PHOTON_GPIO_SDMC_CD_N           (38)
#define PHOTON_GPIO_UP_INT_N            (39)
#define PHOTON_GPIO_CAM_I2C_SCL         (49)

/* Battery */
#define PHOTON_GPIO_CHARGE_EN_N			(23) //standard charger
#define PHOTON_FAST_CHARGER_DIS			(17) //disabled when AC cable is plugged
#define PHOTON_FAST_CHARGER_EN			(32) //enabled when AC cable is plugged

/* WLAN SD data */
#define PHOTON_GPIO_SD_D3               (51)
#define PHOTON_GPIO_SD_D2               (52)
#define PHOTON_GPIO_SD_D1               (53)
#define PHOTON_GPIO_SD_D0               (54)
#define PHOTON_GPIO_SD_CMD              (55)
#define PHOTON_GPIO_SD_CLK_0            (56)

/* I2C */
#define PHOTON_GPIO_I2C_SCL             (60)
#define PHOTON_GPIO_I2C_SDA             (61)

/* MicroSD */
#define PHOTON_GPIO_SDMC_CLK_0          (62)
#define PHOTON_GPIO_SDMC_CMD            (63)
#define PHOTON_GPIO_SDMC_D3             (64)
#define PHOTON_GPIO_SDMC_D2             (65)
#define PHOTON_GPIO_SDMC_D1             (66)
#define PHOTON_GPIO_SDMC_D0             (67)

/* BT PCM */
#define PHOTON_GPIO_AUD_PCM_DO          (68)
#define PHOTON_GPIO_AUD_PCM_DI          (69)
#define PHOTON_GPIO_AUD_PCM_SYNC        (70)
#define PHOTON_GPIO_AUD_PCM_CLK         (71)

#define PHOTON_GPIO_UP_RESET_N          (76)
#define PHOTON_GPIO_UART3_RX            (86)
#define PHOTON_GPIO_UART3_TX            (87)
#define PHOTON_GPIO_VIB_3V_EN           (92)
#define PHOTON_GPIO_LS_EN               (93)
#define PHOTON_GPIO_WIFI_EN             (108)
#define PHOTON_GPIO_USBPHY_3V3_EN       (109)

#define PHOTON_PROJECT_NAME		"photon"
#define PHOTON_LAYOUTS			{ \
		{ { -1,  0, 0}, {  0, -1, 0}, {0, 0, 1} },  \
		{ { 0, -1, 0}, { 1,  0, 0}, {0, 0, -1} },  \
		{ { 0, -1, 0}, {  1, 0, 0}, {0, 0, 1} },  \
		{ { -1,  0, 0}, {  0,  0, -1}, {0, 1,  0} }   \
					}

/* Proximity  */
#define PHOTON_GPIO_PROXIMITY_INT       (21)
#define PHOTON_GPIO_PROXIMITY_EN        (119)

/* Navi key output/input matrix */
#define PHOTON_GPIO_KP_MKOUT2           (33)
#define PHOTON_GPIO_KP_MKOUT1           (34)
#define PHOTON_GPIO_KP_MKOUT0           (35)
#define PHOTON_GPIO_KP_MKIN2            (40)
#define PHOTON_GPIO_KP_MKIN1            (41)
#define PHOTON_GPIO_KP_MKIN0            (42)

/* BT */
#define PHOTON_GPIO_BT_UART1_RTS        (43)
#define PHOTON_GPIO_BT_UART1_CTS        (44)
#define PHOTON_GPIO_BT_UART1_RX         (45)
#define PHOTON_GPIO_BT_UART1_TX         (46)
#define PHOTON_GPIO_BT_RESET_N          (90)
#define PHOTON_GPIO_BT_HOST_WAKE        (112)
#define PHOTON_GPIO_BT_CHIP_WAKE        (122)
#define PHOTON_GPIO_BT_SHUTDOWN_N       (123)

/* Touch Panel */
#define PHOTON_GPIO_TP_ATT_N            (94)
#define PHOTON_LCD_RSTz		 (118)

/* 35mm headset */
#define PHOTON_GPIO_35MM_HEADSET_DET    (83)
#define PHOTON_GPIO_HEADSET_AMP			(89)

/*Camera AF VCM POWER*/
#define PHOTON_GPIO_VCM_PWDN            (82)
#define PHOTON_GPIO_CAM1_RST_N          (121)

/*Display*/
#define PHOTON_GPIO_LCD_ID0             (57)
#define PHOTON_GPIO_LCD_ID1             (58)
#define PHOTON_GPIO_LCD_VSYNC           (97)
#define PHOTON_GPIO_LCD_RST_N           (118)

int __init photon_init_keypad(void);
int photon_init_mmc(unsigned int sys_rev);
int __init photon_init_panel(void);
int photon_is_nand_boot(void);
#endif /* GUARD */

