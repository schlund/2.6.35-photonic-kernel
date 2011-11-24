/* arch/arm/mach-msm/htc_fb_console.c
 *
 * By Octavian Voicu <octavian@voicu.gmail.com>
 *
 * Fixed for HTC HD Mini by cardsharing-x 2011
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


/*
 * Ugly hack to get some basic console working for HTC Diamond but should
 * also work for Raphael and other similar phones. Will register itself
 * as an early console to show boot messages. This won't work unless
 * the LCD is already powered up and functional (from wince, boot
 * using Haret). We just DMA pixels to the LCD, all configuration
 * must already be done.
 *
 * Not exactly very clean nor optimized, but it's a one night hack
 * and it works :)
 *
 * If anyone makes any progress on HTC Diamond drop me a line
 * (see email at the top), I do wanna see Android on my HTC Diamond!
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/font.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mach/msm_iomap.h>
#include <mach/msm_fb.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>
#include "../../../drivers/video/msm/mdp_hw.h"
#include <asm/mach-types.h>

// Hack
//#define CONFIG_HTC_FB_CONSOLE_BOOT 	1
#define CONFIG_HTC_FB_CONSOLE_DELAY 	1

/* Defined in /arch/arm/mm/mmu.c, helps us map physical memory to virtual memory.
 * It should work pretty early in the boot process, that why we use it */
extern void create_mapping(struct map_desc *md);

/* Defined in include/linux/fb.h. When, on write, a registered framebuffer device is detected,
 * we immediately unregister ourselves. */
#ifndef CONFIG_HTC_FB_CONSOLE_BOOT
extern int num_registered_fb;
#endif

/* Red message (color = 1), the reset color to white (color = 7) */
#define HTC_FB_MSG		("\n\n" "\x1b" "1" "HTC HD Mini Linux framebuffer console" "\x1b" "7" "\n\n")

struct fb_para {
	uint32_t htc_fb_lcd_width;
	uint32_t htc_fb_lcd_height;
	uint32_t htc_fb_phys;
	uint32_t htc_fb_off;
	uint32_t htc_fb_size;
	uint32_t htc_fb_con_max_rows;
	uint32_t htc_fb_con_max_cols;
} para;


struct fb_para photon_para = {
 	.htc_fb_lcd_width 	= 320,
	.htc_fb_lcd_height 	= 480,
	.htc_fb_phys 	 	= 0x25800000,
	.htc_fb_off 	 	= 0x00000000,
	.htc_fb_size 	 	= 0x00200000,
	.htc_fb_con_max_rows 	= 80,
	.htc_fb_con_max_cols 	= 120,
};

struct fb_para default_para = {
 	.htc_fb_lcd_width 	= 480, 		/* LCD resolution width*/
	.htc_fb_lcd_height 	= 640, 		/* LCD resolution height*/
	.htc_fb_phys 	 	= 0x16800000, 	/* physical page for our fb */
	.htc_fb_off 	 	= 0x0006a000, 	/* offset in the page to start of fb */
	.htc_fb_size 	 	= 0x00100000, 	/* map 1 MB (640 * 480 * 2 rounded properly) */
	.htc_fb_con_max_rows 	= 120,		/* Set max console size for a 4x4 font */
	.htc_fb_con_max_cols 	= 160,		/* Set max console size for a 4x4 font */
};

/* Framebuffer stuff */
//after splhood base!
#define HTC_FB_BASE		0xF9400000 /* virtual page for our fb */
/* Pack color data in 565 RGB format; r and b are 5 bits, g is 6 bits */
#define HTC_FB_RGB(r, g, b) 	((((r) & 0x1f) << 11) | (((g) & 0x3f) << 5) | (((b) & 0x1f) << 0))

/* Some standard colors */
unsigned short htc_fb_colors[8] = {
	HTC_FB_RGB(0x00, 0x00, 0x00), /* Black */
	HTC_FB_RGB(0x1f, 0x00, 0x00), /* Red */
	HTC_FB_RGB(0x00, 0x15, 0x00), /* Green */
	HTC_FB_RGB(0x0f, 0x15, 0x00), /* Brown */
	HTC_FB_RGB(0x00, 0x00, 0x1f), /* Blue */
	HTC_FB_RGB(0x1f, 0x00, 0x1f), /* Magenta */
	HTC_FB_RGB(0x00, 0x3f, 0x1f), /* Cyan */
	HTC_FB_RGB(0x1f, 0x3f, 0x1f)  /* White */
};

/* We can use any font which has width <= 8 pixels */
const struct font_desc *htc_fb_default_font;

/* Pointer to font data (255 * font_rows bytes of data)  */
const unsigned char *htc_fb_font_data;

/* Size of font in pixels */
unsigned int htc_fb_font_cols, htc_fb_font_rows;

/* Size of console in chars */
unsigned int htc_fb_console_cols, htc_fb_console_rows;

/* Current position of cursor (where next character will be written) */
unsigned int htc_fb_cur_x, htc_fb_cur_y;

/* Current fg / bg colors */
unsigned char htc_fb_cur_fg, htc_fb_cur_bg;

/* Buffer to hold characters and attributes, set it to the biggest size! */
unsigned char htc_fb_chars[80][120];
unsigned char htc_fb_fg[80][120];
unsigned char htc_fb_bg[80][120];

static void htc_fb_console_write(struct console *console, const char *s, unsigned int count);

/* Console data */
static struct console htc_fb_console = {
	.name	= "htc_fb",
	.write	= htc_fb_console_write,
	.flags	=
#ifdef CONFIG_HTC_FB_CONSOLE_BOOT
		CON_BOOT |
#endif
		CON_PRINTBUFFER | CON_ENABLED,
	.index	= -1,
};

/* Update the framebuffer from the character buffer then start DMA */
static void htc_fb_console_update(void)
{
	unsigned int memaddr, fbram, stride, width, height, x, y, i, j, r1, c1, r2, c2;
	unsigned int dma2_cfg;
	unsigned short ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */
	unsigned short *ptr;
	unsigned char ch;

	fbram = para.htc_fb_phys + para.htc_fb_off;
	memaddr = HTC_FB_BASE + para.htc_fb_off;

	ptr = (unsigned short*) memaddr;
	for (i = 0; i < htc_fb_console_rows * htc_fb_font_rows; i++) {
		r1 = i / htc_fb_font_rows;
		r2 = i % htc_fb_font_rows;
		for (j = 0; j < htc_fb_console_cols * htc_fb_font_cols; j++) {
			c1 = j / htc_fb_font_cols;
			c2 = j % htc_fb_font_cols;
			ch = htc_fb_chars[r1][c1];
			*ptr++ = htc_fb_font_data[(((int) ch) * htc_fb_font_rows) + r2] & ((1 << (htc_fb_font_cols - 1)) >> c2)
				? htc_fb_colors[htc_fb_fg[r1][c1]]
				: htc_fb_colors[htc_fb_bg[r1][c1]];
		}
		ptr += para.htc_fb_lcd_width - htc_fb_console_cols * htc_fb_font_cols;
	}

	stride = para.htc_fb_lcd_width * 2;
	width = para.htc_fb_lcd_width;
	height = para.htc_fb_lcd_height;
	x = 0;
	y = 0;

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_PACK_PATTERN_RGB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= DMA_IBUF_FORMAT_RGB565;
	dma2_cfg |= DMA_OUT_SEL_MDDI;
	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;
	dma2_cfg |= DMA_DITHER_EN;

	/* setup size, address, and stride */
	writel((height << 16) | (width), MSM_MDP_BASE + 0x90004);
	writel(fbram, MSM_MDP_BASE + 0x90008);
	writel(stride, MSM_MDP_BASE + 0x9000c);        

	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

	/* set y & x offset and MDDI transaction parameters */
	writel((y << 16) | (x), MSM_MDP_BASE + 0x90010);
	writel(ld_param, MSM_MDP_BASE + 0x00094);
	writel((MDDI_VDO_PACKET_DESC_RGB565 << 16) | MDDI_VDO_PACKET_PRIM, MSM_MDP_BASE + 0x00094);

	writel(dma2_cfg, MSM_MDP_BASE + 0x90000);

	/* start DMA2 */
	writel(0, MSM_MDP_BASE + 0x00044);

	/* Wait a bit to let the transfer finish */
	mdelay(16);

//	*(volatile unsigned int*)(0xF8003000 + 0x808) |= 0x1000000;
}

/* Clear screen and buffers */
static void htc_fb_console_clear(void)
{
	/* Default white on black, clear everything */
	memset((void*) (HTC_FB_BASE + para.htc_fb_off), 0, para.htc_fb_lcd_width * para.htc_fb_lcd_height * 2);
	memset(htc_fb_chars, 0, htc_fb_console_cols * htc_fb_console_rows);
	memset(htc_fb_fg, 7, htc_fb_console_cols * htc_fb_console_rows);
	memset(htc_fb_bg, 0, htc_fb_console_cols * htc_fb_console_rows);
	htc_fb_cur_x = htc_fb_cur_y = 0;
	htc_fb_cur_fg = 7;
	htc_fb_cur_bg = 0;
	htc_fb_console_update();
}

static struct console htc_fb_console;

/* Write a string to character buffer; handles word wrapping, auto-scrolling, etc
 * After that, calls htc_fb_console_update to send data to the LCD */
static void htc_fb_console_write(struct console *console, const char *s, unsigned int count)
{
	unsigned int i, j, k, scroll;
	const char *p;

#ifndef CONFIG_HTC_FB_CONSOLE_BOOT
	// See if a framebuffer has been registered. If so, we disable this console to prevent conflict with
	// other FB devices (i.e. msm_fb).
	if (num_registered_fb > 0) {
		//*(volatile unsigned int*)(0xF8003000 + 0x808) |= 0x1000000;
		//printk(KERN_INFO "htc_fb_console: framebuffer device detected, disabling boot console\n");
		console->flags = 0;
		return;
	}
#endif

	scroll = 0;
	for (k = 0, p = s; k < count; k++, p++) {
		if (*p == '\n') {
			/* Jump to next line */
			scroll = 1;
		} else if (*p == '\t') {
			/* Tab size 8 chars */
			htc_fb_cur_x = (htc_fb_cur_x + 7) % 8;
			if (htc_fb_cur_x >= htc_fb_console_cols) {
				scroll = 1;
			}
		} else if (*p == '\x1b') {
			/* Escape char (ascii 27)
			 * Some primitive way to change color:
			 * \x1b followed by one digit to represent color (0 black ... 7 white) */
			if (k < count - 1) {
				p++;
				htc_fb_cur_fg = *p - '0';
				if (htc_fb_cur_fg >= 8) {
					htc_fb_cur_fg = 7;
				}
			}
		} else if (*p != '\r') {
			/* Ignore \r, other cars get written here */
			htc_fb_chars[htc_fb_cur_y][htc_fb_cur_x] = *p;
			htc_fb_fg[htc_fb_cur_y][htc_fb_cur_x] = htc_fb_cur_fg;
			htc_fb_bg[htc_fb_cur_y][htc_fb_cur_x] = htc_fb_cur_bg;
			htc_fb_cur_x++;
			if (htc_fb_cur_x >= htc_fb_console_cols) {
				scroll = 1;
			}
		}
		if (scroll) {
			scroll = 0;
			htc_fb_cur_x = 0;
			htc_fb_cur_y++;
			if (htc_fb_cur_y == htc_fb_console_rows) {
				/* Scroll below last line, shift all rows up
				 * Should have used a bigger buffer so no shift,
				 * would actually be needed -- but hey, it's a one night hack */
				htc_fb_cur_y--;
				for (i = 1; i < htc_fb_console_rows; i++) {
					for (j = 0; j < htc_fb_console_cols; j++) {
						htc_fb_chars[i - 1][j] = htc_fb_chars[i][j];
						htc_fb_fg[i - 1][j] = htc_fb_fg[i][j];
						htc_fb_bg[i - 1][j] = htc_fb_bg[i][j];
					}
				}
				for (j = 0; j < htc_fb_console_cols; j++) {
					htc_fb_chars[htc_fb_console_rows - 1][j] = 0;
					htc_fb_fg[htc_fb_console_rows - 1][j] = htc_fb_cur_fg;
					htc_fb_bg[htc_fb_console_rows - 1][j] = htc_fb_cur_bg;
				}
			}
		}
	}

	htc_fb_console_update();

#ifdef CONFIG_HTC_FB_CONSOLE_DELAY
	/* Delay so we can see what's there, we have no keys to scroll */
	mdelay(100);
#endif
}

static int already_inited;
/* Init console on LCD using MDDI/MDP interface to transfer pixel data.
 * We can DMA to the board, as long as we give a physical address to the LCD
 * controller and use the coresponding virtual address to write pixels to.
 * The physical address I used is the one wince had for the framebuffer */
int __init htc_fb_console_init(void)
{
	struct map_desc map;

	if (already_inited)
	{
		return 0;
	}
	already_inited = 1;

	switch(__machine_arch_type) {
		case MACH_TYPE_PHOTON:
			memcpy(&para, &photon_para, sizeof(para));
			break;
		default:
			memcpy(&para, &default_para, sizeof(para));
			break;
	}

	/* Map the framebuffer Windows was using, as we know the physical address */
	map.pfn = __phys_to_pfn(para.htc_fb_phys & SECTION_MASK);
	map.virtual = HTC_FB_BASE;
	map.length = ((unsigned long) para.htc_fb_size + ~SECTION_MASK) & SECTION_MASK;
	map.type = MT_MEMORY;
	/* Ugly hack, but we're not sure what works and what doesn't,
	 * so better use the lowest level we have for setting the mapping */
	create_mapping(&map);

	/* Init font (we support any font that has width <= 8; height doesn't matter) */
	htc_fb_default_font = get_default_font(para.htc_fb_lcd_width, para.htc_fb_lcd_height, 0xFF, 0xFFFFFFFF);
	if (!htc_fb_default_font) {
		printk(KERN_WARNING "Can't find a suitable font for htc_fb");
		return -1;
	}

	htc_fb_font_data = htc_fb_default_font->data;
	htc_fb_font_cols = htc_fb_default_font->width;
	htc_fb_font_rows = htc_fb_default_font->height;
	htc_fb_console_cols = para.htc_fb_lcd_width / htc_fb_font_cols;
	if (htc_fb_console_cols > para.htc_fb_con_max_cols)
		htc_fb_console_cols = para.htc_fb_con_max_cols;
	htc_fb_console_rows = para.htc_fb_lcd_height / htc_fb_font_rows;
	if (htc_fb_console_rows > para.htc_fb_con_max_rows)
		htc_fb_console_rows = para.htc_fb_con_max_rows;


	/* Clear the buffer; we could probably see the Haret output if we didn't clear
	 * the buffer (if it used same physical address) */
	htc_fb_console_clear();

	/* Welcome message */
	htc_fb_console_write(&htc_fb_console, HTC_FB_MSG, strlen(HTC_FB_MSG));

	/* Register console */
	register_console(&htc_fb_console);
	console_verbose();

	return 0;
}

console_initcall(htc_fb_console_init);

