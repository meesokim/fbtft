/*
 * FB driver for the ILI9481 LCD Controller
 *
 * Copyright (c) 2014 Petr Olivka
 * Copyright (c) 2013 Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <mach/platform.h>

#include "fbtft.h"

#define DRVNAME		"fb_kedei35"
#define WIDTH		320
#define HEIGHT		480

#define GPIOSET(no, ishigh)	{ if (ishigh) set|=(1<<no); else reset|=(1<<no); } while(0)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

	
#define  TS_START	0X80
#define  TS_X		0XD3
#define  TS_Y		0X93
#define  TS_Z1		0XB3
#define  TS_Z2		0XC3

#define SPI_CS			0x00
#define SPI_FIFO		0x04
#define SPI_CLK			0x08
#define SPI_DLEN		0x0c
#define SPI_LTOH		0x10
#define SPI_DC			0x14


#define SPI_CS_LEN_LONG		0x02000000
#define SPI_CS_DMA_LEN		0x01000000
#define SPI_CS_CSPOL2		0x00800000
#define SPI_CS_CSPOL1		0x00400000
#define SPI_CS_CSPOL0		0x00200000
#define SPI_CS_RXF		0x00100000
#define SPI_CS_RXR		0x00080000
#define SPI_CS_TXD		0x00040000
#define SPI_CS_RXD		0x00020000
#define SPI_CS_DONE		0x00010000
#define SPI_CS_LEN		0x00002000
#define SPI_CS_REN		0x00001000
#define SPI_CS_ADCS		0x00000800
#define SPI_CS_INTR		0x00000400
#define SPI_CS_INTD		0x00000200
#define SPI_CS_DMAEN		0x00000100
#define SPI_CS_TA		0x00000080
#define SPI_CS_CSPOL		0x00000040
#define SPI_CS_CLEAR_RX		0x00000020
#define SPI_CS_CLEAR_TX		0x00000010
#define SPI_CS_CPOL		0x00000008
#define SPI_CS_CPHA		0x00000004
#define SPI_CS_CS_10		0x00000002
#define SPI_CS_CS_01		0x00000001

#define SPI_CS_TXTX		0X000000B0

#define RESET		0X00
#define CMD_BE		0X11
#define CMD_AF		0X1B

#define DATE_BE		0X15
#define DATE_AF		0X1F

static void __iomem *lcd_spi_base;

static void spi_tx(unsigned char *spi_buff, int len)
{	
	writel(SPI_CS_TXTX|SPI_CS_CS_01,lcd_spi_base+SPI_CS);
	while(!(readl(lcd_spi_base+SPI_CS)&SPI_CS_TXD));
	writel(0x00,lcd_spi_base+SPI_FIFO);
	writel(*(spi_buff),lcd_spi_base+SPI_FIFO);
	writel(*(spi_buff+1),lcd_spi_base+SPI_FIFO);
	if (len > 2)
		writel((unsigned char)*(spi_buff+2),lcd_spi_base+SPI_FIFO);
	while(!(readl(lcd_spi_base+SPI_CS)&SPI_CS_DONE));	
}

#if 0
static int default_init_sequence[] = {

	/* SLP_OUT - Sleep out */
	-1, 0x11,
	-2, 50,
	/* Power setting */
	-1, 0xD0, 0x07, 0x42, 0x18,
	/* VCOM */
	-1, 0xD1, 0x00, 0x07, 0x10,
	/* Power setting for norm. mode */
	-1, 0xD2, 0x01, 0x02,
	/* Panel driving setting */
	-1, 0xC0, 0x10, 0x3B, 0x00, 0x02, 0x11,
	/* Frame rate & inv. */
	-1, 0xC5, 0x03,
	/* Pixel format */
	-1, 0x3A, 0x55,
	/* Gamma */
	-1, 0xC8, 0x00, 0x32, 0x36, 0x45, 0x06, 0x16,
		  0x37, 0x75, 0x77, 0x54, 0x0C, 0x00,
	/* DISP_ON */
	-1, 0x29,
	-3
};
#endif

void lcd_param(struct spi_device *spi, u8 data) {
	u8 db[] = {0,0x15};
	u8 da[] = {0,0x1f};
	db[0] = data;
	spi_write(spi, db, 2);
	spi_write(spi, da, 2);
}
void write_reg8_kedei(struct fbtft_par *par, int len, ...)
{
	va_list args;
	int i, ret, j=0;
	u8 *buf = (u8 *)par->buf;
//	u8 db[] = {0,0x15};
//	u8 da[] = {0,0x1f};
	u8 cb[] = {0,0x11};
	u8 ca[] = {0,0x1b};

	va_start(args, len);
	for (i = 0; i < len; i++) {
		buf[i] = (u8)va_arg(args, unsigned int);
	}
	va_end(args);

	if (unlikely(par->debug & DEBUG_WRITE_REGISTER)) {
		fbtft_par_dbg_hex(DEBUG_WRITE_REGISTER, par, par->info->device, u8, buf, len, "%s: ", __func__);
	}

	cb[0] = buf[j++];
	ret = spi_write(par->spi, cb, 2);
	ret += spi_write(par->spi, ca, 2);
	if (ret < 0) {
		dev_err(par->info->device, "%s: write() failed and returned %d\n", __func__, ret);
		return;
	}
	len--;

	if (len) {
		i = len;
		while (i--) 
//			db[0] = buf[j++];
			lcd_param(par->spi, buf[j++]);
//			ret = spi_write(par->spi, db, 2);
//			ret += spi_write(par->spi, da, 2);
//			if (ret < 0) {
//				dev_err(par->info->device, "%s: write() failed and returned %d\n", __func__, ret);
//				return;
	}
}

int write_vmem_kedei(struct fbtft_par *par, size_t offset, size_t len)
{
	u8 db[] = {0,0,0x15};
	u8 da[] = {0,0x1f};
	u8 *vmem8;
//	u16 *txbuf16 = (u16 *)par->txbuf.buf;
	size_t remain;
//	size_t to_copy;
//	size_t tx_array_size;
//	int i;
	int ret = 0;
//	size_t startbyte_size = 0;

	fbtft_par_dbg(DEBUG_WRITE_VMEM, par, "%s (offset=%zu, len=%zu):\n",
		__func__, offset, len);

	remain = len / 2;
	vmem8 = (u8 *)(par->info->screen_base + offset);
	while (remain--)
	{
		//*data = *vmem16;
		db[0] = *(vmem8+1);
		db[1] = *vmem8;
#if 0		
		spi_tx(db, 3);
		spi_tx(da, 2);
#else		
		spi_write(par->spi, db, 3);
		spi_write(par->spi, da, 2);		
#endif
		vmem8+=2;
	}
#if 0	
	/* non buffered write */
	if (!par->txbuf.buf)
		return spi_write(par->spi, vmem16, len);

	/* buffered write */
	tx_array_size = par->txbuf.len / 2;

	if (par->startbyte) {
		txbuf16 = (u16 *)(par->txbuf.buf + 1);
		tx_array_size -= 2;
		*(u8 *)(par->txbuf.buf) = par->startbyte | 0x2;
		startbyte_size = 1;
	}

	while (remain) {
		to_copy = remain > tx_array_size ? tx_array_size : remain;
		dev_dbg(par->info->device, "    to_copy=%zu, remain=%zu\n",
						to_copy, remain - to_copy);

		for (i = 0; i < to_copy; i++)
			txbuf16[i] = cpu_to_be16(vmem16[i]);

		vmem16 = vmem16 + to_copy;
		ret = spi_write(par->spi, par->txbuf.buf,
						startbyte_size + to_copy * 2);
		if (ret < 0)
			return ret;
		remain -= to_copy;
	}
#endif
	return ret;
}			

static int init_display(struct fbtft_par *par)
{
	int i = 0;
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);
	lcd_spi_base = ioremap(SPI0_BASE, SZ_256 - 1); 
	par->fbtftops.reset(par);
#if 0	
	if (par->gpio.cs != -1)
		gpio_set_value(par->gpio.cs, 0);  /* Activate chip */
	else
		gpio_set_value(8, 0);  /* Activate chip */
#endif
	/* startup sequence for Kedei LCD */
	mdelay(100);
	write_reg(par, 0x00); /* software reset */
	mdelay(1);
	write_reg(par, 0xb0, 0x00);
	write_reg(par, 0x11);
	mdelay(50);
	/* --------------------------------------------------------- */
    write_reg(par, 0xb3, 0x02, 0x00, 0x00, 0x00);
    write_reg(par, 0xc0, 0x10, 0x3b, 0x00, 0x02, 0x00, 0x01, 0x00, 0x43);
    write_reg(par, 0xc1, 0x08, 0x16, 0x08, 0x08);
    write_reg(par, 0xc4, 0x11, 0x07, 0x03, 0x03);
    write_reg(par, 0xc6, 0x00);
    write_reg(par, 0xc8, 0x03, 0x03, 0x13, 0x5c, 0x03, 0x07, 0x14, 0x08, 0x00, 0x21, 0x08, 0x14, 0x07, 0x53, 0x0c, 0x13, 0x03, 0x03, 0x21, 0x00);
    write_reg(par, 0x35, 0x00);
    write_reg(par, 0x36, 0x28);
    write_reg(par, 0x3a, 0x55);
    write_reg(par, 0x44, 0x00, 0x01);
	write_reg(par, 0xb6, 0x00, 0x02, 0x3b);
	write_reg(par, 0xd0, 0x07, 0x07, 0x1d);
	write_reg(par, 0xd1, 0x00, 0x03, 0x00);
	write_reg(par, 0xd2, 0x03, 0x14, 0x04);
	write_reg(par, 0xe0, 0x1f, 0x2c, 0x2c, 0x0b, 0x0c, 0x04, 0x4c, 0x64, 0x36, 0x03, 0x0e, 0x01, 0x10, 0x01, 0x00);
	write_reg(par, 0xe1, 0x1f, 0x3f, 0x3f, 0x0f, 0x1f, 0x0f, 0x7f, 0x32, 0x36, 0x04, 0x0b, 0x00, 0x19, 0x14, 0x0f);
	write_reg(par, 0xe2, 0x0f, 0x0f, 0x0f);
	write_reg(par, 0xe3, 0x0f, 0x0f, 0x0f);
	write_reg(par, 0x13);
	write_reg(par, 0x29);
	mdelay(20);
	write_reg(par, 0xb4, 0x00);
	mdelay(20); 
	write_reg(par, 0x2c);
	write_reg(par, 0x2a, 0x00, 0x00, 0x01, 0xdf);
	write_reg(par, 0x2b, 0x00, 0x00, 0x01, 0x3f);
	write_reg(par, 0x2c);	

	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
		"%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	/* column address */
	write_reg(par, 0x2a, xs >> 8, xs & 0xff, xe >> 8, xe & 0xff);

	/* Row address */
	write_reg(par, 0x2b, ys >> 8, ys & 0xff, ye >> 8, ye & 0xff);

	/* memory write */
	write_reg(par, 0x2c);
}

#define HFLIP 0x01
#define VFLIP 0x02
#define ROWxCOL 0x20
static int set_var(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	switch (par->info->var.rotate) {
	case 270:
		write_reg(par, 0x36, ROWxCOL | HFLIP | VFLIP | (par->bgr << 3));
		break;
	case 180:
		write_reg(par, 0x36, VFLIP | (par->bgr << 3));
		break;
	case 90:
		write_reg(par, 0x36, ROWxCOL | (par->bgr << 3));
		break;
	default:
		write_reg(par, 0x36, HFLIP | (par->bgr << 3));
		break;
	}

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.fbtftops = {
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.set_var = set_var,
		.write_register = write_reg8_kedei,
		.write_vmem = write_vmem_kedei,
	},
};
EXPORT_SYMBOL(write_reg8_kedei);
EXPORT_SYMBOL(write_vmem_kedei);
FBTFT_REGISTER_DRIVER(DRVNAME, "kedei,kedei35", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:kedei35");
MODULE_ALIAS("platform:kedei35");

MODULE_DESCRIPTION("FB driver for the KEDEI (ILI9341) LCD display controller");
MODULE_AUTHOR("Miso Kim");
MODULE_LICENSE("GPL");
