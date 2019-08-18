/*
*  Raspberry-Pi Console Controller Driver
*
*  Copyright (c) 2017 Tomas Granlund
*
*  Based on the mk_arcade_joystick_rpi driver by Matthieu Proucelle
*/


/*
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
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
* MA 02110-1301, USA.
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/i2c.h>
#include <linux/ioport.h>
#include <asm/io.h>

#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/sched.h>
#include <linux/wait.h>


MODULE_AUTHOR("Tomas Granlund");
MODULE_DESCRIPTION("Raspberry-Pi Console Controller Driver");
MODULE_LICENSE("GPL");

#ifdef RPI2
#define PERI_BASE        0x3F000000
#else
#define PERI_BASE        0x20000000
#endif

#define GPIO_BASE                (PERI_BASE + 0x200000) /* GPIO controller */

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_READ(g)  *(gpio + 13) &= (1<<(g))

#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)

#define BSC1_BASE  (PERI_BASE + 0x804000)

/*
 * Teensy i2c gamepad defines
 */
#define TEENSY_READ_INPUT      0x10
#define TEENSY_BOUNCE_INTERVAL 0x20
#define TEENSY_I2C_CLOCKRATE   0x30
#define TEENSY_READ_VOLUME     0x40

 /*
 * Defines for I2C peripheral (aka BSC, or Broadcom Serial Controller)
 */

#define BSC1_C  *(bsc1 + 0x00)
#define BSC1_S  *(bsc1 + 0x01)
#define BSC1_DLEN *(bsc1 + 0x02)
#define BSC1_A  *(bsc1 + 0x03)
#define BSC1_FIFO *(bsc1 + 0x04)

#define BSC_C_I2CEN (1 << 15)
#define BSC_C_INTR (1 << 10)
#define BSC_C_INTT (1 << 9)
#define BSC_C_INTD (1 << 8)
#define BSC_C_ST (1 << 7)
#define BSC_C_CLEAR (1 << 4)
#define BSC_C_READ 1

#define START_READ BSC_C_I2CEN|BSC_C_ST|BSC_C_CLEAR|BSC_C_READ
#define START_WRITE BSC_C_I2CEN|BSC_C_ST

#define BSC_S_CLKT (1 << 9)
#define BSC_S_ERR (1 << 8)
#define BSC_S_RXF (1 << 7)
#define BSC_S_TXE (1 << 6)
#define BSC_S_RXD (1 << 5)
#define BSC_S_TXD (1 << 4)
#define BSC_S_RXR (1 << 3)
#define BSC_S_TXW (1 << 2)
#define BSC_S_DONE (1 << 1)
#define BSC_S_TA 1

#define CLEAR_STATUS BSC_S_CLKT|BSC_S_ERR|BSC_S_DONE

#define SPI0_BASE (PERI_BASE + 0x204000)

#define SPI0_CS 	*(spi0 + 0x00)
#define SPI0_FIFO	*(spi0 + 0x01)
#define SPI0_CLK	*(spi0 + 0x02)
#define SPI0_DLEN	*(spi0 + 0x03)
#define SPI0_LTOH	*(spi0 + 0x04)
#define SPI0_DC		*(spi0 + 0x05)

#define SPI0_CS_RXF			(1 << 20)
#define SPI0_CS_RXR 		(1 << 19)
#define SPI0_CS_TXD 		(1 << 18)
#define SPI0_CS_RXD			(1 << 17)
#define SPI0_CS_DONE 		(1 << 16)

#define SPI0_CS_INTR 		(1 << 10)
#define SPI0_CS_INTD 		(1 << 9)

#define SPI0_CS_TA			(1 << 7)
#define SPI0_CS_CSPOL		(1 << 6)
#define SPI0_CS_CLEAR_RX 	(1 << 5)
#define SPI0_CS_CLEAR_TX 	(1 << 4)
#define SPI0_CS_CPOL		(1 << 3)
#define SPI0_CS_CPHA		(1 << 2)
#define SPI0_CS_CHIP2		2 
#define SPI0_CS_CHIP1		1
#define SPI0_CS_CHIP0		0

// MCP3008 
#define MCP_START	(1 << 24)
#define MCP_SINGLE	(1 << 23)
#define MCP_CH(ch)	((ch % 8) << 20)

static volatile unsigned *gpio;
static volatile unsigned *bsc1;
static volatile unsigned *spi0;

struct pn_config {
	int args[2];
	unsigned int nargs;
};

static struct pn_config pn_cfg __initdata;

module_param_array_named(args, pn_cfg.args, int, &(pn_cfg.nargs), 0);
MODULE_PARM_DESC(args, "0: TPA2016D2 i2c address, 1: DS1050 i2c address");

#define PN_REFRESH_TIME HZ/100

#define PN_BUTTON_COUNT 12 //14
#define PN_MCP_CHANNELS 6

struct pn {
	struct input_dev* inpdev;
	struct timer_list timer;
	struct mutex mutex;
	int used;
	int tpa2016address;
	int ds1050address;
	int mcp_failed;
	unsigned char volume;
	unsigned char volume_dirty;
};

static struct pn *pn_base;

static const int pn_teensy_button_count = 16;
static const int pn_teensy_package_bytes = 25;
static const int pn_teensy_interrupt_gpio = 26;

// Teensy axes (4): L-Stick X, L-Stick Y, R-Stick X, R-Stick Y
//                  ABS_X,     ABS_Y,     ABS_RX,    ABS_RY

// Teensy buttons (16): A, B, X, Y, L, R, Start, Select, D-Pad Left, D-Pad Right, D-Pad Up, D-Pad Down, L-Trigger, R-Trigger, L-Stick press, R-Stick press
static const int pn_teensy_buttons[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, BTN_START, BTN_SELECT, BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_TL2, BTN_TR2, BTN_THUMBL, BTN_THUMBR
};

static const int pn_buttons[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, /*BTN_START, BTN_SELECT,*/ BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_THUMBL, BTN_THUMBR
};

static const int pn_gpio_map[] = {
	17, 23, 26, 27, 20, 24, /*18, 25,*/ 19, 6, 22, 13, 21, 4
};

// 
static const int pn_mcp_map[] = {
	2, 3, 4, 5, 6, 7
};

static void setGpioPullUpState(int gpioMask) {
	*(gpio + 37) = 0x02;
	udelay(10);
	*(gpio + 38) = gpioMask;
	udelay(10);
	*(gpio + 37) = 0x00;
	*(gpio + 38) = 0x00;
}

static int getPullUpMask(const int gpioMap[]) {
	int mask = 0x0000000;

	int i;
	for (i = 0; i < PN_BUTTON_COUNT; i++) {
		int pin_mask = 1 << gpioMap[i];
		mask = mask | pin_mask;
	}

	return mask;
}

static void i2c_init(void) {
	INP_GPIO(2);
	SET_GPIO_ALT(2, 0);
	INP_GPIO(3);
	SET_GPIO_ALT(3, 0);
}

static void spi_init(void) {
	INP_GPIO(8);
	INP_GPIO(9);
	INP_GPIO(10);
	INP_GPIO(11);
	
	SET_GPIO_ALT(8, 0);
	SET_GPIO_ALT(9, 0);
	SET_GPIO_ALT(10, 0);
	SET_GPIO_ALT(11, 0);
}

static void gpio_init(void) {
	int i;
	for (i = 0; i < PN_BUTTON_COUNT; i++) {
		INP_GPIO(pn_gpio_map[i]);
	}

	setGpioPullUpState(getPullUpMask(pn_gpio_map));
}

static void wait_i2c_done() {
	while ((!((BSC1_S)& BSC_S_DONE))) {
		udelay(10);
	}
}

static void pn_i2c_write(int dev_addr, char reg_addr, char *buf, unsigned short len) {
	int idx;

	BSC1_A = dev_addr;
	BSC1_DLEN = len + 1; // one byte for the register address, plus the buffer length
	BSC1_C = BSC_C_CLEAR;

	BSC1_FIFO = reg_addr; // start register address
	for (idx = 0; idx < len; idx++)
		BSC1_FIFO = buf[idx];

	BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
	BSC1_C = START_WRITE; // Start Write (see #define)

	wait_i2c_done();
}

static void pn_i2c_read(char dev_addr, char *buf, unsigned short len) {
	unsigned short bufidx;

	bufidx = 0;

	memset(buf, 0, len); // clear the buffer

	BSC1_DLEN = len;
	BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
	BSC1_C = START_READ; // Start Read after clearing FIFO (see #define)

	do {
		// Wait for some data to appear in the FIFO
		while ((BSC1_S & BSC_S_TA) && !(BSC1_S & BSC_S_RXD));

		// Consume the FIFO
		while ((BSC1_S & BSC_S_RXD) && (bufidx < len)) {
			buf[bufidx++] = BSC1_FIFO;
		}
	} while ((!(BSC1_S & BSC_S_DONE)));
}

static void pn_mcp_read(unsigned char *in_buf, int in_len, unsigned char *out_buf, int out_len) {
	int in_idx = 0;
	int out_idx = 0;
	int i;
	
	SPI0_CS |= SPI0_CS_CHIP0|SPI0_CS_CLEAR_RX|SPI0_CS_CLEAR_TX|SPI0_CS_CPHA;
	
	// start transfer
	SPI0_CS |= SPI0_CS_TA;
	
	do {
		while (in_idx < in_len && (SPI0_CS & SPI0_CS_TXD)) {
			SPI0_FIFO = in_buf[in_idx];
			in_idx++;
		}
		
		while (out_idx < out_len && (SPI0_CS & SPI0_CS_RXD)) {
			out_buf[out_idx] = SPI0_FIFO;
			out_idx++;
		}
		
	} while (!(SPI0_CS & SPI0_CS_DONE));
	
	// end transfer
	SPI0_CS &= ~(SPI0_CS_TA);
}

static unsigned short pn_mcp_read_channel(int channel) {
	const int len = 3;
	unsigned char in_buf[len];
	unsigned char out_buf[len];
	
	in_buf[0] = 1;
	in_buf[1] = 128 | ((channel % 8) << 4);
	in_buf[2] = 0;
		
	pn_mcp_read(in_buf, len, out_buf, len);
	
	return (out_buf[1] << 8) | out_buf[2];
}

static unsigned char pn_read_gpio(int btn) {
	int read = GPIO_READ(btn);
	if (read > 0) {
		return 0;
	} else {
		return 1;
	}

	return 0;
}

static void pn_log_buttons(unsigned char* btn_data, int btn_len) {
	int i;
	for (i = 0; i < btn_len; i++) {
		pr_err("gpio %i value %i.", pn_gpio_map[i], btn_data[i]);
	}
}

static void pn_read_packet(unsigned char *btn_data, unsigned short *mcp_data, int btn_len, int mcp_len) {
	int i;
	for (i = 0; i < mcp_len; i++) {
		mcp_data[i] = pn_mcp_read_channel(pn_mcp_map[i]);
	}
	
	for (i = 0; i < btn_len; i++) {
		btn_data[i] = pn_read_gpio(pn_gpio_map[i]);
	}
}

static void pn_input_report(struct input_dev* dev, unsigned short *mcp_data, unsigned char *btn_data) {
	int i;

	int lx = mcp_data[0];
	int ly = mcp_data[1];
	int rx = mcp_data[2];
	int ry = mcp_data[3];
	
	// send joystick data to input device
	input_report_abs(dev, ABS_X, lx);
	input_report_abs(dev, ABS_Y, ly);
	input_report_abs(dev, ABS_RX, rx);
	input_report_abs(dev, ABS_RY, ry);

	// send button data to input device
	for (i = 0; i < PN_BUTTON_COUNT; i++) {
		input_report_key(dev, pn_buttons[i], btn_data[i]);
	}

	input_sync(dev);
}

static void pn_set_volume(int dev_addr, unsigned char data) {
	int error = 0;
	unsigned char read_volume;

	pn_i2c_write(dev_addr, 0x05, NULL, 0);
	pn_i2c_read(dev_addr, &read_volume, 1);

	if (read_volume != data && !error) {
		pn_i2c_write(dev_addr, 0x05, &data, 1);
	}
}

static void pn_process_packet(struct pn* pn) {
	unsigned short mcp_data[PN_MCP_CHANNELS];
	unsigned char btn_data[PN_BUTTON_COUNT];
	
	pn_read_packet(btn_data, mcp_data, PN_BUTTON_COUNT, PN_MCP_CHANNELS);

	/*
	if (pn->mcp_failed == 0) {
		pn_log_buttons(btn_data, PN_BUTTON_COUNT);
		pn->mcp_failed = 1;
	}
	*/
	
	pn_input_report(pn->inpdev, mcp_data, btn_data);
	
	pn_set_volume(pn->tpa2016address, mcp_data[4]);
}

static void pn_timer(unsigned long private) {
	struct pn* pn = (void *) private;
	pn_process_packet(pn);
	mod_timer(&pn->timer, jiffies + PN_REFRESH_TIME);
}

static int pn_open(struct input_dev* dev) {
	struct pn* pn = input_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&pn->mutex);
	if (err)
		return err;

	if (!pn->used++)
		mod_timer(&pn->timer, jiffies + PN_REFRESH_TIME);

	mutex_unlock(&pn->mutex);
	return 0;
}

static void pn_close(struct input_dev* dev) {
	struct pn* pn = input_get_drvdata(dev);

	mutex_lock(&pn->mutex);
	if (!--pn->used) {
		del_timer_sync(&pn->timer);
	}
	mutex_unlock(&pn->mutex);
}

static int __init pn_setup(struct pn* pn) {
	struct input_dev *input_dev;
	int i;
	int err;
	int timeout;
	char phys[32];
	unsigned char tx = 0x00;
	
	pn->mcp_failed = 0;

	pn->inpdev = input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("Not enough memory for input device!\n");
		return -ENOMEM;
	}

	snprintf(phys, sizeof(phys),
		"input%d", 0);

	input_dev->name = "Pimension Controller";
	input_dev->phys = phys;
	input_dev->id.bustype = BUS_PARPORT;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 6;
	input_dev->id.version = 0x0100;

	input_set_drvdata(input_dev, pn);

	input_dev->open = pn_open;
	input_dev->close = pn_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	for (i = 0; i < 2; i++)
		input_set_abs_params(input_dev, ABS_X + i, 0, 1023, 16, 32);

	// setup right stick axes
	for (i = 0; i < 2; i++)
		input_set_abs_params(input_dev, ABS_RX + i, 0, 1023, 16, 32);

	// setup buttons
	for (i = 0; i < PN_BUTTON_COUNT; i++) {
		__set_bit(pn_buttons[i], input_dev->keybit);
	}

	// Configuring the amplifier
	tx = 0xC2;
	pn_i2c_write(pn->tpa2016address, 0x01, &tx, 1, &timeout);

	err = input_register_device(pn->inpdev);
	if (err)
		goto err_free_dev;
	
	SPI0_CLK = (1 << 6);

	return 0;

err_free_dev:
	input_free_device(pn->inpdev);
	pn->inpdev = NULL;
	return err;
}

static struct pn __init * pn_probe(int* addresses, int n_addresses) {
	struct pn* pn;
	int i;
	int count = 0;
	int err;

	pn = kzalloc(sizeof(struct pn), GFP_KERNEL);
	if (!pn) {
		pr_err("Not enough memory\n");
		err = -ENOMEM;
		goto err_out;
	}

	mutex_init(&pn->mutex);
	setup_timer(&pn->timer, pn_timer, (long)pn);

	pn->tpa2016address = addresses[0];
	pn->ds1050address = addresses[1];

	i2c_init();
	spi_init();
	gpio_init();

	err = pn_setup(pn);
	if (err)
		goto err_unreg_dev;

	return pn;

err_unreg_dev:
	input_unregister_device(pn->inpdev);
err_out:
	return ERR_PTR(err);
}

static void pn_remove(struct pn* pn) {

	input_unregister_device(pn->inpdev);

	kfree(pn);
}

static int __init pn_init(void) {
	/* Set up gpio pointer for direct register access */
	if ((gpio = ioremap(GPIO_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}
	/* Set up i2c pointer for direct register access */
	if ((bsc1 = ioremap(BSC1_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}
	if ((spi0 = ioremap(SPI0_BASE, 0xB0)) == NULL) {
		pr_err("io remap failed\n");
		return -EBUSY;
	}
	if (pn_cfg.nargs < 2) {
		pr_err("insufficient i2c addresses\n");
		return -EINVAL;
	}
	else {
		pn_base = pn_probe(pn_cfg.args, pn_cfg.nargs);
		if (IS_ERR(pn_base))
			return -ENODEV;
	}
	return 0;
}

static void __exit pn_exit(void) {
	if (pn_base)
		pn_remove(pn_base);

	iounmap(gpio);
	iounmap(bsc1);
	iounmap(spi0);
}

module_init(pn_init);
module_exit(pn_exit);
