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
#define SPI0_CS_CHIP0		0
#define SPI0_CS_CHIP1		1
#define SPI0_CS_CHIP2		2

// MCP3008 
#define MCP_START	(1 << 4)
#define MCP_SINGLE	(1 << 3)
#define MCP_CH(ch)	((ch % 8)|MCP_SINGLE|MCP_START)

static volatile unsigned *gpio;
static volatile unsigned *bsc1;
static volatile unsigned *spi0;

struct pn_config {
	int args[2];
	unsigned int nargs;
};

static struct pn_config pn_cfg __initdata;

module_param_array_named(args, pn_cfg.args, int, &(pn_cfg.nargs), 0);
MODULE_PARM_DESC(args, "0: teensy i2c address, 1: TPA2016D2 i2c address");

#define PN_REFRESH_TIME HZ/100

struct pn {
	struct input_dev* inpdev;
	struct timer_list timer;
	struct mutex mutex;
	int used;
	int i2cAddresses[2];
	int mcp_failed;
};

static struct pn *pn_base;

static const int pn_teensy_button_count = 16;
static const int pn_teensy_package_bytes = 25;
static const int pn_teensy_interrupt_gpio = 26;
static const int pn_i2c_timeout_cycles = 5000;

// Teensy axes (4): L-Stick X, L-Stick Y, R-Stick X, R-Stick Y
//                  ABS_X,     ABS_Y,     ABS_RX,    ABS_RY

// Teensy buttons (16): A, B, X, Y, L, R, Start, Select, D-Pad Left, D-Pad Right, D-Pad Up, D-Pad Down, L-Trigger, R-Trigger, L-Stick press, R-Stick press
static const short pn_teensy_buttons[] = {
	BTN_A, BTN_B, BTN_X, BTN_Y, BTN_TL, BTN_TR, BTN_START, BTN_SELECT, BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_TL2, BTN_TR2, BTN_THUMBL, BTN_THUMBR
};

static void setGpioPullUpState(int state, int gpioMask) {
	*(gpio + 37) = state;
	udelay(10);
	*(gpio + 38) = gpioMask;
	udelay(10);
	*(gpio + 37) = 0x00;
	*(gpio + 38) = 0x00;
}

static void setGpioAsInput(int gpioNum) {
	INP_GPIO(gpioNum);
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

static void wait_i2c_done(int* timeout) {
	int timeout_counter = pn_i2c_timeout_cycles;
	while ((!((BSC1_S)& BSC_S_DONE)) && --timeout_counter) {
		udelay(10);
	}

	if (timeout_counter == 0) {
		*(timeout) = 1;
	}
}

static void pn_i2c_write(int dev_addr, char reg_addr, char *buf, unsigned short len, int* timeout) {
	int idx;

	BSC1_A = dev_addr;
	BSC1_DLEN = len + 1; // one byte for the register address, plus the buffer length
	BSC1_C = BSC_C_CLEAR;

	BSC1_FIFO = reg_addr; // start register address
	for (idx = 0; idx < len; idx++)
		BSC1_FIFO = buf[idx];

	BSC1_S = CLEAR_STATUS; // Reset status bits (see #define)
	BSC1_C = START_WRITE; // Start Write (see #define)

	wait_i2c_done(timeout);
}

static void pn_i2c_read(char dev_addr, char *buf, unsigned short len, int* error) {
	unsigned short bufidx;
	int timeout = 0;
	int interrupt = 0;

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

	if ((BSC1_S & BSC_S_CLKT)) {
		*(error) = 1;
	}
}

static int pn_mcp_read(int in) {
	int tempbuf;
	
	// configure
	int cs = SPI0_CS;
	cs |= SPI0_CS_CHIP0|SPI0_CS_CLEAR_RX|SPI0_CS_CLEAR_TX;
	SPI0_CS = cs;
		
	// start transfer
	cs = SPI0_CS;
	cs |= SPI0_CS_TA;
	SPI0_CS = cs;
	
	while (!(SPI0_CS & SPI0_CS_TXD));

	SPI0_FIFO = in;
	
	while (!(SPI0_CS & SPI0_CS_RXD));
		
	tempbuf = SPI0_FIFO;
	
	while (!(SPI0_CS & SPI0_CS_DONE));
	
	// end transfer
	cs = SPI0_CS;
	cs &= 0xffffffff ^ SPI0_CS_TA;
	SPI0_CS = cs;
	
	return tempbuf;
}

static void pn_teensy_read_packet(int i2cAddress, unsigned char *data, int* error) {
	int i;
	int i2c_read_error = 0, interrupt = 0, timeout = 0;
	int max_int_read_tries = 10;
	const int package_len = 11;

	/*
	* byte 0-1: L-Stick X
	* byte 2-3: L-Stick Y
	* byte 4-5: R-Stick X
	* byte 6-7: R-Stick Y
	* byte 8: A, B, X, Y, L, R, Select, Start
	* byte 9: L-Stick, R-Stick, D-Pad Left, D-Pad Right, D-Pad Up, D-Pad Down, Custom1, Custom2
	* byte 10: volume
	*/
	char result[package_len];

	pn_i2c_write(i2cAddress, TEENSY_READ_INPUT, NULL, 0, &timeout);

	if (!timeout) {
		do {
			interrupt = GPIO_READ(pn_teensy_interrupt_gpio);

			if (interrupt) {
				pn_i2c_read(i2cAddress, result, package_len, &i2c_read_error);
				break;
			}

			udelay(1000);
		} while (--max_int_read_tries);
	}

	if (i2c_read_error || timeout || !max_int_read_tries) {
		*(error) = 1;
	}
	else {

		// read the first 8 bytes as axes
		for (i = 0; i < 8; i++) {
			data[i] = result[i];
		}

		// read 8 buttons in the 9th byte
		for (i = 8; i < 16; i++) {
			data[i] = (result[8] << (i - 8)) & 0x80;
		}

		// read 8 buttons in the 10th byte
		for (i = 16; i < 24; i++) {
			data[i] = (result[9] << (i - 16)) & 0x80;
		}

		data[24] = result[10];
	}
}

static void pn_mcp_read_packet(unsigned char *data, int *error) {
	int ch;
	int buf;
	
	for (ch = 0; ch < 6; ch++) {
		printk("sending %d to MCP.\n", MCP_CH(ch));
		buf = pn_mcp_read(MCP_CH(ch));
		printk("channel %d: %d\n", ch, buf);
	}
}

static int pn_constrain_number(int num, int min, int max) {

  if (num < min) {
    num = min;
  }
  else if (num > max) {
    num = max;
  }

  return num;
}

static void pn_teensy_input_report(struct input_dev* dev, unsigned char * data) {
	int j;

	int lx = (data[1] << 8) | data[0];
	int ly = (data[3] << 8) | data[2];
	int rx = (data[5] << 8) | data[4];
	int ry = (data[7] << 8) | data[6];

        lx = pn_constrain_number(lx, 0, 1023);
        ly = pn_constrain_number(ly, 0, 1023);
        rx = pn_constrain_number(rx, 0, 1023);
        ry = pn_constrain_number(ry, 0, 1023);

	// send joystick data to input device
	input_report_abs(dev, ABS_X, lx);
	input_report_abs(dev, ABS_Y, ly);
	input_report_abs(dev, ABS_RX, rx);
	input_report_abs(dev, ABS_RY, ry);

	// send button data to input device
	for (j = 8; j < 24; j++) {
		input_report_key(dev, pn_teensy_buttons[j - 8], data[j]);
	}

	input_sync(dev);
}

static void pn_set_volume(int dev_addr, unsigned char data) {
	int error = 0;
	unsigned char read_volume;

	pn_i2c_write(dev_addr, 0x05, NULL, 0, &error);
	pn_i2c_read(dev_addr, &read_volume, 1, &error);

	if (read_volume != data && !error) {
		pn_i2c_write(dev_addr, 0x05, &data, 1, &error);
	}
}

static void pn_process_packet(struct pn* pn) {
	unsigned char data[pn_teensy_package_bytes];
	unsigned char mcp_test_data[12];
	int error = 0;

	pn_teensy_read_packet(pn->i2cAddresses[0], data, &error);
	if (!pn->mcp_failed) {
		pn_mcp_read_packet(mcp_test_data, &error);
		pn->mcp_failed = 1;
	}

	if (!error) {
		pn_teensy_input_report(pn->inpdev, data);
		pn_set_volume(pn->i2cAddresses[1], data[24]);
	}
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

static int __init pn_setup_teensy(struct pn* pn) {
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
	for (i = 0; i < pn_teensy_button_count; i++) {
		__set_bit(pn_teensy_buttons[i], input_dev->keybit);
	}

	setGpioAsInput(pn_teensy_interrupt_gpio);
	setGpioPullUpState(0x01, (1 << pn_teensy_interrupt_gpio));

	// Configuring the amplifier
	tx = 0xC2;
	pn_i2c_write(pn->i2cAddresses[1], 0x01, &tx, 1, &timeout);

	err = input_register_device(pn->inpdev);
	if (err)
		goto err_free_dev;

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

	pn->i2cAddresses[0] = addresses[0];
	pn->i2cAddresses[1] = addresses[1];

	i2c_init();
	spi_init();

	err = pn_setup_teensy(pn);
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
