/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef LINUX_BMA150_MODULE_H
#define LINUX_BMA150_MODULE_H

#define BMA150_I2C_NAME "bma150"
#ifdef CONFIG_SPI_QSD
#define BMA150_G_SENSOR_NAME "bma150_uP_spi"
#else
#define BMA150_G_SENSOR_NAME "bma150"
#endif

#define BMAIO				0xA1

#define SMB150_STATUS_REG	0x09
#define SMB150_CTRL_REG		0x0a
#define SMB150_CONF1_REG	0x0b
#define RANGE_BWIDTH_REG	0x14
#define SMB150_CONF2_REG	0x15

/* IOCTLs*/
#define BMA_IOCTL_INIT                  _IO(BMAIO, 0x31)
#define BMA_IOCTL_WRITE                 _IOW(BMAIO, 0x32, char[5])
#define BMA_IOCTL_READ                  _IOWR(BMAIO, 0x33, char[5])
#define BMA_IOCTL_READ_ACCELERATION    _IOWR(BMAIO, 0x34, short[7])
#define BMA_IOCTL_SET_MODE	  _IOW(BMAIO, 0x35, short)
#define BMA_IOCTL_GET_INT	  _IOR(BMAIO, 0x36, short)
#define BMA_IOCTL_GET_CHIP_LAYOUT	_IOR(BMAIO, 0x37, short)

/* mode settings */
#define BMA_MODE_NORMAL   	0
#define BMA_MODE_SLEEP       	1

/**
 * struct bma150_platform_data - data to set up bma150 driver
 *
 * @setup: optional callback to activate the driver.
 * @teardown: optional callback to invalidate the driver.
 *
**/

struct bma150_platform_data {
	int (*setup)(struct device *);
	void (*teardown)(struct device *);
	int (*power_on)(void);
	void (*power_off)(void);
};

#endif /* LINUX_BMA150_MODULE_H */
