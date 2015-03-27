/* arch/arm/mach-msm/board-bravo.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 * Author: Dima Zavin <dima@android.com>
 * Copyright (C) 2010 Giulio Cervera <giulio.cervera@gmail.com>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
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

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/reboot.h>
#include <linux/msm_kgsl.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/usb/composite.h>
#include <linux/usb/android_composite.h>
//#include <linux/usb/f_accessory.h>
#include <linux/android_pmem.h>
#include <linux/ion.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/capella_cm3602_htc.h>
#include <linux/akm8973.h>
#include <linux/regulator/machine.h>
#include <linux/ds2784_battery.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/msm_iomap.h>
#include <mach/msm_serial_debugger.h>
#include <mach/system.h>
#include <mach/msm_serial_hs.h>
#ifdef CONFIG_SERIAL_BCM_BT_LPM
#include <mach/bcm_bt_lpm.h>
#endif

#include <mach/board.h>
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/socinfo.h>
#include <mach/msm_spi.h>
#include <mach/htc_usb.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_smd.h>
#include <mach/gpiomux.h>
#include <mach/msm_flashlight.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#include <mach/vreg.h>
#include <mach/board-bravo-microp-common.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
 
#include "acpuclock.h"

#include "board-bravo.h"
#include "devices.h"
#include "proc_comm.h"
#include "board-bravo-tpa2018d1.h"
#include "board-bravo-smb329.h"

#include "timer.h"
#include "acpuclock.h"
#include "pm.h"
#include "irq.h"
#include "dex_comm.h"
#include "pm-boot.h"
#include "footswitch.h"
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
#include <linux/curcial_oj.h>
#endif

static uint debug_uart;

module_param_named(debug_uart, debug_uart, uint, 0);

extern void notify_usb_connected(int);
extern void msm_init_pmic_vibrator(void);
extern void __init bravo_audio_init(void);

extern int microp_headset_has_mic(void);

///////////////////////////////////////////////////////////////////////
// KGSL (HW3D support)#include <linux/android_pmem.h>
///////////////////////////////////////////////////////////////////////

//* start kgsl */
static struct resource kgsl_3d0_resources[] = {
        {
                .name  = KGSL_3D0_REG_MEMORY,
                .start = 0xA0000000,
                .end = 0xA001ffff,
                .flags = IORESOURCE_MEM,
        },
        {
                .name = KGSL_3D0_IRQ,
                .start = INT_GRAPHICS,
                .end = INT_GRAPHICS,
                .flags = IORESOURCE_IRQ,
        },
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
        .pwrlevel = {
                {
                        .gpu_freq = 0,
                        .bus_freq = 128000000,
                },
        },
        .init_level = 0,
        .num_levels = 1,
        .set_grp_async = NULL,
        .idle_timeout = HZ/5,
        .clk_map = KGSL_CLK_CORE | KGSL_CLK_IFACE,
};

struct platform_device msm_kgsl_3d0 = {
        .name = "kgsl-3d0",
        .id = 0,
        .num_resources = ARRAY_SIZE(kgsl_3d0_resources),
        .resource = kgsl_3d0_resources,
        .dev = {
                .platform_data = &kgsl_3d0_pdata,
        },
};
/* end kgsl */

/* start footswitch regulator */
struct platform_device *msm_footswitch_devices[] = {
	FS_PCOM(FS_GFX3D,  "fs_gfx3d"),
};
unsigned msm_num_footswitch_devices = ARRAY_SIZE(msm_footswitch_devices);
/* end footswitch regulator */

///////////////////////////////////////////////////////////////////////
// Memory
///////////////////////////////////////////////////////////////////////

#define MSM_AUDIO_SIZE		0x80000

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION

static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#endif

/* pmem heaps */
#ifndef CONFIG_ION_MSM

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

#endif

/* ion heaps */
#ifdef CONFIG_ION_MSM
static struct ion_co_heap_pdata co_ion_pdata = {
        .adjacent_mem_id = INVALID_HEAP_ID,
        .align = PAGE_SIZE,
};

static struct ion_platform_data ion_pdata = {
        .nr = 2,
        .heaps = {
                {
                        .id        = ION_SYSTEM_HEAP_ID,
                        .type        = ION_HEAP_TYPE_SYSTEM,
                        .name        = ION_VMALLOC_HEAP_NAME,
                },
                /* PMEM_MDP = SF */
                {
                        .id        = ION_SF_HEAP_ID,
                        .type        = ION_HEAP_TYPE_CARVEOUT,
                        .name        = ION_SF_HEAP_NAME,
                        .base        = MSM_PMEM_MDP_BASE,
                        .size        = MSM_PMEM_MDP_SIZE,
                        .memory_type = ION_EBI_TYPE,
                        .extra_data = (void *)&co_ion_pdata,
                },
        }
};

static struct platform_device ion_dev = {
        .name = "ion-msm",
        .id = 1,
        .dev = { .platform_data = &ion_pdata },
};
#endif
/* end ion heaps */

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_venc_pdata = {
	.name = "pmem_venc",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

#ifndef CONFIG_ION_MSM
static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};
#endif

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

static struct platform_device android_pmem_venc_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_venc_pdata },
};

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_venc_size = MSM_PMEM_VENC_SIZE;
static int __init pmem_venc_size_setup(char *p)
{
	pmem_venc_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_venc_size", pmem_venc_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static struct memtype_reserve qsd8x50_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_device(struct android_pmem_platform_data *pdata, unsigned long start, unsigned long size)
{
  pdata->size = size;
  pr_info("%s: pmem %s requests %lu bytes dynamically.\n",
      __func__, pdata->name, size);
}

static void __init size_pmem_devices(void)
{
#ifndef CONFIG_ION_MSM
  size_pmem_device(&android_pmem_pdata, 0, pmem_mdp_size);
#endif
#ifdef CONFIG_ANDROID_PMEM
  size_pmem_device(&android_pmem_adsp_pdata, 0, pmem_adsp_size);
  size_pmem_device(&android_pmem_venc_pdata, 0, pmem_venc_size);
  qsd8x50_reserve_table[MEMTYPE_EBI1].size += PMEM_KERNEL_EBI1_SIZE;
#endif
}



static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
  pr_info("%s: reserve %lu bytes from memory %d for %s.\n", __func__, p->size, p->memory_type, p->name);
  qsd8x50_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
#endif
#ifndef CONFIG_ION_MSM
        reserve_memory_for(&android_pmem_pdata);
#endif
}


static void __init qsd8x50_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int qsd8x50_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct reserve_info qsd8x50_reserve_info __initdata = {
	.memtype_reserve_table = qsd8x50_reserve_table,
	.calculate_reserve_sizes = qsd8x50_calculate_reserve_sizes,
	.paddr_to_memtype = qsd8x50_paddr_to_memtype,
};

static void __init qsd8x50_reserve(void)
{
	reserve_info = &qsd8x50_reserve_info;
	msm_reserve();
}

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

///////////////////////////////////////////////////////////////////////
// Real Time Clock
///////////////////////////////////////////////////////////////////////

struct platform_device msm_device_rtc = {
	.name = "msm_rtc",
	.id = -1,
};

static struct platform_device bravo_rfkill = {
	.name = "bravo_rfkill",
	.id = -1,
};

static struct resource ram_console_resources[] = {
	{
		.start	= MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name		= "ram_console",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

static int bravo_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on) {
		/* level shifter should be off */
		gpio_set_value(BRAVO_GPIO_TP_EN, 1);
		msleep(120);
		/* enable touch panel level shift */
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 1);
		msleep(3);
	} else {
		gpio_set_value(BRAVO_GPIO_TP_LS_EN, 0);
		gpio_set_value(BRAVO_GPIO_TP_EN, 0);
		udelay(50);
	}

	return 0;
}

static struct synaptics_i2c_rmi_platform_data bravo_synaptics_ts_data[] = {
	{
		.version = 0x100,
		.power = bravo_ts_power,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 480,
		.inactive_right = -1 * 0x10000 / 480,
		.inactive_top = -5 * 0x10000 / 800,
		.inactive_bottom = -5 * 0x10000 / 800,
		.sensitivity_adjust = 12,
	}
};

static struct akm8973_platform_data compass_platform_data = {
	.layouts = BRAVO_LAYOUTS,
	.project_name = BRAVO_PROJECT_NAME,
	.reset = BRAVO_GPIO_COMPASS_RST_N,
	.intr = BRAVO_GPIO_COMPASS_INT_N,
};

static struct regulator_consumer_supply tps65023_dcdc1_supplies[] = {
	{
		.supply = "acpu_vcore",
	},
};

static struct regulator_init_data tps65023_data[5] = {
	{
		.constraints = {
			.name = "dcdc1", /* VREG_MSMC2_1V29 */
			.min_uV = BRAVO_MIN_UV_MV * 1000,
			.max_uV = BRAVO_MAX_UV_MV * 1000,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
		},
		.consumer_supplies = tps65023_dcdc1_supplies,
		.num_consumer_supplies = ARRAY_SIZE(tps65023_dcdc1_supplies),
	},
	/* dummy values for unused regulators to not crash driver: */
	{
		.constraints = {
			.name = "dcdc2", /* VREG_MSMC1_1V26 */
			.min_uV = 1260000,
			.max_uV = 1260000,
		},
	},
	{
		.constraints = {
			.name = "dcdc3", /* unused */
			.min_uV = 800000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "ldo1", /* unused */
			.min_uV = 1000000,
			.max_uV = 3150000,
		},
	},
	{
		.constraints = {
			.name = "ldo2", /* V_USBPHY_3V3 */
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
};

static void ds2482_set_slp_n(unsigned n)
{
	gpio_direction_output(BRAVO_GPIO_DS2482_SLP_N, n);
}

static int capella_cm3602_power(int pwr_device, uint8_t enable);
static struct microp_function_config microp_functions[] = {
	{
		.name = "light_sensor",
		.category = MICROP_FUNCTION_LSENSOR,
		.levels = { 0x000, 0x001, 0x00F, 0x01E, 0x03C, 0x121, 0x190, 0x2BA, 0x35C, 0x3FF },
		.channel = 6,
		.int_pin = IRQ_LSENSOR,
		.golden_adc = 0xC0,
		.ls_power = capella_cm3602_power,
	},
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_functions[0],
	.irq = MSM_uP_TO_INT(9),
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = BRAVO_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static struct tpa2018d1_platform_data tpa2018_data = {
	.gpio_tpa2018_spk_en = BRAVO_CDMA_GPIO_AUD_SPK_AMP_EN,
};

static struct i2c_board_info base_i2c_devices[] = {
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x40),
		.platform_data = bravo_synaptics_ts_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO("bravo-microp", 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO("ds2482", 0x30 >> 1),
		.platform_data = ds2482_set_slp_n,
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_COMPASS_INT_N),
	},
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.platform_data = tps65023_data,
	},
};

static struct i2c_board_info rev_CX_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tpa2018d1", 0x58),
		.platform_data = &tpa2018_data,
	},
	{
		I2C_BOARD_INFO("smb329", 0x6E >> 1),
	},
};

///////////////////////////////////////////////////////////////////////
// USB
///////////////////////////////////////////////////////////////////////

#define USB_LINK_RESET_TIMEOUT      (msecs_to_jiffies(10))
#define CLKRGM_APPS_RESET_USBH      37
#define CLKRGM_APPS_RESET_USB_PHY   34

#define ULPI_VERIFY_MAX_LOOP_COUNT  3
static void *usb_base;
#ifndef MSM_USB_BASE
#define MSM_USB_BASE              ((unsigned)usb_base)
#endif
static unsigned bravo_ulpi_read(void __iomem *usb_base, unsigned reg)
{
	unsigned timeout = 100000;

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_read: timeout %08x\n",
			readl(USB_ULPI_VIEWPORT));
		return 0xffffffff;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int bravo_ulpi_write(void __iomem *usb_base, unsigned val, unsigned reg)
{
	unsigned timeout = 10000;

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while ((readl(USB_ULPI_VIEWPORT) & ULPI_RUN) && (--timeout))
		cpu_relax();

	if (timeout == 0) {
		printk(KERN_ERR "ulpi_write: timeout\n");
		return -1;
	}

	return 0;
}

void msm_hsusb_apps_reset_link(int reset)
{
	int ret;
	unsigned usb_id = CLKRGM_APPS_RESET_USBH;

	if (reset)
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
				&usb_id, NULL);
	else
		ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
				&usb_id, NULL);
	if (ret)
		printk(KERN_INFO "%s: Cannot set reset to %d (%d)\n",
			__func__, reset, ret);
}
EXPORT_SYMBOL(msm_hsusb_apps_reset_link);

void msm_hsusb_apps_reset_phy(void)
{
	int ret;
	unsigned usb_phy_id = CLKRGM_APPS_RESET_USB_PHY;

	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_ASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
	msleep(1);
	ret = msm_proc_comm(PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
			&usb_phy_id, NULL);
	if (ret) {
		printk(KERN_INFO "%s: Cannot assert (%d)\n", __func__, ret);
		return;
	}
}
EXPORT_SYMBOL(msm_hsusb_apps_reset_phy);

static int msm_hsusb_phy_verify_access(void __iomem *usb_base)
{
	int temp;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		if (bravo_ulpi_read(usb_base, ULPI_DEBUG) != (unsigned)-1)
			break;
		msm_hsusb_apps_reset_phy();
	}

	if (temp == ULPI_VERIFY_MAX_LOOP_COUNT) {
		pr_err("%s: ulpi read failed for %d times\n",
				__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
		return -1;
	}

	return 0;
}

static unsigned msm_hsusb_ulpi_read_with_reset(void __iomem *usb_base, unsigned reg)
{
	int temp;
	unsigned res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = bravo_ulpi_read(usb_base, reg);
		if (res != -1)
			return res;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi read failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);

	return -1;
}

static int msm_hsusb_ulpi_write_with_reset(void __iomem *usb_base,
		unsigned val, unsigned reg)
{
	int temp;
	int res;

	for (temp = 0; temp < ULPI_VERIFY_MAX_LOOP_COUNT; temp++) {
		res = bravo_ulpi_write(usb_base, val, reg);
		if (!res)
			return 0;
		msm_hsusb_apps_reset_phy();
	}

	pr_err("%s: ulpi write failed for %d times\n",
			__func__, ULPI_VERIFY_MAX_LOOP_COUNT);
	return -1;
}

static int msm_hsusb_phy_caliberate(void __iomem *usb_base)
{
	int ret;
	unsigned res;

	ret = msm_hsusb_phy_verify_access(usb_base);
	if (ret)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_read_with_reset(usb_base, ULPI_FUNC_CTRL_CLR);
	if (res == -1)
		return -ETIMEDOUT;

	res = msm_hsusb_ulpi_write_with_reset(usb_base,
			res | ULPI_SUSPENDM,
			ULPI_FUNC_CTRL_CLR);
	if (res)
		return -ETIMEDOUT;

	msm_hsusb_apps_reset_phy();

	return msm_hsusb_phy_verify_access(usb_base);
}

void msm_hsusb_8x50_phy_reset(void)
{
	u32 temp;
	unsigned long timeout;
	int ret, usb_phy_error;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	usb_base = ioremap(MSM_HSUSB_PHYS, 4096);

	msm_hsusb_apps_reset_link(1);
	msm_hsusb_apps_reset_phy();
	msm_hsusb_apps_reset_link(0);

	/* select ULPI phy */
	temp = (readl(USB_PORTSC) & ~PORTSC_PTS);
	writel(temp | PORTSC_PTS_ULPI, USB_PORTSC);

	if ((ret = msm_hsusb_phy_caliberate(usb_base))) {
		usb_phy_error = 1;
		pr_err("msm_hsusb_phy_caliberate returned with %i\n", ret);
		return;
	}

	/* soft reset phy */
	writel(USBCMD_RESET, USB_USBCMD);
	timeout = jiffies + USB_LINK_RESET_TIMEOUT;
	while (readl(USB_USBCMD) & USBCMD_RESET) {
		if (time_after(jiffies, timeout)) {
			pr_err("usb link reset timeout\n");
			break;
		}
		msleep(1);
	}
	usb_phy_error = 0;

	return;
}

static int bravo_phy_init_seq[] ={0x0C, 0x31, 0x30, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1};

static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_reset		= msm_hsusb_8x50_phy_reset,
	.phy_init_seq		= bravo_phy_init_seq,
	.mode			= USB_PERIPHERAL,
	.otg_control		= OTG_PHY_CONTROL,
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "HTC",
	.product	= "Desire",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	.vendorID	= 0x0bb4,
	.vendorDescr	= "HTC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0bb4,
	.product_id	= 0x0c02,
	.version	= 0x0100,
	.product_name		= "Desire",
	.manufacturer_name	= "HTC",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.fserial_init_string = "tty:modem,tty:autobot,tty:serial,tty:autobot",
	.nluns = 1,
	.usb_id_pin_gpio = BRAVO_GPIO_USB_ID_PIN,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

void bravo_add_usb_devices(void)
{
	printk(KERN_INFO "%s rev: %d\n", __func__, system_rev);
	android_usb_pdata.products[0].product_id =
			android_usb_pdata.product_id;

	android_usb_pdata.serial_number = board_serialno();

	/* add cdrom support in normal mode */
	if (board_mfg_mode() == 0) {
		android_usb_pdata.nluns = 3;
		android_usb_pdata.cdrom_lun = 0x4;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	//msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_device_gadget_peripheral.dev.parent = &msm_device_otg.dev;
	//usb_gpio_init();
	platform_device_register(&msm_device_otg);
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

unsigned bravo_get_vbus_state(void)
{
	if(readl(MSM_SHARED_RAM_BASE+0xef20c))
		return 1;
	else
		return 0;
}

static int __capella_cm3602_power(int on)
{
	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");
	if (on) {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 0);
		gpio_direction_output(BRAVO_GPIO_PROXIMITY_EN, 1);
	} else {
		gpio_direction_output(BRAVO_GPIO_LS_EN_N, 1);
	}
	return 0;
};

static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	/* TODO eolsen Add Voltage reg control */
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
};

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = BRAVO_GPIO_PROXIMITY_EN,
	.p_out = BRAVO_GPIO_PROXIMITY_INT_N,
	.irq = MSM_GPIO_TO_INT(BRAVO_GPIO_PROXIMITY_INT_N),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};

///////////////////////////////////////////////////////////////////////
// Flashlight
///////////////////////////////////////////////////////////////////////

static uint32_t flashlight_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static uint32_t flashlight_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(BRAVO_CDMA_GPIO_FLASHLIGHT_TORCH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_FLASHLIGHT_FLASH, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

static int config_bravo_flashlight_gpios(void)
{
	if (is_cdma_version(system_rev)) {
		config_gpio_table(flashlight_gpio_table_rev_CX, ARRAY_SIZE(flashlight_gpio_table_rev_CX));
	} else {
		config_gpio_table(flashlight_gpio_table, ARRAY_SIZE(flashlight_gpio_table));
	}
	return 0;
}

static struct flashlight_platform_data bravo_flashlight_data = {
	.gpio_init = config_bravo_flashlight_gpios,
	.torch = BRAVO_GPIO_FLASHLIGHT_TORCH,
	.flash = BRAVO_GPIO_FLASHLIGHT_FLASH,
	.flash_duration_ms = 600
};

static struct platform_device bravo_flashlight_device = {
	.name = "flashlight",
	.dev = {
		.platform_data = &bravo_flashlight_data,
	},
};

static int flashlight_control(int mode)
{
        return aat1271_flashlight_control(mode);
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

///////////////////////////////////////////////////////////////////////
// Camera
///////////////////////////////////////////////////////////////////////

static uint32_t camera_off_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] =
{
	PCOM_GPIO_CFG(0, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT1 */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* HSYNC */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), /* VSYNC */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* MCLK */
};

int config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table));

	return 0;
}

void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table, ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] =
{
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		 INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data =
{
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data =
{
	.sensor_name = "s5k3e2fx",
	.sensor_reset = 144,
	/* CAM1_PWDN, enabled in a9 */
	//.sensor_pwd = 143,
	/* CAM1_VCM_EN, enabled in a9 */
	//.vcm_pwd = 31,
	.pdata = &msm_camera_device_data,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg = &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k3e2fx =
{
	.name     = "msm_camera_s5k3e2fx",
	.dev      = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};


///////////////////////////////////////////////////////////////////////
// Vibrator
///////////////////////////////////////////////////////////////////////

static struct timed_gpio timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = BRAVO_GPIO_VIBRATOR_ON,
		.max_timeout = 15000,
	},
};

static struct timed_gpio_platform_data timed_gpio_data = {
	.num_gpios	= ARRAY_SIZE(timed_gpios),
	.gpios		= timed_gpios,
};

static struct platform_device bravo_timed_gpios = {
	.name		= "timed-gpio",
	.id		= -1,
	.dev		= {
		.platform_data = &timed_gpio_data,
	},
};

///////////////////////////////////////////////////////////////////////
// Bluetooth
///////////////////////////////////////////////////////////////////////

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.rx_wakeup_irq = -1,
	.inject_rx_on_wakeup = 0,
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	.exit_lpm_cb = bcm_bt_lpm_exit_lpm_locked,
#endif
};
#ifdef CONFIG_SERIAL_BCM_BT_LPM
static struct bcm_bt_lpm_platform_data bcm_bt_lpm_pdata = {
	.gpio_wake = BRAVO_GPIO_BT_WAKE,
	.gpio_host_wake = BRAVO_GPIO_BT_HOST_WAKE,
	.request_clock_off_locked = msm_hs_request_clock_off,
	.request_clock_on_locked = msm_hs_request_clock_on_locked,
};

struct platform_device bcm_bt_lpm_device = {
	.name = "bcm_bt_lpm",
	.id = 0,
	.dev = {
		.platform_data = &bcm_bt_lpm_pdata,
	},
};

#endif
#endif

#define ATAG_BDADDR 0x43294329  /* bravo bluetooth address tag */
#define ATAG_BDADDR_SIZE 4
#define BDADDR_STR_SIZE 18

static char bdaddr[BDADDR_STR_SIZE];

module_param_string(bdaddr, bdaddr, sizeof(bdaddr), 0400);
MODULE_PARM_DESC(bdaddr, "bluetooth address");

static int __init parse_tag_bdaddr(const struct tag *tag)
{
	unsigned char *b = (unsigned char *)&tag->u;

	if (tag->hdr.size != ATAG_BDADDR_SIZE)
		return -EINVAL;

	snprintf(bdaddr, BDADDR_STR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X",
			b[0], b[1], b[2], b[3], b[4], b[5]);

	return 0;
}

__tagtable(ATAG_BDADDR, parse_tag_bdaddr);

static uint32_t bt_gpio_table[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

static uint32_t bt_gpio_table_rev_CX[] = {
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RTS, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_CTS, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_RX, 2, GPIO_INPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_UART1_TX, 2, GPIO_OUTPUT,
		      GPIO_PULL_UP, GPIO_8MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_RESET_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_SHUTDOWN_N, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_CDMA_GPIO_BT_WAKE, 0, GPIO_OUTPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
	PCOM_GPIO_CFG(BRAVO_GPIO_BT_HOST_WAKE, 0, GPIO_INPUT,
		      GPIO_PULL_DOWN, GPIO_4MA),
};

///////////////////////////////////////////////////////////////////////
// Battery
///////////////////////////////////////////////////////////////////////

static int ds2784_charge(int on, int fast)
{
	if (is_cdma_version(system_rev)) {
		if (!on)
			smb329_set_charger_ctrl(SMB329_DISABLE_CHG);
		else
			smb329_set_charger_ctrl(fast ? SMB329_ENABLE_FAST_CHG : SMB329_ENABLE_SLOW_CHG);
	}
	else
		gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_CURRENT, !!fast);
	gpio_direction_output(BRAVO_GPIO_BATTERY_CHARGER_EN, !on);
	return 0;
}

static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
	.gpio_mbat_in = -1,
	.gpio_mchg_en_n = BRAVO_GPIO_BATTERY_CHARGER_EN,
	.gpio_iset = BRAVO_GPIO_BATTERY_CHARGER_CURRENT,
	.gpio_adp_9v = BRAVO_GPIO_POWER_USB,
	.guage_driver = GUAGE_DS2784,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
	.force_no_rpc = 0,
/*	.int_data = {
		.chg_int = HTCLEO_GPIO_BATTERY_OVER_CHG,
	},*/
};



static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};

///////////////////////////////////////////////////////////////////////
// SPI
///////////////////////////////////////////////////////////////////////

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start  = INT_SPI_INPUT,
		.end    = INT_SPI_INPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start  = INT_SPI_OUTPUT,
		.end    = INT_SPI_OUTPUT,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start  = INT_SPI_ERROR,
		.end    = INT_SPI_ERROR,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start  = 0xA1200000,
		.end    = 0xA1200000 + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.name   = "spi_clk",
		.start  = 17,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_mosi",
		.start  = 18,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_miso",
		.start  = 19,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_cs0",
		.start  = 20,
		.end    = 1,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_pwr",
		.start  = 21,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_cs0",
		.start  = 22,
		.end    = 0,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct spi_platform_data bravo_spi_pdata = {
	.clk_rate	= 4800000,
};

struct platform_device qsd_device_spi = {
	.name           = "spi_qsd",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(qsd_spi_resources),
	.resource       = qsd_spi_resources,
	.dev		= {
		.platform_data = &bravo_spi_pdata
	},
};

///////////////////////////////////////////////////////////////////////
// Optical Joystick
///////////////////////////////////////////////////////////////////////

#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
static void curcial_oj_shutdown(int enable)
{
	uint8_t cmd[3];

	memset(cmd, 0x00, sizeof(uint8_t)*3);
	cmd[2] = 0x20;
	// microp firmware(v04) non-shutdown by default
	microp_i2c_write(0x90, cmd, 3);
	pr_err("%s\n", __func__);
}

#define CURCIAL_OJ_POWER		150
static int curcial_oj_poweron(int on)
{
	uint8_t data[2];

#ifdef CONFIG_MACH_BRAVO
	struct vreg *oj_power = vreg_get(0, "gp2");
	if (IS_ERR(oj_power)) {
		pr_err("%s: Error power domain\n", __func__);
		return 0;
	}

	if (on) {
		vreg_set_level(oj_power, 2750);
		vreg_enable(oj_power);
	} else {
		/* for microp firmware(v04) setting*/
		microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
		if (data[0] < 4) {
			printk("Microp firmware version: %d\n", data[0]);
			return 1;
		}
		vreg_disable(oj_power);
	}
	pr_err("%s: OJ power enable(%d)\n", __func__, on);
#else
	/* for microp firmware(v04) setting*/
	if (on == 0) {
		microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
		if (data[0] < 4) {
			printk("Microp firmware version:%d\n",data[0]);
			return 1;
		}
	}

	gpio_set_value(CURCIAL_OJ_POWER, on);

	if (gpio_get_value(CURCIAL_OJ_POWER) != on) {
		printk(KERN_ERR "%s:OJ:power status fail \n", __func__);
		return 0;
	}
	printk(KERN_ERR "%s:OJ:power status ok \n", __func__);
#endif
	return 1;
}

static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;

	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (1) {
		deltaX = (1)*((int8_t) data[2]); /*X=2*/
		deltaY = (-1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (-1)*((int8_t) data[1]);
		deltaY = (1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}

#define BRAVO_MICROP_VER	0x03
static struct curcial_oj_platform_data bravo_oj_data = {
	.oj_poweron	= curcial_oj_poweron,
	.oj_shutdown	= curcial_oj_shutdown,
	.oj_adjust_xy	= curcial_oj_adjust_xy,
	.microp_version	= BRAVO_MICROP_VER,
	.mdelay_time	= 0,
	.normal_th	= 8,
	.xy_ratio	= 15,
#ifdef CONFIG_MACH_BRAVO
	.interval	= 0,
	.swap		= false,
	.y		= -1,
#else
	.interval	= 10,
	.swap		= true,
	.y		= 1,
#endif
	.x		= 1,
	.share_power	= false,
	.debugflag	= 0,
	.ap_code	= false,
	.sht_tbl	= {0, 1000, 1250, 1500, 1750, 2000, 3000},
	.pxsum_tbl	= {0, 0, 90, 100, 110, 120, 130},
	.degree		= 7,
	.Xsteps = {0, 1, 2, 3, 4, 5, 6, 8, 10, 12,
		14, 16, 18, 20, 22, 24, 26, 27, 28, 29,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.Ysteps = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
		10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		9, 9, 9, 9, 9, 9, 9, 9, 9, 9},
	.irq		= MSM_uP_TO_INT(12),
};

static struct platform_device bravo_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data = &bravo_oj_data,
	}
};
#endif

///////////////////////////////////////////////////////////////////////
// Devices
///////////////////////////////////////////////////////////////////////

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart1,
#endif
	&qsd_device_spi,
#ifdef CONFIG_SERIAL_BCM_BT_LPM
	&bcm_bt_lpm_device,
#endif
	&msm_device_uart_dm1,
	&ram_console_device,
	&bravo_rfkill,
	&msm_device_dmov,
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_rtc,
#ifdef CONFIG_USB_G_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#endif
#ifdef CONFIG_ION_MSM
        &ion_dev,
#endif
#ifndef CONFIG_ION_MSM
	&android_pmem_device,
#endif
	&android_pmem_adsp_device,
	&android_pmem_venc_device,
	&msm_kgsl_3d0,
	&msm_device_i2c,
	&msm_camera_sensor_s5k3e2fx,
	&bravo_flashlight_device,
	&htc_battery_pdev,
#ifdef CONFIG_OPTICALJOYSTICK_CRUCIAL
	&bravo_oj,
#endif
	&capella_cm3602,
};

static struct msm_gpio misc_gpio_table[] = {
    { GPIO_CFG(BRAVO_GPIO_LCD_RST_N, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
    { GPIO_CFG(BRAVO_GPIO_LED_3V3_EN, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
    { GPIO_CFG(BRAVO_GPIO_DOCK, 0, GPIO_CFG_OUTPUT,
               GPIO_CFG_NO_PULL, GPIO_CFG_4MA)},
};

static struct msm_gpio key_int_shutdown_gpio_table[] = {
    {GPIO_CFG(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_NO_PULL, GPIO_CFG_2MA)},
};

static void bravo_headset_init(void)
{
	if (is_cdma_version(system_rev))
		return;
	msm_gpios_enable(key_int_shutdown_gpio_table,
                         ARRAY_SIZE(key_int_shutdown_gpio_table));
	gpio_set_value(BRAVO_GPIO_35MM_KEY_INT_SHUTDOWN, 0);
}



#ifdef CONFIG_PERFLOCK
static unsigned bravo_perf_acpu_table[] = {
	245000000,
	576000000,
	998400000,
};

static struct perflock_platform_data bravo_perflock_data = {
	.perf_acpu_table = bravo_perf_acpu_table,
	.table_size = ARRAY_SIZE(bravo_perf_acpu_table),
};
#endif

///////////////////////////////////////////////////////////////////////
// Reset
///////////////////////////////////////////////////////////////////////

static void bravo_reset(void)
{
	printk("bravo_reset()\n");
	gpio_set_value(BRAVO_GPIO_PS_HOLD, 0);
}

static void do_grp_reset(void)
{
   	writel(0x20000, MSM_CLK_CTL_BASE + 0x214);
}

static void do_sdc1_reset(void)
{
	volatile uint32_t* sdc1_clk = MSM_CLK_CTL_BASE + 0x218;

	*sdc1_clk |= (1 << 9);
   	mdelay(1);
	*sdc1_clk &= ~(1 << 9);
}

///////////////////////////////////////////////////////////////////////
// I2C
///////////////////////////////////////////////////////////////////////

#define GPIO_I2C_CLK 95
#define GPIO_I2C_DAT 96

static void msm_i2c_gpio_config(int adap_id, int config_type)
{
	unsigned id;


	if (adap_id > 0) return;

	if (config_type == 0) 
	{
		id = GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	} 
	else 
	{
		id = GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}


static struct msm_i2c_platform_data msm_i2c_pdata = 
{
	.clk_freq = 100000,
	.pri_clk = GPIO_I2C_CLK,
	.pri_dat = GPIO_I2C_DAT,
	.rmutex  = 0,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	msm_i2c_gpio_init();
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}
///////////////////////////////////////////////////////////////////////
// PM Platform data
///////////////////////////////////////////////////////////////////////

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};
#define CT_CSR_PHYS		0xA8700000
#define TCSR_SPI_MUX		(ct_csr_base + 0x54)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_csr_base = 0;
	u32 spi_mux;
	int ret = 0;

	ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
	if (!ct_csr_base) {
		pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
		return -1;
	}

	spi_mux = readl(TCSR_SPI_MUX);
	switch (spi_mux) {
	case (1):
		qsd_spi_resources[4].start  = DMOV_HSUART1_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART1_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[4].start  = DMOV_HSUART2_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART2_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[4].start  = DMOV_CE_OUT_CHAN;
		qsd_spi_resources[4].end    = DMOV_CE_IN_CHAN;
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -1;
	}

	iounmap(ct_csr_base);
	return ret;
}

static uint32_t qsd_spi_gpio_config_data[] = {
	PCOM_GPIO_CFG(17, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	PCOM_GPIO_CFG(18, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	PCOM_GPIO_CFG(19, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	PCOM_GPIO_CFG(20, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	PCOM_GPIO_CFG(21, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
};

static int msm_qsd_spi_gpio_config(void)
{
	int rc;

	config_gpio_table(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));

	/* Set direction for SPI_PWR */
	gpio_direction_output(21, 1);

	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 19200000,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	int rc;
	rc = gpio_request(21, "spi_pwr");
	if (rc)
		pr_err("Failed requesting spi_pwr gpio\n");
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

int bravo_init_mmc(int sysrev, unsigned debug_uart);

static void __init bravo_init(void)
{
	int ret;

	pr_info("bravo_init() revision=%d\n", system_rev);
	msm_hw_reset_hook = bravo_reset;

	do_grp_reset();
	do_sdc1_reset();

	msm_clock_init(&qds8x50_clock_init_data);
	acpuclk_init(&acpuclk_8x50_soc_data);

	msm_gpios_enable(misc_gpio_table, ARRAY_SIZE(misc_gpio_table));

	gpio_request(BRAVO_GPIO_TP_LS_EN, "tp_ls_en");
	gpio_direction_output(BRAVO_GPIO_TP_LS_EN, 0);
	gpio_request(BRAVO_GPIO_TP_EN, "tp_en");
	gpio_direction_output(BRAVO_GPIO_TP_EN, 0);
	gpio_request(BRAVO_GPIO_LS_EN_N, "ls_en");
	gpio_request(BRAVO_GPIO_COMPASS_INT_N, "compass_int");
	gpio_direction_input(BRAVO_GPIO_COMPASS_INT_N);

	gpio_request(BRAVO_GPIO_DS2482_SLP_N, "ds2482_slp_n");

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

	config_gpio_table(bt_gpio_table, ARRAY_SIZE(bt_gpio_table));

	platform_add_devices(devices, ARRAY_SIZE(devices));

	platform_add_devices(msm_footswitch_devices,
			msm_num_footswitch_devices);

    	msm_device_i2c_init();
	msm_qsd_spi_init();

	i2c_register_board_info(0, base_i2c_devices, ARRAY_SIZE(base_i2c_devices));

	ret = bravo_init_mmc(system_rev, debug_uart);
	if (ret != 0)
		pr_crit("%s: Unable to initialize MMC\n", __func__);

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(MSM_PM_BOOT_CONFIG_RESET_VECTOR, ioremap(0x0, PAGE_SIZE)));
#ifdef CONFIG_USB_G_ANDROID
	bravo_add_usb_devices();
#endif

	bravo_audio_init();
	bravo_headset_init();

	platform_device_register(&bravo_timed_gpios);
}

static void __init bravo_fixup(struct machine_desc *desc, struct tag *tags,
				 char **cmdline, struct meminfo *mi)
{
	printk("bravo_fixup(...)\n");
	mi->nr_banks = 2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = MSM_EBI1_BANK0_SIZE;
	mi->bank[1].start = MSM_EBI1_BANK1_BASE;
	mi->bank[1].size = MSM_EBI1_BANK1_SIZE;

}

static void __init bravo_allocate_memory_regions(void)
{
	unsigned long size;

	size = MSM_FB_SIZE;
	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at 0x%p (0x%lx physical) for fb\n",
		size, __va(MSM_FB_BASE), (unsigned long) MSM_FB_BASE);
}

static void __init bravo_init_early(void)
{
	bravo_allocate_memory_regions();
}

static void __init bravo_map_io(void)
{
    printk("bravo_map_io()\n");
    msm_map_qsd8x50_io();

    if (socinfo_init() < 0)
        pr_err("socinfo_init() failed!\n");
}

extern struct sys_timer msm_timer;

#ifdef CONFIG_MACH_BRAVO
MACHINE_START(BRAVO, "bravo")
#else
MACHINE_START(BRAVOC, "bravoc")
#endif
    .boot_params = 0x20000100,
    .fixup = bravo_fixup,
    .map_io = bravo_map_io,
    .reserve = qsd8x50_reserve,
    .init_irq = msm_init_irq,
    .init_machine = bravo_init,
    .timer = &msm_timer,
    .init_early = bravo_init_early,
MACHINE_END
