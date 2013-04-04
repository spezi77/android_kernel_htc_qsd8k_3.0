/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <mach/irqs.h>
#include <mach/gpiomux.h>
#include "gpiomux-8x50.h"

static struct gpiomux_setting uart3_suspended_cfg = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
};

#if defined(CONFIG_MMC_MSM) || defined(CONFIG_MMC_MSM_MODULE)
static struct gpiomux_setting sdc1_data_active_cfg = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_8MA,
    .pull = GPIOMUX_PULL_UP,
};
static struct gpiomux_setting sdc1_clk_active_cfg = {
    .func = GPIOMUX_FUNC_1,
    .drv = GPIOMUX_DRV_8MA,
    .pull = GPIOMUX_PULL_NONE,
};
#else
#define SDCC_DAT_0_3_CMD_ACTV_CFG 0
#define SDCC_CLK_ACTV_CFG 0
static struct gpiomux_setting sdc1_data_active_cfg = { };
static struct gpiomux_setting sdc1_clk_active_cfg = { };
#endif

static struct gpiomux_setting sdc1_suspended_cfg = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config qsd8x50_uart_configs[] __initdata = {
    {
        .gpio = 86,
        .settings = {
            [GPIOMUX_SUSPENDED] = &uart3_suspended_cfg,
        },
    },
    {
        .gpio = 87,
        .settings = {
            [GPIOMUX_SUSPENDED] = &uart3_suspended_cfg,
        },
    },
};

static struct msm_gpiomux_config qsd8x50_sdc_configs[] __initdata = {
    {
        .gpio = 51,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_data_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
    {
        .gpio = 52,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_data_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
    {
        .gpio = 53,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_data_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
    {
        .gpio = 54,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_data_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
    {
        .gpio = 55,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_data_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
    {
        .gpio = 56,
        .settings = {
            [GPIOMUX_ACTIVE]    = &sdc1_clk_active_cfg,
            [GPIOMUX_SUSPENDED] = &sdc1_suspended_cfg,
        },
    },
};

struct msm_gpiomux_configs
qsd8x50_gpiomux_cfgs[] __initdata = {
    {qsd8x50_uart_configs, ARRAY_SIZE(qsd8x50_uart_configs)},
    {qsd8x50_sdc_configs, ARRAY_SIZE(qsd8x50_sdc_configs)},
    {NULL, 0},
};

void __init qsd8x50_init_gpiomux(struct msm_gpiomux_configs *cfgs)
{
    int rc;
    rc = msm_gpiomux_init(NR_GPIO_IRQS);
    if (rc) {
        pr_err("%s failure: %d\n", __func__, rc);
        return;
    }

    while (cfgs->cfg) {
        msm_gpiomux_install(cfgs->cfg, cfgs->ncfg);
        ++cfgs;
    }
}
