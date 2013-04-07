/* include/linux/ds2784_battery.h
 *
 * Copyright (C) 2009 HTC Corporation
 * Copyright (C) 2009 Google, Inc.
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
#ifndef _DS2784_BATTERY_H_
#define _DS2784_BATTERY_H_
#include <linux/notifier.h>
#include <mach/htc_battery.h>
#include <linux/wrapper_types.h>

struct battery_type{

		BOOL is_power_on_reset;

		INT32 voltage_mV;
		INT32 current_mA;
		INT32 discharge_mA;
		INT32 charge_counter_mAh;
		INT32 temp_01c;
		INT32 last_temp_01c;
		INT32 id_ohm;
		INT32 vref_mv;

		INT32 voltage_adc;
		INT32 current_adc;
		INT32 discharge_adc;
		INT32 charge_counter_adc;
		INT32 temp_adc;
		INT32 last_temp_adc;
		INT32 id_adc;
		INT32 vref_adc;

		INT32 id_index;
		INT32 charge_full_design_mAh;
		INT32 charge_full_real_mAh;

		INT32 temp_index;
		INT32 temp_check_index;

		INT32 KADC_01p;
		INT32 RARC_01p;
		INT32 pd_m;

		INT32 software_charge_counter_mAms;
		INT32 thermal_id;
};

struct protect_flags_type{

		BOOL is_charging_enable_available;
		BOOL is_charging_high_current_avaialble;
		BOOL is_charging_indicator_available;
		BOOL is_battery_dead;
#if 0
		BOOL is_battery_overtemp;
#endif
		BOOL is_fake_room_temp;
};


enum ds2784_notify_evt_t{
	DS2784_CHARGING_CONTROL = 0,
	DS2784_LEVEL_UPDATE,
	DS2784_BATTERY_FAULT,
	DS2784_OVER_TEMP,
	DS2784_NUM_EVENTS,
};

struct poweralg_type
{
	int charge_state;
	int capacity_01p;
	int last_capacity_01p;
	int fst_discharge_capacity_01p;
	int fst_discharge_acr_mAh;
	int charging_source;
	int charging_enable;
	BOOL is_need_calibrate_at_49p;
	BOOL is_need_calibrate_at_14p;
	BOOL is_charge_over_load;
	struct battery_type battery;
	struct protect_flags_type protect_flags;
	BOOL is_china_ac_in;
	BOOL is_cable_in;
	BOOL is_voltage_stable;
	BOOL is_software_charger_timeout;
	UINT32 state_start_time_ms;
};

struct poweralg_config_type
{
	INT32 full_charging_mv;
	INT32 full_charging_ma;
	INT32 full_pending_ma;			/* 0 to disable*/
	INT32 full_charging_timeout_sec;	 /* 0 to disable*/
	INT32 voltage_recharge_mv;  		 /* 0 to disable*/
	INT32 capacity_recharge_p;  		 /* 0 to disable*/
	INT32 voltage_exit_full_mv; 		 /* 0 to disable*/
	INT32 wait_votlage_statble_sec;
	INT32 predict_timeout_sec;
	INT32 polling_time_in_charging_sec;
	INT32 polling_time_in_discharging_sec;

	BOOL enable_full_calibration;
	BOOL enable_weight_percentage;
	INT32 software_charger_timeout_sec;  /* 0 to disable*/

	BOOL debug_disable_shutdown;
	BOOL debug_fake_room_temp;
	BOOL debug_disable_hw_timer;
	BOOL debug_always_predict;
	INT32 full_level;                  /* 0 to disable*/
};

#ifdef CONFIG_BATTERY_DS2784
extern int ds2784_register_notifier(struct notifier_block *nb);
extern int ds2784_unregister_notifier(struct notifier_block *nb);
extern int ds2784_get_battery_info(struct battery_info_reply *batt_info);
extern ssize_t htc_battery_show_attr(struct device_attribute *attr,
			char *buf);
#else
static int ds2784_register_notifier(struct notifier_block *nb) { return 0; }
static int ds2784_unregister_notifier(struct notifier_block *nb) { return 0; }
static int ds2784_get_battery_info(struct battery_info_reply *batt_info)
{
	batt_info->level = 10;
	return 0;
}
extern ssize_t htc_battery_show_attr(struct device_attribute *attr,
			char *buf){ return 0; }

#endif
struct ds2784_platform_data {
	int (*charge)(int on, int fast);
	void *w1_slave;
};

/* battery algorithm public functions*/

int get_state_check_interval_min_sec( void);
BOOL do_power_alg( BOOL is_event_triggered);
void power_alg_init( struct poweralg_config_type *debug_config);
void power_alg_preinit( void);
void calibrate_id_ohm( struct battery_type *battery);

#endif //endif _DS2784_BATTERY_H_
