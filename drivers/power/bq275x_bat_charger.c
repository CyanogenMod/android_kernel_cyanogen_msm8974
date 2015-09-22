/*
 * BQ27x00 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

/*
 * Datasheets:
 * http://focus.ti.com/docs/prod/folders/print/bq27000.html
 * http://focus.ti.com/docs/prod/folders/print/bq27500.html
 */
/*
   BQ27531+BQ2419x for K5

 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/reboot.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/string.h>
#include <linux/power/bq27x00_battery.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include "bq275x_firmware.h"
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/qpnp/power-on.h>

#ifdef DEBUG
#define DBG(fmt...) printk(KERN_DEBUG fmt)
#else
#define DBG(fmt...)
#endif

#define DRIVER_VERSION			"1.2.0"

#define SUPPORT_QPNP_VBUS_OVP
//#define BQ_WRITE_TEMP

#define BQ27530_REG_CNTL		0x00 /*control*/
#define BQ27x00_REG_TEMP		0x06 /*Temperature*/
#define BQ27x00_REG_VOLT		0x08 /*Voltage*/
#define BQ27x00_REG_AI			0x14 /*AverageCurrent*/
#define BQ27x00_REG_FLAGS		0x0A /*FLAGS*/
#define BQ27500_FLAG_DSC		BIT(0)/*Discharging detected*/
#define BQ27500_FLAG_SOC1		BIT(2)/*Soc threshold 1*/
#define BQ27500_FLAG_BAT_DET	BIT(3)/*bat_detected*/
#define BQ27500_FLAG_CHG_EN		BIT(8)/*enable charge*/
#define BQ27500_FLAG_FC			BIT(9)/*Full-charged condition reached*/
#define BQ27500_FLAG_CNTL_EN		BIT(10)/*Full-charged condition reached*/
#define BQ27500_FLAG_UT			BIT(14)/*Undertemperature*/
#define BQ27500_FLAG_OT			BIT(15)/*Overtemperature*/
#define BQ27530_REG_RM			0x10 /*Remaining Capacity*/
#define BQ27530_REG_FCC			0x12 /*FullChargeCapacity*/
#define BQ27500_REG_PCAI		0x1E /*get Charging Current*/
#define BQ27500_REG_PCV			0x20 /*get Charging Voltage*/
#define BQ27500_REG_SOC			0x2C /*SOC*/
#define BQ27500_REG_TRUESOC		0x2E /*TRUE SOC*/
#define BQ27x00_REG_TTE			0x16 /*TimeToEmpty*/
#define BQ27x00_REG_NAC			0x0C /* Nominal available capaciy */
#define BQ27x00_REG_FAC			0x0E /* Full Available capaciy */
#define BQ27x00_REG_CYCT		0x2A /* Cycle count total */
#define BQ27530_REG_CCAI		0x70 /*CalcChargingCurrent*/
#define BQ27530_REG_CCV			0x72 /*CalcChargingVoltage*/
#define BQ27500_REG_DCAP		0x3C /* Design capacity*/

/*Charger Command*/
#define BQ24192_REG_STAT		0x74 /*ChargerStatus*/
#define BQ24192_STAT_WE			BIT(7) /*Waiting on GG_CHGCNTRL_ENB command*/
#define BQ24192_STAT_ERR		BIT(6) /*I2C comm error 27530-24912*/
#define BQ24192_STAT_WFAIL		BIT(4) /*Write Charger error*/
#define BQ24192_STAT_INIT		BIT(2) /*init error*/
#define BQ24192_STAT_WRITE		BIT(1) /*write*/
#define BQ24192_STAT_READ		BIT(0) /*read*/

#define BQ24192_REG_CTL_0		0x75 /*Charger Control REG 0*/
#define BQ24192_0_HIZ   		BIT(7) /*Charger Source*/
#define BQ24192_0_STAT_2		BIT(6) /*Charger Source*/
#define BQ24192_0_STAT_1		BIT(5)
#define BQ24192_0_STAT_0		BIT(4)
#define BQ24192_0_FAL_2			BIT(2)/*Charger fault status bit*/
#define BQ24192_0_FAL_1			BIT(1)
#define BQ24192_0_FAL_0			BIT(0)
#define BQ24192_REG_POR_1		0x76 /*Charger Control REG 1*/

#define BQ24192_REG_CURRENT		0x77 /*Charge current */
#define BQ24192_CC_5			BIT(7) /*2048mA */
#define BQ24192_CC_4			BIT(6) /*1024mA */
#define BQ24192_CC_3			BIT(5) /*512mA */
#define BQ24192_CC_2			BIT(4) /*256mA */
#define BQ24192_CC_1			BIT(3) /*128mA */
#define BQ24192_CC_0			BIT(2) /*64mA */

#define BQ24192_REG_PRE_TERM_CURRENT	0x78 /*PreTerm_Charge current */
#define BQ24192_PRE_CC_3		BIT(7) /*1024mA */
#define BQ24192_PRE_CC_2		BIT(6) /*512mA */
#define BQ24192_PRE_CC_1		BIT(5) /*256mA */
#define BQ24192_PRE_CC_0		BIT(4) /*128mA */
#define BQ24192_TERM_CC_3		BIT(3) /*1024mA */
#define BQ24192_TERM_CC_2		BIT(2) /*512mA */
#define BQ24192_TERM_CC_1		BIT(1) /*256mA */
#define BQ24192_TERM_CC_0		BIT(0) /*128mA */


#define BQ24192_REG_VOLT_4		0x79 /*Charger Voltage REG 3*/
#define BQ24192_4_VREG_5		BIT(7) /*512mV*/
#define BQ24192_4_VREG_4		BIT(6) /*256mV*/
#define BQ24192_4_VREG_3		BIT(5) /*128mV*/
#define BQ24192_4_VREG_2		BIT(4) /*64mV*/
#define BQ24192_4_VREG_1		BIT(3) /*32mV*/
#define BQ24192_4_VREG_0		BIT(2) /*16mV*/
#define BQ24192_4_BATLOWV		BIT(1) /*Pre-charge to fast charge 0=2.8V 1=3.0V*/
#define BQ24192_4_VRECHG		BIT(0) /*Recharge Threshold 0=100mV 1=300mV*/

#define BQ24192_REG_TERM_TIMER		0x7a
#define BQ24192_5_EN_TERM		BIT(7) /*term detect by coulomb counter*/
#define BQ24192_5_WT_TIMER		0x03 /*watchdog timer set*/
#define BQ24192_5_EN_TIMER		BIT(3) /*0=disable charging timer,1=enable*/
#define BQ24192_5_CHG_TIMER		0x03 /*Fast charging timer*/

#define BQ24192_REG8_STATUS		0x7D
#define BQ24192_8_VBUS_STAT		0x03
#define BQ24192_8_CHRG_STAT		0x03
#define BQ24192_8_DPM_STAT		BIT(3)
#define BQ24192_8_PG_STAT		BIT(2)

#define BQ24192_REG9_FAULT		0x7E
#define BQ24192_9_WT			BIT(7)
#define BQ24192_9_OTG			BIT(6)
#define BQ24192_9_CHRG			0x03
#define BQ24192_9_BAT			BIT(3)

#define BQ24192_REG_DEV_VER		0x7F
#define BQ24192_A_PN			0x07


#define BQ27x00_REG_TTF			0x18 /*TimeToFull--------del or rename*/
#define BQ27x00_REG_TTECP		0x26 /*del or rename FCCF*/
#define BQ27x00_REG_LMD			0x12 /* Last measured discharge ---del or rename*/
#define BQ27x00_REG_AE			0x22 /* Available enery----del or rename*/
#define BQ27000_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27000_REG_ILMD		0x76 /* Initial last measured discharge */
#define BQ27000_FLAG_CHGS		BIT(7)
#define BQ27000_FLAG_FC			BIT(5)
#define BQ27000_RS			20 /* Resistor sense */

/* ADC Channel Numbers */
#define CLT_BATT_NUM_GPADC_SENSORS	1
#define CLT_GPADC_USB_VOLTAGE		0x8
#define CLT_GPADC_VOLTAGE_SAMPLE_COUNT	10


#define BQ24190_IC_VERSION			0x4
#define BQ24192_IC_VERSION			0x5
#define BQ24292I_IC_VERSION			0x3
#define BQ24192I_IC_VERSION			0x3

#define BATT_TEMP_MAX			600
#define BATT_VOLT_MAX			4425
#define BATT_VOLT_MIN			3000

#define BQ_I2C_VTG_MIN_UV	1800000
#define BQ_I2C_VTG_MAX_UV	1800000
#define BQ_I2C_LOAD_UA	10000
#define BQ_I2C_LPM_LOAD_UA	10

#define BQ_BATT_TEMP_OFFSET	20

#define BATT_VENDOR_LG			1
#define BATT_VENDOR_ATL_AND_DUMY	2

#define BATT_FW_VER_LG		0xAC02
#define BATT_FW_VER_ATL		0xAD02

extern long qpnp_batt_id;
int soc_prev = 0;
volatile int rd_count=0;
volatile int htemp_cnt=0;
int temp_prev = 2986;
struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(struct bq27x00_device_info *di, u8 reg, bool single);
};

enum current_level {
    CURRENT_LOW,
    CURRENT_HIGH,
    CURRENT_UNDEFINE,
};

enum usbin_health {
	USBIN_UNKNOW,
	USBIN_OK,
	USBIN_OVP,
	USBIN_DPM,
};

struct bq27x00_reg_cache {
	int temperature;
	int time_to_empty;
	int time_to_empty_avg;
	int time_to_full;
	int charge_full;
	int cycle_count;
	int capacity;
	int flags;
	int voltage;
	int current_now;
};

struct bq27530_platform_data{
   unsigned fg_int_gpio; 
   unsigned usb_sw_en_gpio; 
   unsigned chg_int_gpio; 
   unsigned usb_sw_gpio; 
   unsigned chg_en_gpio; 
   unsigned chg_otg_gpio; 
   unsigned chg_psel_gpio; 
   u32  fg_int_flag;
   u32  chg_int_flag;
   u32  chg_current;
   bool  chg_en_flag;
   const char *fw_image_name;
};

struct bq27x00_device_info {
	struct device 		*dev;
    struct bq27530_platform_data *board;
	struct regulator *vcc_i2c;
	int			id;

	struct bq27x00_reg_cache cache;
	int charge_design_full;
    int chrg_type;

	unsigned long last_update;
	struct delayed_work work;
	struct delayed_work ovp_work;
	struct power_supply	bat;
	struct power_supply	mains;
	struct power_supply	usb;
	bool mains_online;
	bool usb_online;
	int usb_present;
	int host_mode;
	char chrg_vendor[16];
	struct bq27x00_access_methods bus;
	int chrg_irq_n;
	int fg_irq_n;
	int usb_ovp;
	int chg_state;
	int chg_pn;
	int df_ver;

	struct mutex lock;
	struct wake_lock wakelock;
	struct wake_lock lowcap_wakelock;

	int is_rom_mode;
	int fw_dw_done;
	int is_suspend;
	int soc_reset;
	int reset_p;
	int present_check_count;

#ifdef SUPPORT_QPNP_VBUS_OVP
	struct delayed_work vbus_work;
	int vbus_ovp;
#endif
#ifdef CONFIG_FB
        struct notifier_block fb_notif;
#endif
        struct notifier_block kpdpwr_notif;
	int thermal_mitigation;
	struct delayed_work boot_work;

	int battery_vendor_index;
};

#ifdef SUPPORT_QPNP_VBUS_OVP
extern int qpnp_check_vbus_ovp(int *vusb);
static int vusb_uv = 0;
#endif

struct bq27x00_device_info *bqdi = NULL;

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};
static enum power_supply_property bq24912_mains_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER
};
static enum power_supply_property bq24912_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MANUFACTURER
};

int g_call_status = 0;

enum iinlim_level {
    IINLIM_100 = 0,
    IINLIM_150,
    IINLIM_500,
    IINLIM_900,
    IINLIM_1200,
    IINLIM_1500,
    IINLIM_2000,
    IINLIM_3000,
};

#ifdef CONFIG_FB
int fb_status = FB_BLANK_UNBLANK;
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data);
static int bq27531_config_charging_current(struct bq27x00_device_info *di, int level);
#endif
static void configure_fb_notifiler(struct bq27x00_device_info *di);
static int bq27531_op_thermal_mitigation(struct bq27x00_device_info *di, int level);
static int bq27531_op_set_input_limit(struct bq27x00_device_info *di, int value);
static int bq27531_charge_ic_reset(void);
static int bq27531_soc_reset(void);
static int bq27531_charge_hiz_reset(void);
static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single);

static unsigned int bc_fw = 0;
static unsigned int uart_switch = 0;
static unsigned int switch_to_uart = 0;
module_param(switch_to_uart, uint, 0444);
MODULE_PARM_DESC(switch_to_uart, "get fw download state");

static unsigned int fw_delay = 0;
module_param(fw_delay, uint, 0644);
MODULE_PARM_DESC(fw_delay, "Get fw download delay");

static unsigned int fw_long_delay = 70;
module_param(fw_long_delay, uint, 0644);
MODULE_PARM_DESC(fw_long_delay, "Get fw download delay");

static unsigned int fw_more_delay = 70;
module_param(fw_more_delay, uint, 0644);
MODULE_PARM_DESC(fw_long_delay, "Get fw download delay");

static int disable_charging_force = 0;
static int chg_type = POWER_SUPPLY_TYPE_BATTERY;

static unsigned int delay_t = 30000;//30s
module_param(delay_t, uint, 0644);
MODULE_PARM_DESC(delay_t, "The value is charge state update time");

int force_check_thermal = -1;
int boot_done = 0;

extern int bq24192_is_usbin_present(void);
extern int bq24192_get_usbin_health(void);
extern int bq24192_is_usbin_hostmode(void);
#ifdef BQ_WRITE_TEMP
static int bq27531_data_flash_write(struct bq27x00_device_info *di,unsigned char sub_class,
				unsigned char offset,unsigned char new_value);
#endif
static int ovp_check_cnt = 0;
/*
 * Common code for BQ27x00 devices
 */
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}
static unsigned long last_report_soc_time = 0;
static unsigned long low_vol_start_time = 0;
static int last_report_soc = -1;
static int last_cache_soc = 0;
static int delta_soc = 0;
static int batt_status = 0;//0:discharging,1:charing
static unsigned long reset_time = 0;	/*The time when reset*/

int calculate_report_soc(struct bq27x00_device_info *di)
{
	unsigned long now_tm_sec = 0;
	int delta_time;
	int low_vol_delta_time;
	int current_now;
	int current_reset;	/*current when soc reset*/
	int fg_pc_volt;
	int rsoc = -1;
	int tsoc = -1;		/*true soc*/
	int d_soc;		/*The different between truesoc and soc*/

	di->usb_present = bq24192_is_usbin_present();
	di->host_mode = bq24192_is_usbin_hostmode();

	if(get_current_time(&now_tm_sec)){
		pr_err("RTC read failed.\n");
		return -1;
	}

	delta_time = now_tm_sec - last_report_soc_time;

	if(di->usb_present && !di->host_mode){
		if(batt_status != 1){
			batt_status = 1;//charging
			last_report_soc_time = now_tm_sec;
			delta_time = 0;
		}
		if(last_report_soc > 0){
			current_now = (int)((s16)di->cache.current_now);
			/*printk("%s:temp=%d,voltage=%d,curr=%d,batt_status=%d.\n",__func__,di->cache.temperature,di->cache.voltage,current_now,batt_status);*/
			fg_pc_volt = bq27x00_read(di,BQ27500_REG_PCV,false);
			if(di->cache.temperature-2731 >= 450
					&& di->cache.temperature-2731 < 500
					&& (fg_pc_volt == 4112)
					&& di->cache.voltage >= 4100
					&& current_now < 500){
				if(delta_time > 20 && last_report_soc < 100){
					di->cache.capacity = last_report_soc+1;
				}else{
					di->cache.capacity = last_report_soc;
				}
				printk("%s:cache_cap=%d,report_soc=%d,del_time=%d.\n",__func__,di->cache.capacity,last_report_soc,delta_time);
			}else{
				if(((di->cache.capacity	- last_report_soc) > 1)){
					if(delta_time < 60){
						di->cache.capacity = last_report_soc;
					}else{
						di->cache.capacity = last_report_soc+1;
					}
					printk("%s charging:delta_time=%d,calc_cap=%d,last_report_soc=%d.\n",
						__func__,delta_time,di->cache.capacity,last_report_soc);
				}else if(di->cache.capacity < last_report_soc){
					if(delta_soc < 0 && delta_time > 60){
						di->cache.capacity = last_report_soc-1;
					}else{
						di->cache.capacity = last_report_soc;
					}
					printk("%s charging:delta_soc=%d,delta_time=%d,calc_cap=%d,last_report_soc=%d.\n",
						__func__,delta_soc,delta_time,di->cache.capacity,last_report_soc);
				}
			}
			/*printk("%s:cache_cap=%d,report_soc=%d,del_time=%d.\n",__func__,di->cache.capacity,last_report_soc,delta_time);*/
		}
	}else{
		/*add wake lock*/
		if(!wake_lock_active(&di->lowcap_wakelock))
			wake_lock(&di->lowcap_wakelock);

		if(batt_status != 0){
			batt_status = 0;//discharging
			last_report_soc_time = now_tm_sec;
			delta_time = 0;
		}
		if(last_report_soc > 0){
			if(di->cache.capacity > last_report_soc){
				printk("%s discharging:report last_soc=%d.\n",__func__,last_report_soc);
				di->cache.capacity = last_report_soc;
			}else if((last_report_soc-di->cache.capacity) > 1){
				if(delta_time < 30){
					di->cache.capacity = last_report_soc;
				}else if(delta_time < 600){
					di->cache.capacity = last_report_soc-1;
				}
				printk("%s discharging:delta_time=%d,calc_cap=%d,last_report_soc=%d.\n",
					__func__,delta_time,di->cache.capacity,last_report_soc);
			}
		}

		if(di->cache.voltage <= 3400){
			if(low_vol_start_time > 0){
				low_vol_delta_time = now_tm_sec - low_vol_start_time;
				dev_info(di->dev,"low_vol_delta_time=%d,low_vol_start_time=%ld.\n",low_vol_delta_time,low_vol_start_time);
				if(low_vol_delta_time > 60){
					dev_info(di->dev,"report for low voltage,cache_cap=%d.\n",di->cache.capacity);
					di->cache.capacity = 0;
					// ? low_vol_start_time = now_tm_sec;
				}
			}else{
				low_vol_start_time = now_tm_sec;
			}
		}else{
			low_vol_start_time = 0;
		}

		if(di->cache.capacity <= 1){
			bq27531_charge_ic_reset();
			if(last_report_soc > 0){
				if(delta_time < 20){
					di->cache.capacity = last_report_soc;
				}else{
					di->cache.capacity = last_report_soc-1;
				}
			}
			dev_info(di->dev,"availabe remain capcity,report cap=%d.\n",last_report_soc);

			if(di->cache.capacity == 0){
				if(g_call_status){
					di->cache.capacity = 1;
				} else if(di->cache.voltage <= 3400 || (di->cache.flags&0x2) == 0x2){
					di->cache.capacity = 0;
					dev_info(di->dev,"report cap=0.\n");
				} else {
					di->cache.capacity = 1;
				}
			}
		}else{
			if(wake_lock_active(&di->lowcap_wakelock))
				wake_unlock(&di->lowcap_wakelock);
		}

	}
	if(last_report_soc != di->cache.capacity){
		last_report_soc_time = now_tm_sec;
		power_supply_changed(&di->bat);
	}

	if(di->cache.capacity > 100)
		di->cache.capacity = 100;

	last_report_soc = di->cache.capacity;

 	/* "Liulf8 begin" Reset soc when truesoc is different between soc.Not allow twice reset in 6 hours*/
	/*d_soc > 10 means soc is abnormal.d_soc = |true_soc - soc|*/
	rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	tsoc = bq27x00_read(di, BQ27500_REG_TRUESOC, false);

	d_soc = rsoc - tsoc;
	if(d_soc < 0)
		d_soc = 0 - d_soc;

	current_now = bq27x00_read(di, BQ27x00_REG_AI, false);
	if(current_now < 0)
		current_reset = 0 - current_now;
	else 
		current_reset = current_now;

	if((d_soc > 10) && di->reset_p)
		printk("%s: vol = soc=%d,tsoc=%d.\n", __func__, rsoc, tsoc);

	if(di->reset_p)
		if((now_tm_sec - reset_time) > 21600)
			di->reset_p = 0;
		
	if ((di->cache.temperature-2731 >= 200) && (di->cache.temperature-2731 <= 400)&&(!di->reset_p) && (!di->soc_reset) && (di->fw_dw_done)) {
		if((d_soc > 10)&&(d_soc < 15) && (current_reset < 500)) {
			printk("%s: fg reset 10, %d,%d,%d,%d\n", __func__, di->cache.temperature-2731, current_now, rsoc, tsoc);
				bq27531_soc_reset();
				reset_time = now_tm_sec;
				di->reset_p = 1;
		}else if(d_soc > 15){
			printk("%s: fg reset 15, %d,%d,%d,%d\n", __func__, di->cache.temperature-2731, current_now, rsoc, tsoc);
				bq27531_soc_reset();
				reset_time = now_tm_sec;
				di->reset_p = 1;
			
		}
	}else if((d_soc > 20) && (!di->reset_p) && (!di->soc_reset) && (di->fw_dw_done)){
			printk("%s: fg reset T, %d,%d,%d,%d\n", __func__, di->cache.temperature-2731, current_now, rsoc, tsoc);
				bq27531_soc_reset();
				reset_time = now_tm_sec;
				di->reset_p = 1;
		}
 	/* "Liulf8 end" Reset soc when truesoc is different between soc.Not allow twice reset in 6 hours*/
	
	return 0;

}
static inline int bq27x00_read(struct bq27x00_device_info *di, u8 reg,
		bool single)
{
	return di->bus.read(di, reg, single);
}

void bq24192_update_chrg_type(int type)
{
	printk("usb set external chrg_type=%d.\n",type);
    chg_type = type; 
	if(bqdi == NULL){
		pr_err("%s:bqdi is NULL,ignore.\n",__func__);
		return;
	}
	if (!wake_lock_active(&bqdi->wakelock))
		wake_lock(&bqdi->wakelock);
	__cancel_delayed_work(&bqdi->work);
	schedule_delayed_work(&bqdi->work, msecs_to_jiffies(2000));//2s
}

int bq24192_update_ovp_state(int usb_health)
{
	if(NULL == bqdi){
		return -1;
	}
	printk("update_ovp state=%d.\n",usb_health);
	ovp_check_cnt = 0;
	bqdi->usb_ovp = usb_health;
	schedule_delayed_work(&bqdi->ovp_work, msecs_to_jiffies(500));//500ms
	
	return 0;
}

static int bq24192_get_vbus_state(struct bq27x00_device_info *di)
{
	int ret;

	ret = bq27x00_read(di, BQ24192_REG8_STATUS, true);
	if(ret < 0){
		printk("read charger status err:0x%x.\n",ret);
		return USBIN_UNKNOW;
	}

	return (ret>>6);
}

static int bq24192_get_usb_health(struct bq27x00_device_info *di)
{
	int ret;

	ret = bq27x00_read(di, BQ24192_REG8_STATUS, true);
	if(ret < 0){
		printk("read charger status err:0x%x.\n",ret);
		return USBIN_UNKNOW;
	}

	if(ret & 0x08){
		ret = USBIN_DPM;
	}else{
		ret = USBIN_OK;
	}

	return ret;
}

static int bq27x00_read_i2c(struct bq27x00_device_info *di, u8 reg, bool single)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[2];
	int ret,i;

	if (!client->adapter)
		return -ENODEV;

	if (di->is_rom_mode)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	if (single)
		msg[1].len = 1;
	else
		msg[1].len = 2;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0){
		printk("%s: 1reg:0x%x, err:0x%x.\n", __func__, reg, ret);
		for(i=0;i<3;i++){
			ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
			if(ret > 0)
				goto ret_true;
		}
		return ret;
	}
ret_true:
	if (!single)
		ret = get_unaligned_le16(data);
	else
		ret = data[0];
	return ret;
}

static int bq27x00_write_i2c(struct bq27x00_device_info *di, u8 reg, int num, unsigned char *buf,int rom_mode)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[1];
    unsigned char *data;
    int i = 0;
	int ret = 0;

	if (!client->adapter)
		return -ENODEV;
    
	if (!rom_mode && di->is_rom_mode)
		return -ENODEV;

	data = kzalloc(sizeof(char) * (num+1), GFP_KERNEL);
    
    data[0] = reg;
    for(i=0;i<num;i++)
        data[i+1] = buf[i];
    

	if(rom_mode)
		msg[0].addr = 0x0b;
	else
		msg[0].addr = client->addr;

	msg[0].flags = 0;
	msg[0].buf = data;
	msg[0].len = num+1;
    /*printk("%s: i2c addr0 is 0x%x, addr1=0x%x \n",__func__,msg[0].addr,msg[1].addr);*/

    ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0)
		printk("%s: reg:0x%x, err:0x%x.\n", __func__, reg, ret);
    /*ret = i2c_master_send(client, data, num+1);*/
    
    kfree(data);

	return ret;
}

//extern void popup_usb_select_window(int popup);
static void bq24192_set_chrg_type(struct bq27x00_device_info *di)
{

    di->chrg_type = chg_type;
    switch(di->chrg_type){
        case POWER_SUPPLY_TYPE_USB_CDP:
        case POWER_SUPPLY_TYPE_USB_ACA:
        case POWER_SUPPLY_TYPE_USB_DCP:
        case POWER_SUPPLY_TYPE_UNKNOWN:
            di->mains_online = 1;
            di->usb_online = 0;
            break;
        case POWER_SUPPLY_TYPE_USB:
            di->usb_online = 1;
            di->mains_online = 0;
            break;
        case POWER_SUPPLY_TYPE_BATTERY:
            di->usb_online = 0;
	    di->mains_online = 0;
#if 0
	    if(bq24192_is_usbin_present() == 0)
		    popup_usb_select_window(2);
#endif
	    break;
	default:
	    printk("%s:defalt chg_type",__func__);
	    break;
    } 
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */

static int chrg_type_double_check(struct bq27x00_device_info *di)
{
	di->usb_present = bq24192_is_usbin_present();
	if(!di->usb_present){
		di->chrg_type = POWER_SUPPLY_TYPE_BATTERY;
		chg_type = POWER_SUPPLY_TYPE_BATTERY;
		di->usb_online = 0;
		di->mains_online = 0;
	}
	DBG("%s,mains_online=%d,usb_online=%d,chrg_type=%d.\n",__func__,di->mains_online,di->usb_online,di->chrg_type);
	return 0;
}
static int bq27x00_battery_read_temp(struct bq27x00_device_info *di)
{
	int temp;

	temp = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	temp = temp - BQ_BATT_TEMP_OFFSET;
	printk("bq275x %s temp: %d\n", __func__, temp);

	if(temp - 2731 < 580){
		temp_prev = temp;
		htemp_cnt = 0;
	}else if(temp - 2731 >= 580){
		htemp_cnt++;
		if(htemp_cnt <= 3){
			dev_info(di->dev,"High temp detect %dC %d times!\n",temp,htemp_cnt);
			temp = temp_prev;
		}else{
			dev_info(di->dev,"High temp detect:%dC %d times,report it!\n",temp,htemp_cnt);
			temp_prev = temp;
			htemp_cnt = 0;
		}
	}

	return temp;
}

static int bq27x00_battery_read_rsoc(struct bq27x00_device_info *di)
{
	int rsoc = -1;
	int volt = -1;
	int chrg_status,chrg_term_tim,chrg_term_curr;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	rsoc = bq27x00_read(di, BQ27500_REG_SOC, false);
	if (rsoc < 0 || volt < 0 )
			dev_err(di->dev, "error reading soc %d or volt %d\n",rsoc,volt);

	//for calculate soc
	if(rsoc >= 0 && rsoc <= 100){
		if(rsoc != last_cache_soc)
			delta_soc = rsoc - last_cache_soc;
		DBG("%s:delta_soc=%d,rsoc=%d,last_cap=%d.\n",
			__func__,delta_soc,rsoc,last_cache_soc);
		last_cache_soc = rsoc;
	}

	chrg_status = bq27x00_read(di, BQ24192_REG8_STATUS,true);
	if(di->mains_online == 1 || di->usb_online == 1){
		if((chrg_status >> 4 & 0x3) == 0x3){
			if(rsoc != 100){
				rsoc = 100;
				chrg_term_curr = bq27x00_read(di,0x78,true);
				chrg_term_tim = bq27x00_read(di,0x7A,true);
				dev_err(di->dev,"0x7D is 0x%x,0x78 is 0x%x,0x7A is 0x%x!\n",
						chrg_status,chrg_term_curr,chrg_term_tim);
			}
		}
	}
	if(rsoc >= 0 && rsoc <= 100){
		soc_prev = rsoc;
		rd_count = 0;
	}else{
		if(rd_count >= 9){
			rd_count = 0;
			dev_info(di->dev,"Low soc %d detect!\n",rsoc);
		}else{
			rd_count++;
			dev_info(di->dev,"Low soc %d prev %d loop %d!\n",rsoc,soc_prev,rd_count);
			rsoc = soc_prev;
		}
	}
	return rsoc;
}
/*
 * Return the battery Cycle count total
 * Or < 0 if something fails.
 */
static int bq27x00_battery_read_cyct(struct bq27x00_device_info *di)
{
	int cyct;

	cyct = bq27x00_read(di, BQ27x00_REG_CYCT, false);
	if (cyct < 0)
		dev_err(di->dev, "error reading cycle count total\n");

	return cyct;
}

static void usb_switch_to_uart(struct bq27x00_device_info *di, bool enable)
{
    int val = enable;    

    if(di->board->usb_sw_gpio <= 0){
       printk("di->board->usb_sw_gpio is < 0.\n"); 
       return;
    }

	printk("%s,switch to %s.\n",__func__,val?"uart":"usb_psy");
	gpio_set_value_cansleep(di->board->usb_sw_gpio, val);
    
    if(val){
			switch_to_uart = 1;
    }else{
			switch_to_uart = 0;
    }
    
}

static void bq27530_set_current(struct bq27x00_device_info *di, enum current_level cl)
{

    int val = !cl;

    if(di->board->chg_psel_gpio <= 0){
       printk("di->board->chg_psel_gpio is < 0.\n"); 
       return;
    }
        
	/*printk("%s:chg_psel_gpio %d,cl_now:%d,cl_to_set:%d.\n",__func__,di->board->chg_psel_gpio,*/
						/*di->board->chg_current,cl);*/

    if(di->board->chg_current != cl){
			gpio_set_value_cansleep(di->board->chg_psel_gpio, val);
			di->board->chg_current = cl;
			printk("%s: bq27530 set  %s .\n", __func__, cl ? "current high":"current low");
    }

    return;
}
/*enable/disable charger IC get current from usb/charger*/
static void bq27530_enable_charging(struct bq27x00_device_info *di, bool enable)
{

  	int val = !enable;
	int batt_present = 1;
	int last_charging_state;
#if 1
	unsigned char data;
#endif
	int i,ret;
	bool enable_flag = enable;

	printk("%s enable=%d\n", __func__, enable);

    if(di == NULL){
       printk("di is NULL.\n"); 
       return;
    }

    if(di->board->chg_en_gpio <= 0){
       printk("di->board->chg_en_gpio is < 0.\n"); 
       return;
    }

	if(di->usb_ovp == USBIN_OVP && enable){
		printk("usb_vop:ignore enable charging.\n");
		return;
	}

	last_charging_state = di->board->chg_en_flag;
	/* optimistically set the state, change it back later if one of the corner
	 * cases is triggered. Userspace expects this to be fast and the i2c methods
	 * below take time which might cause charger mode to abort. */
        di->board->chg_en_flag = enable;

#ifdef SUPPORT_QPNP_VBUS_OVP
	di->vbus_ovp = qpnp_check_vbus_ovp(&vusb_uv);
	printk("%s: vbus is %d\n", __func__, vusb_uv);
	if(di->vbus_ovp && enable) {
		di->board->chg_en_flag = last_charging_state;
		printk("vbus_vop:ignore enable charging.\n");
		return;
	}
#endif

	ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	batt_present = (ret & BQ27500_FLAG_BAT_DET)>0 ? 1:0;
	
	if(!batt_present){//battery is not present
		di->board->chg_en_flag = last_charging_state;
		printk("%s:battery is not present.\n",__func__);
		enable_flag = false;
		val = !enable_flag;
	}
        
    if(!disable_charging_force){
        /*printk("bq27530 set enable gpio %s .\n",enable_flag ? "enabled":"disabled");*/
        /*gpio_set_value_cansleep(di->board->chg_en_gpio, val);*/
        di->board->chg_en_flag = enable_flag; 
        /*printk("%s HIZ mode.\n",enable_flag?"disable":"enable");*/
        for(i=0;i<5;i++){
            ret = bq27x00_read_i2c(di, BQ24192_REG_CTL_0, 1);
            if(ret > 0)
                break;
        }
        printk("%s: read hiz_reg=0x%x,i=%d.\n",__func__,ret,i);
#if 1
        if(ret < 0){
            pr_err("bq27530 read charger reg00 err.\n");
        }else{
            data = ret & 0xff;
            if(enable_flag)
                data &= ~BQ24192_0_HIZ;
            else
                data |= BQ24192_0_HIZ;
            for(i=0;i<5;i++){
                ret = bq27x00_write_i2c(di,BQ24192_REG_CTL_0,1,&data,0);
                if(ret>0)
                    break;
            }
            printk("%s: write hiz_reg=0x%x,ret=%d,i=%d.\n",__func__,data,ret,i);
            if(ret <0)
                pr_err("%s: HIZ mode write error\n",__func__);
        }
#endif
#ifdef CONFIG_FB
        if(enable_flag) {
		if (fb_status == FB_BLANK_UNBLANK)
			bq27531_config_charging_current(di, 0);
		else if (fb_status == FB_BLANK_POWERDOWN)
			bq27531_config_charging_current(di, 1);
	}
#endif
    }else{
      di->board->chg_en_flag = last_charging_state;
      ret = bq27x00_read_i2c(di, BQ24192_REG_CTL_0, 1);
			printk("wyh chg disable_force,hiz_reg=%d .\n",ret);
		}
    return;
}

static void fg_reg_show(struct bq27x00_device_info *di)
{

	unsigned char data[2]={0};
	int ret,fg_cont,fg_rm_cap,fg_fc_cap,fg_pc_volt,fg_cc_volt,nac,
	fac,rcuf,rcf,tc,flags,temp,current_now,voltage,fccu,fccf,soc;
	int chg_status,ctrl,chrg_por,current_chg,chrg_volt,status,chrg_fault, max_capa;

	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error\n",__func__);
	}
	mdelay(50);
	nac = bq27x00_read(di,BQ27x00_REG_NAC, false);//nominal available capacity
	fac = bq27x00_read(di,BQ27x00_REG_FAC, false);//full available capacity
	rcuf = bq27x00_read(di,0x18, false);//remain capacity unflittered
	rcf = bq27x00_read(di,0x1C, false);//remain capacity flittered
	fccu = bq27x00_read(di,0x22, false);//remain capacity flittered
	tc = bq27x00_read(di,0x2E, false);//true capacity
	fccf = bq27x00_read(di,0x26, false);//full charger capacity flittered
	soc = bq27x00_read(di,0x2c, false);//state of charge
	flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	tc = bq27x00_read(di,0x2E, false);//true capacity
	temp = bq27x00_read(di, BQ27x00_REG_TEMP, false);
	temp = temp - BQ_BATT_TEMP_OFFSET;
	current_now = bq27x00_read(di, BQ27x00_REG_AI, false);
	voltage = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	max_capa = bq27x00_read(di, 0x62, false);

	fg_cont = bq27x00_read(di,BQ27530_REG_CNTL, false);
	fg_rm_cap = bq27x00_read(di,BQ27530_REG_RM,false);
	fg_fc_cap = bq27x00_read(di,BQ27530_REG_FCC,false);
	fg_pc_volt = bq27x00_read(di,BQ27500_REG_PCV,false);
	fg_cc_volt = bq27x00_read(di,BQ27530_REG_CCV,false);

    chg_status = bq27x00_read_i2c(di, BQ24192_REG_STAT, 1); //0x74
    ctrl = bq27x00_read_i2c(di, BQ24192_REG_CTL_0, 1);//0x75
	chrg_por = bq27x00_read(di, BQ24192_REG_POR_1,true);//0x76
	current_chg = bq27x00_read(di, BQ24192_REG_CURRENT,true);//0x77
	chrg_volt = bq27x00_read(di, BQ24192_REG_VOLT_4,true);
	status = bq27x00_read(di, BQ24192_REG8_STATUS, true);//0x7d
	chrg_fault = bq27x00_read(di, BQ24192_REG9_FAULT,true);//0x7e
	dev_info(di->dev,"charger chrg_status=0x%x,ctrl=0x%x,chrg_por=0x%x,curr=0x%x,volt=0x%x,status=0x%x,fault=0x%x.\n",
		chg_status,ctrl,chrg_por,current_chg,chrg_volt,status,chrg_fault);
	dev_info(di->dev,"nac=0x%x,fac=0x%x,rcuf=0x%x,rcf=0x%x,fccu=0x%x,fccf=0x%x.\n soc=%d,flags=0x%x,tc=0x%x,temp=%d,current_now=%d,voltage=%d,max_capa=%d.\n",
					nac,fac,rcuf,rcf,fccu,fccf,soc,flags,tc,temp,current_now,voltage,max_capa);
}

static void bq27x00_update(struct bq27x00_device_info *di)
{
	struct bq27x00_reg_cache cache = {0, };

	cache.flags = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (cache.flags >= 0) {
		cache.capacity = bq27x00_battery_read_rsoc(di);
		cache.temperature = bq27x00_battery_read_temp(di);
		cache.cycle_count = bq27x00_battery_read_cyct(di);
		cache.current_now = bq27x00_read(di, BQ27x00_REG_AI, false);
		cache.voltage = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	}

  	bq24192_set_chrg_type(di);
	chrg_type_double_check(di);

	if(di->mains_online){
	   bq27530_set_current(di,CURRENT_HIGH);
	}else{
	   bq27530_set_current(di,CURRENT_LOW);
	}

	/* Ignore current_now which is a snapshot of the current battery state
	 * and is likely to be different even between two consecutive reads */
	if (memcmp(&di->cache, &cache, sizeof(cache) - sizeof(int)) != 0) {
		di->cache = cache;
		power_supply_changed(&di->bat);
	}

	di->last_update = jiffies;
	dev_info(di->dev, "usb_p=%d,host_m=%d,chg_t=%d,cap=%d,temp=%d,curr=%d,vol=%d,flag=0x%x,cl=%d\n",
			di->usb_present,di->host_mode,di->chrg_type,cache.capacity,(cache.temperature - 2731),
			(int)((s16)cache.current_now),cache.voltage,cache.flags,
			di->board->chg_current);
	fg_reg_show(di);
}

#ifdef SUPPORT_QPNP_VBUS_OVP
static void bq27x00_charger_vbus_check(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, vbus_work.work);
	int ret;

	di->usb_ovp = bq24192_get_usbin_health();
	di->vbus_ovp = qpnp_check_vbus_ovp(&vusb_uv);

	//printk("%s usb_health =%d, vbus_ovp=%d.\n",__func__,di->usb_ovp,di->vbus_ovp);

	ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	if (!(ret & BQ27500_FLAG_BAT_DET)) {
		power_supply_changed(&di->bat);
		return;
	}

	if (di->usb_ovp == USBIN_OK) {
        	if (di->board->chg_en_flag != (!di->vbus_ovp)) {
			if (di->vbus_ovp) {
				pr_err("%s, vbus ovp, stop charging\n", __func__);
				bq27530_enable_charging(di,false);
			} else {
				pr_err("%s, vbus ok, start charging\n", __func__);
				bq27530_enable_charging(di,true);
			}
			power_supply_changed(&di->bat);
		}
		schedule_delayed_work(&di->vbus_work, msecs_to_jiffies(1000));
	}

	return;
}
#endif

static void bq27x00_charger_ovp(struct work_struct *work)
{
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, ovp_work.work);

	di->usb_ovp = bq24192_get_usbin_health();
#ifdef SUPPORT_QPNP_VBUS_OVP
	di->vbus_ovp = qpnp_check_vbus_ovp(&vusb_uv);
	printk("%s usb_health =%d,check_cnt=%d,vbus=%d,vbus_ovp=%d.\n",__func__,di->usb_ovp,ovp_check_cnt,vusb_uv,di->vbus_ovp);
	if((di->usb_ovp == USBIN_OVP) || (di->usb_ovp == USBIN_UNKNOW) || di->vbus_ovp){
#else
	DBG("%s usb_health = %d,check_cnt=%d.\n",__func__,di->usb_ovp,ovp_check_cnt);
	if(di->usb_ovp == USBIN_OVP){
#endif
		bq27530_enable_charging(di,false);
	} else if(di->usb_ovp == USBIN_OK){
		bq27530_enable_charging(di,true);
	}

	ovp_check_cnt++;

	if(ovp_check_cnt < 10){
		schedule_delayed_work(&di->ovp_work, msecs_to_jiffies(500));
#ifdef SUPPORT_QPNP_VBUS_OVP
	} else {
		if (di->usb_ovp)
			schedule_delayed_work(&di->vbus_work, msecs_to_jiffies(500));
		else
			cancel_delayed_work(&di->vbus_work);
#endif
	}
	return;
}

static void bq27x00_battery_poll(struct work_struct *work)
{
	int vbus_state;
#if 0
	int ret;
#endif
	struct bq27x00_device_info *di =
		container_of(work, struct bq27x00_device_info, work.work);

	dev_info(di->dev, "fg event worker\n");

	di->usb_ovp = bq24192_get_usbin_health();
	if (di->usb_ovp == USBIN_OK) {
		vbus_state = bq24192_get_vbus_state(di);
		if(!vbus_state || di->cache.voltage < 3500){
			bq27530_enable_charging(di,true);
		}
	}
		
	bq27x00_update(di);

#if 0
	if(di->cache.voltage < 4050){
		if((di->cache.temperature-2731) > 0 && (di->cache.temperature-2731) < 100){
			ret = bq27531_data_flash_write(di,0x4a,6,20);//T0
			if(ret < 0)
				pr_err("%s:write err ret=%d.\n",__func__,ret);
		} else if((di->cache.temperature-2731) >= 100 && (di->cache.temperature-2731) < 230){
			ret = bq27531_data_flash_write(di,0x4a,7,50);//T1
			if(ret < 0)
				pr_err("%s:write err ret=%d.\n",__func__,ret);
		}
	}else if(di->cache.voltage >= 4050){
		if((di->cache.temperature-2731) > 0 && (di->cache.temperature-2731) < 100){
			ret = bq27531_data_flash_write(di,0x4a,6,15);//T0
			if(ret < 0)
				pr_err("%s:write err ret=%d.\n",__func__,ret);
		} else if((di->cache.temperature-2731) >= 100 && (di->cache.temperature-2731) < 230){
			ret = bq27531_data_flash_write(di,0x4a,7,30);//T1
			if(ret < 0)
				pr_err("%s:write err ret=%d.\n",__func__,ret);
		}
	}
#endif

	dev_info(di->dev, "%s:chg_en=%d,chg-type=%d,mains_online=%d,usb_online=%d,usb_ovp=%d,chg_st=%d.\n",
		__func__,di->board->chg_en_flag,di->chrg_type,di->mains_online,di->usb_online,di->usb_ovp,di->chg_state);
	if(di->mains_online || di->usb_online || (di->usb_ovp == USBIN_OVP)){//usb present
		if (!wake_lock_active(&di->wakelock)) {
			printk("%s: add wake lock in main or usb online\n", __func__);
			wake_lock(&di->wakelock);
		}
#ifdef CONFIG_FB
		if (fb_status == FB_BLANK_UNBLANK)
			bq27531_config_charging_current(di, 0);
		else if (fb_status == FB_BLANK_POWERDOWN)
			bq27531_config_charging_current(di, 1);
		schedule_delayed_work(&di->work, msecs_to_jiffies(20000));//20s
#else
		schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t));//30s
#endif
	}else{
		if(wake_lock_active(&di->wakelock))
			wake_unlock(&di->wakelock);
		if((di->cache.temperature-2731)>580)
			schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t/3));//10s
		else
			schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t));//30s
	}
}


/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	if (di->cache.temperature < 0)
		return di->cache.temperature;

#if 0
	if(((di->cache.temperature <= (2381 - BQ_BATT_TEMP_OFFSET)) && (force_check_thermal != 1)))
            val->intval = 255;
        else
            val->intval = di->cache.temperature - 2731;
#else
            val->intval = di->cache.temperature - 2731;
#endif

	return 0;
}

/*
 * Return the battery average current in µA
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int curr;
	curr = bq27x00_read(di, BQ27x00_REG_AI, false);
	val->intval = (int)((s16)curr);
	val->intval *= -1000;

	return 0;
}
static int bq27x00_battery_health(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int ret;

	if (di->cache.temperature < 0){
		dev_err(di->dev, "battery pack temp read fail\n");
		return val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}
	if((di->cache.temperature - 2731) > BATT_TEMP_MAX){
		dev_err(di->dev, "battery temp over head:%d\n",
					di->cache.temperature - 2731);
		return val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	ret = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if(ret < 0){
		dev_err(di->dev, "battery pack voltage read fail\n");
		return val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	}
	if(ret > BATT_VOLT_MAX){
		dev_err(di->dev, "Battery Over Voltage condition Detected:%d\n",ret);
		return val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	}
	if(ret < BATT_VOLT_MIN){
		dev_err(di->dev, "Low Battery condition Detected:%d\n",ret);
		return val->intval = POWER_SUPPLY_HEALTH_DEAD;
	}
	return val->intval = POWER_SUPPLY_HEALTH_GOOD;
}

static int bq27x00_battery_status(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int batt_temp,chrg_stat,ret;

	if(di->usb_ovp == USBIN_OVP){
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}
	bq24192_set_chrg_type(di);
	chrg_stat = bq27x00_read(di,BQ24192_REG8_STATUS, true);

	DBG("%s:usb=%d,mains=%d,chg_t=%d.\n",__func__,di->usb_online,di->mains_online,di->chrg_type);
	/*when battery capacity fall to 99,recharging begin*/
	if(di->usb_online || di->mains_online){
#ifdef SUPPORT_QPNP_VBUS_OVP
		if (di->vbus_ovp)
			//val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else {
			if(last_report_soc == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
	} else {
		/*when usb is otg mode or usb is plug off*/
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}
#else
		if(last_report_soc == 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}else{
		/*when usb is otg mode or usb is plug off*/
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
#endif
	batt_temp = di->cache.temperature - 2731;
	//if(batt_temp <=0 || batt_temp >=500){
	//	pr_err("discharging:temperatur abnormal or on_init, batt_temp=%d\n", batt_temp);
	//	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	//}else if(batt_temp <= 20 || batt_temp >= 480){
	if(batt_temp <= 20 || batt_temp >= 480){
		//msleep(3000);
		ret = bq27x00_read(di, BQ24192_REG8_STATUS, true);
		printk("reg8_status=0x%x.\n",ret);
		if((ret >> 4 & 0x3) == 0x0){
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}else if((ret >> 4 & 0x3) == 0x3){
			val->intval = POWER_SUPPLY_STATUS_FULL;
		}else {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
	}
	di->chg_state = val->intval;

	return 0;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di,
		union power_supply_propval *val)
{
	int volt;

	volt = bq27x00_read(di, BQ27x00_REG_VOLT, false);
	if (volt < 0)
		return volt;

	val->intval = volt * 1000;

	return 0;
}

static irqreturn_t charge_irq(int irq, void *devid)
{
	int chrg_fault = 0;
	int chrg_status = 0;
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)devid;
	dev_info(di->dev, "CHARGER ISR\n");
	chrg_fault = bq27x00_read(di, BQ24192_REG9_FAULT,true);
	chrg_status = bq27x00_read(di, BQ24192_REG8_STATUS, true);
	dev_info(di->dev,"Chrg fault=0x%x,status=0x%x.\n",chrg_fault,chrg_status);
	if (!wake_lock_active(&di->wakelock))
		wake_lock(&di->wakelock);
	__cancel_delayed_work(&di->work);
	schedule_delayed_work(&di->work, msecs_to_jiffies(2000));//2s
	ovp_check_cnt = 0;
	__cancel_delayed_work(&di->ovp_work);
	schedule_delayed_work(&di->ovp_work, msecs_to_jiffies(500));//500ms
#ifdef SUPPORT_QPNP_VBUS_OVP
	__cancel_delayed_work(&di->vbus_work);
	schedule_delayed_work(&di->vbus_work, msecs_to_jiffies(500));//500ms
#endif
	return IRQ_HANDLED;
}

static irqreturn_t fg_irq(int irq, void *devid)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t fg_thread_handler(int irq, void *devid)
{
	struct bq27x00_device_info *di = (struct bq27x00_device_info *)devid;
	dev_info(di->dev, "FG ISR\n");
	if (!wake_lock_active(&di->wakelock))
		wake_lock(&di->wakelock);
	__cancel_delayed_work(&di->work);
	schedule_delayed_work(&di->work, msecs_to_jiffies(2000));//2s

	return IRQ_HANDLED;
}

static int bq27x00_simple_value(int value,
		union power_supply_propval *val)
{
	if (value < 0)
		return value;

	val->intval = value;

	return 0;
}

#define to_bq27x00_device_info(x) container_of((x), \
		struct bq27x00_device_info, bat);

static int bq27x00_batt_property_is_writeable(struct power_supply *psy,
                                    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
}

static int bq27x00_battery_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

    switch(psp){
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
        if(di->board->chg_en_flag != val->intval){
                if(val->intval){
                    disable_charging_force = 0;
                    bq27530_enable_charging(di,val->intval);
                }else{
                    bq27530_enable_charging(di,val->intval);
                    disable_charging_force = 1;
                }
        }
        break;
    default:
        return -EINVAL;
    }
    
    return 0;
}

static int bq27x00_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0, batt_present;
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	if (di->is_suspend) {
		printk("%s: suspend = %d\n", __func__, di->is_suspend);
		return -ENODEV;
	}

	if (psp != POWER_SUPPLY_PROP_PRESENT && di->cache.flags < 0)
		return -ENODEV;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			ret = bq27x00_battery_status(di, val);
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = bq27x00_battery_voltage(di, val);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
				val->intval = di->board->chg_en_flag;
				break;
		case POWER_SUPPLY_PROP_PRESENT:
			ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
			val->intval = (ret & BQ27500_FLAG_BAT_DET)>0 ? 1:0;
			if (!val->intval)
				printk("%s:battery is not present. ret=0x%x\n",__func__, ret);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = bq27x00_battery_current(di, val);
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			if(((di->cache.temperature <= (2381 - BQ_BATT_TEMP_OFFSET)) && (force_check_thermal != 1)) || di->is_rom_mode || !di->fw_dw_done || di->soc_reset)
				val->intval = 50;
			else{
				ret = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
				batt_present = (ret & BQ27500_FLAG_BAT_DET)>0 ? 1:0;
				if(!batt_present){//battery is not present
					printk("%s:2battery is not present. check_count=%d, ret=0x%x\n",
							__func__, di->present_check_count, ret);
					if (di->present_check_count > 5)
						val->intval = 0;
					else {
						calculate_report_soc(di);
						ret = bq27x00_simple_value(di->cache.capacity, val);
						di->present_check_count++;
					}
				}else{
					calculate_report_soc(di);
					ret = bq27x00_simple_value(di->cache.capacity, val);
					di->present_check_count = 0;
				}
			}
			break;
		case POWER_SUPPLY_PROP_TEMP:
			ret = bq27x00_battery_temperature(di, val);
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			ret = bq27x00_battery_health(di,val);
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			ret = bq27x00_simple_value(di->cache.cycle_count, val);
			break;
		default:
			return -EINVAL;
	}
	return ret;
}

int fw_ver_get(struct bq27x00_device_info *di)
{
	int ret;
	unsigned char data[2]={0};

    data[0]=0x02;
    data[1]=0x00;

	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error %d\n",__func__,ret);
	}

	mdelay(50);
	ret = bq27x00_read(di,BQ27530_REG_CNTL, false);
	printk("%s: FW ver ret is 0x%x\n",__func__,ret);

	return ret;
}

int df_ver_get(struct bq27x00_device_info *di)
{

	int ret;
	unsigned char data[2]={0};

	data[0]=0x1F;
	data[1]=0x00;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
	if(ret <0){
		printk("%s: status cmd write error\n",__func__);
	}

	mdelay(50);
	ret = bq27x00_read(di,BQ27530_REG_CNTL, false);
	//printk("%s: DF ver ret is 0x%x\n",__func__,ret);

	return ret;
}

static int fg_version_get(struct bq27x00_device_info *di)
{

	int ret;
	unsigned char data[2]={0};

    data[0]=0x01;
    data[1]=0x00;

    ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);
    if(ret <0){
        printk("%s: status cmd write error %d\n",__func__,ret);
    }

    mdelay(50);
    ret = bq27x00_read(di,BQ27530_REG_CNTL, false);
    
    return ret;

}

static int chrg_get_vendor(struct bq27x00_device_info *di)
{
	int ret;
	ret = bq27x00_read(di, BQ24192_REG_DEV_VER,1);
	if(ret <0){
		printk("%s:REG 0x%x read error code = 0x%x\n",__func__,BQ24192_REG_DEV_VER,ret);
		return ret;
	}

	ret = (ret >> 3) & 0x07;
    
    return ret;
}

static char* bq24192_get_vendor(struct bq27x00_device_info *di)
{
	int ret;
	ret = bq27x00_read(di, BQ24192_REG_DEV_VER,1);
	if(ret <0){
		printk("%s:REG 0x%x read error code = 0x%x\n",__func__,BQ24192_REG_DEV_VER,ret);
		memcpy(di->chrg_vendor, "NULL", sizeof("NULL"));
		return di->chrg_vendor;
	}
	ret = (ret >> 3) & 0x07;
	if(ret == BQ24190_IC_VERSION){
		memcpy(di->chrg_vendor, "bq24190", sizeof("bq24190"));
		return di->chrg_vendor;
	}else if(ret == BQ24192_IC_VERSION){
		memcpy(di->chrg_vendor, "bq24192", sizeof("bq24192"));
		return di->chrg_vendor;
	}else if(ret == BQ24292I_IC_VERSION){
		memcpy(di->chrg_vendor, "bq24292i", sizeof("bq24292i"));
		return di->chrg_vendor;
	}else{
		memcpy(di->chrg_vendor, "Unknown", sizeof("Unknown"));
		return di->chrg_vendor;
	}
}

static int bq24912_mains_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq27x00_device_info *di =
		container_of(psy, struct bq27x00_device_info, mains);
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		bq24192_set_chrg_type(di);
		val->intval = di->mains_online;
		return 0;
	}else if (psp == POWER_SUPPLY_PROP_MANUFACTURER){
		val->strval = bq24192_get_vendor(di);
		return 0;
	}else if(psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = bq24192_get_usb_health(di);
		return 0;
	}
	return EINVAL;
}

static int bq24912_usb_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct bq27x00_device_info *di =
		container_of(psy, struct bq27x00_device_info, usb);
	if (psp == POWER_SUPPLY_PROP_ONLINE) {
		bq24192_set_chrg_type(di);
		val->intval = di->usb_online;
		return 0;
	}else if (psp == POWER_SUPPLY_PROP_MANUFACTURER){
		val->strval = bq24192_get_vendor(di);
		return 0;
	}else if(psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = bq24192_get_usb_health(di);
		return 0;
	}
	return EINVAL;
}

static void bq27x00_boot_work(struct work_struct *work)
{
	if (force_check_thermal == -1)
		force_check_thermal = 1;
	boot_done = 1;

	printk("%s: boot_done=%d, force_check_thermal=%d\n", __func__, boot_done, force_check_thermal);
	return;
}

static int bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	int ret;
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.set_property = bq27x00_battery_set_property;
    di->bat.property_is_writeable = bq27x00_batt_property_is_writeable;

	INIT_DELAYED_WORK(&di->work, bq27x00_battery_poll);
	INIT_DELAYED_WORK(&di->ovp_work, bq27x00_charger_ovp);
#ifdef SUPPORT_QPNP_VBUS_OVP
	INIT_DELAYED_WORK(&di->vbus_work, bq27x00_charger_vbus_check);
#endif
	INIT_DELAYED_WORK(&di->boot_work, bq27x00_boot_work);
	mutex_init(&di->lock);
	wake_lock_init(&di->wakelock, WAKE_LOCK_SUSPEND,
						"ctp_charger_wakelock");
	wake_lock_init(&di->lowcap_wakelock, WAKE_LOCK_SUSPEND,
						"ctp_lowcap_wakelock");

	ret = power_supply_register(di->dev, &di->bat);
	if (ret) {
		dev_err(di->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	di->mains.name = "ac";
	di->mains.type = POWER_SUPPLY_TYPE_MAINS;
	di->mains.properties = bq24912_mains_props;
	di->mains.num_properties = ARRAY_SIZE(bq24912_mains_props);
	di->mains.get_property = bq24912_mains_get_property;
	ret = power_supply_register(di->dev, &di->mains);
	if (ret) {
		dev_err(di->dev, "failed to register mains: %d\n", ret);
		return ret;
	}
	di->usb.name = "usb_bq24912";
	di->usb.type = POWER_SUPPLY_TYPE_USB;
	di->usb.properties = bq24912_usb_props;
	di->usb.num_properties = ARRAY_SIZE(bq24912_usb_props);
	di->usb.get_property = bq24912_usb_get_property;
	ret = power_supply_register(di->dev, &di->usb);
	if (ret) {
		dev_err(di->dev, "failed to register usb: %d\n", ret);
		return ret;
	}

	dev_info(di->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	bq27x00_update(di);

	return 0;
}

static void bq27x00_powersupply_unregister(struct bq27x00_device_info *di)
{
	cancel_delayed_work_sync(&di->work);

	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->mains);
	power_supply_unregister(&di->usb);

	mutex_destroy(&di->lock);
}


/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

static u32 bq27530_read_i2c_4byte(struct bq27x00_device_info *di, u8 reg,int num)
{
	struct i2c_client *client = to_i2c_client(di->dev);
	struct i2c_msg msg[2];
	unsigned char data[4]={0};
	u32 ret;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = 0x0B;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = sizeof(reg);
	msg[1].addr = 0x0B;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;
	msg[1].len = num;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		printk("%s: reg:0x%x, err:0x%x.\n", __func__, reg, ret);
		return ret;
	}
	ret = get_unaligned_le32(data);

	return ret;
}

int firmware_write(struct bq27x00_device_info *di)
{
	int i,j=0,t,offset,size;
	unsigned char data[32]={0};
	struct bqfs *bq;
	u32 ret;
	u32 fw_data;
	unsigned char reg;
	int bqfs_size = 0;
	int retry=0;
	int fw_update_err=0;
	BQFS *bqfs_index = NULL;
	unsigned char *firmware_data = NULL;

  di->chg_pn = chrg_get_vendor(di); 
	if(di->chg_pn == BQ24192_IC_VERSION){
		bqfs_index = bq24192_bqfs_index;
		firmware_data = bq24192_firmware_data;
		bqfs_size = sizeof(bq24192_bqfs_index)/sizeof(*bq);
	} else if(di->chg_pn == BQ24292I_IC_VERSION){
		if(di->battery_vendor_index == BATT_VENDOR_ATL_AND_DUMY) {
			dev_info(di->dev, "FG update fw to 0x%x\n", BATT_FW_VER_ATL);
			bqfs_index = bq24292i_bqfs_index_atl;
			firmware_data = bq24292i_firmware_data_atl;
			bqfs_size = sizeof(bq24292i_bqfs_index_atl)/sizeof(*bq);
		}else{
			dev_info(di->dev, "FG update fw to 0x%x\n", BATT_FW_VER_LG);
			bqfs_index = bq24292i_bqfs_index_lg;
			firmware_data = bq24292i_firmware_data_lg;
			bqfs_size = sizeof(bq24292i_bqfs_index_lg)/sizeof(*bq);
		}
	}else{
		pr_err("%s:chg_pn %d,use default.\n",__func__,di->chg_pn);
		if(di->battery_vendor_index == BATT_VENDOR_ATL_AND_DUMY) {
			dev_info(di->dev, "FG update fw to 0x%x\n", BATT_FW_VER_ATL);
			bqfs_index = bq24292i_bqfs_index_atl;
			firmware_data = bq24292i_firmware_data_atl;
			bqfs_size = sizeof(bq24292i_bqfs_index_atl)/sizeof(*bq);
		}else{
			dev_info(di->dev, "FG update fw to 0x%x\n", BATT_FW_VER_LG);
			bqfs_index = bq24292i_bqfs_index_lg;
			firmware_data = bq24292i_firmware_data_lg;
			bqfs_size = sizeof(bq24292i_bqfs_index_lg)/sizeof(*bq);
		}
	}
	
	dev_info(di->dev, "bq27530 %s:chg_pn %d,bqfs_size=%d\n", __func__,di->chg_pn,bqfs_size);

	for(retry=0;retry<3;retry++){
		data[0]=0x00;
		data[1]=0x0f;
		ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);//Enter ROM Mode
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
			return -1;
		}else{
			printk("%s: reg 0x00 enter rom mode whitout com error\n",__func__);
			di->is_rom_mode = 1;
			di->fw_dw_done = 0;
		}
		mdelay(1000);
		fw_update_err = 0;
		for(i=0;i< bqfs_size;i++){
			for(j=0;j<32;j++)
				data[j]=0;
			j=0;
			t=0;
			offset = bqfs_index[i].data_offset;
			size = bqfs_index[i].data_size;

			if(bqfs_index[i].i2c_cmd == 'W'){//write reg
				reg = firmware_data[offset];
				j = offset;
				while(j<(offset+size-1)){
					/*printk("%s:The reg= 0x%x data= 0x%x.\n",__func__,reg,firmware_data[j+1]);*/
					ret = bq27x00_write_i2c(di,reg,1,&firmware_data[j+1],1);
					if(ret < 0)
						printk("fw ret %d.\n",ret);
					reg+=1;
					j++;
				}
			}else if(bqfs_index[i].i2c_cmd == 'X'){//delay
				if(bqfs_index[i].i2c_addr < 10)
					mdelay(bqfs_index[i].i2c_addr+fw_delay);
				else if(bqfs_index[i].i2c_addr == 170)
					mdelay(bqfs_index[i].i2c_addr+fw_long_delay);
				else
					mdelay(bqfs_index[i].i2c_addr+fw_more_delay);
			}else if(bqfs_index[i].i2c_cmd == 'C'){//compare data 
				reg = firmware_data[offset];
				j = offset;
				ret = bq27530_read_i2c_4byte(di,reg,size-1);
				printk("%s: Reg= 0x%x,mult_data= 0x%x\n",__func__,reg,ret);
				while(j<(offset+size-1)){
					data[t] = firmware_data[j+1];
					j++;
					t++;
				}
				fw_data = get_unaligned_le32(data);
				if(ret != fw_data){
					fw_update_err = 1;
					printk("%s:Error The reg= 0x%x data=0x%x fw_data=0x%x compare fault.\n",__func__,reg,ret,fw_data);
					break;
				}
			}
		}
		if(!fw_update_err)
			break;
	}
	data[0]=0x41;
	data[1]=0x00;
	di->is_rom_mode = 0;
	ret=bq27x00_write_i2c(di,BQ27530_REG_CNTL,2,data,0);/*Reset*/
	if(ret <0){
		pr_err("%s: reset cmd write error\n",__func__);
		return -1;
	}

	di->df_ver = df_ver_get(di);
  	mdelay(1000);
  	di->chg_pn = chrg_get_vendor(di); 
	pr_err("%s:df_ver=0x%x,chg_pn=0x%x. done \n",__func__,di->df_ver, di->chg_pn);
	di->fw_dw_done = 1;

	return 0;

}

int otg_func_set(bool votg_on)
{
	int ret;
	unsigned char data[32]={0};

	if(bqdi == NULL){
		pr_err("Failed enable BQ27xxx OTG func\n");
        return -ENODEV;
	}
    pr_err("%s: run parameters %d\n", __func__, votg_on);

	__cancel_delayed_work(&bqdi->ovp_work);
#ifdef SUPPORT_QPNP_VBUS_OVP
	__cancel_delayed_work(&bqdi->vbus_work);
#endif
	if(bqdi->board->chg_otg_gpio < 0){
		pr_err("chg otg gpio is unvalid.\n");
    	return -ENODEV;
	}

	bq27531_charge_hiz_reset();
	
	if(votg_on){
		data[0]=0x15;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
            return -EIO;
		}
		gpio_direction_output(bqdi->board->chg_otg_gpio, 1);
		mdelay(220);
	}else{
		data[0]=0x16;
		data[1]=0x00;
		ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);
		if(ret <0){
			printk("%s: reg 0x00 write error\n",__func__);
            return -EIO;
		}
		gpio_direction_output(bqdi->board->chg_otg_gpio, 0);
		gpio_direction_input(bqdi->board->chg_otg_gpio);
		mdelay(220);
	}
    pr_err("%s: run parameters exit\n", __func__);
	return 0;
}


static int bq27531_soc_reset(void)
{
	int ret;
	unsigned char data[32]={0};

	if (!bqdi) {
		printk("%s: bqdi is NULL, return \n",__func__);
		return -1;
	}

	bqdi->soc_reset = 1;
	data[0]=0x41;
	data[1]=0x00;
	ret=bq27x00_write_i2c(bqdi,BQ27530_REG_CNTL,2,data,0);/*Reset*/
	if(ret <0){
		pr_err("%s: reset cmd write error\n",__func__);
		return -1;
	}

	bqdi->df_ver = df_ver_get(bqdi);
	mdelay(1000);
	mdelay(1000);
	mdelay(1000);
	bqdi->chg_pn = chrg_get_vendor(bqdi);
	pr_err("%s:df_ver=0x%x,chg_pn=0x%x. done \n",__func__,bqdi->df_ver, bqdi->chg_pn);
	bqdi->soc_reset = 0;

	return 0;
}

static int bq27531_charge_ic_reset(void)
{
	int ret, i;
	u8 data;

	if (!bqdi) {
		printk("%s: bqdi is NULL, return \n",__func__);
		return -1;
	}

        for(i=0;i<5;i++){
            ret = bq27x00_read_i2c(bqdi, BQ24192_REG_CTL_0, 1);
            if(ret > 0)
                break;
        }
        printk("%s: read hiz_reg=0x%x,i=%d.\n",__func__,ret,i);
        if(ret < 0){
            pr_err("bq27530 read charger reg00 err.\n");
        }else{
            data = ret & 0xff;
            data &= ~BQ24192_0_HIZ;
            for(i=0;i<5;i++){
                ret = bq27x00_write_i2c(bqdi,BQ24192_REG_CTL_0,1,&data,0);
                if(ret>0)
                    break;
            }
            printk("%s: write hiz_reg=0x%x,ret=%d,i=%d.\n",__func__,data,ret,i);
            if(ret <0)
                pr_err("%s: HIZ mode write error\n",__func__);
        }

	ret = bq27531_op_thermal_mitigation(bqdi, 0);
        if(ret)
                pr_err("%s: set charging CE failed %d\n", __func__, ret);

	ret = bq27531_op_set_input_limit(bqdi, IINLIM_2000);
        if(ret)
                pr_err("%s: set charging input power limit failed %d\n", __func__, ret);

	return ret;
}

static int bq27531_charge_hiz_reset(void)
{
	int ret, i;
	u8 data;

return 0;
	if (!bqdi) {
		printk("%s: bqdi is NULL, return \n",__func__);
		return -1;
	}

        for(i=0;i<5;i++){
            ret = bq27x00_read_i2c(bqdi, BQ24192_REG_CTL_0, 1);
            if(ret > 0)
                break;
        }
        printk("%s: read hiz_reg=0x%x,i=%d.\n",__func__,ret,i);
        if(ret < 0){
            pr_err("bq27530 read charger reg00 err.\n");
        }else{
            data = ret & 0xff;
            data &= ~BQ24192_0_HIZ;
            for(i=0;i<5;i++){
                ret = bq27x00_write_i2c(bqdi,BQ24192_REG_CTL_0,1,&data,0);
                if(ret>0)
                    break;
            }
            printk("%s: write hiz_reg=0x%x,ret=%d,i=%d.\n",__func__,data,ret,i);
            if(ret <0)
                pr_err("%s: HIZ mode write error\n",__func__);
        }

	return ret;
}

static int fg_reboot_done = 0;
static int fg_reboot_notifier_call(struct notifier_block *notifier,
				     unsigned long what, void *data)
{
	printk("%s\n", __func__);

	if(!bqdi) {
		printk("%s: bqdi is NULL, return\n", __func__);
		return NOTIFY_DONE;
	}

	if(fg_reboot_done) {
		printk("%s: fg reboot had done, return\n", __func__);
		return NOTIFY_DONE;
	}

	fg_reboot_done = 1;
	bq27531_charge_ic_reset();

	disable_irq(bqdi->chrg_irq_n);
	disable_irq(bqdi->fg_irq_n);
	cancel_delayed_work_sync(&bqdi->work);
	//bq27530_enable_charging(bqdi,true);
	otg_func_set(0);
	if(bqdi->chrg_irq_n)
		free_irq(bqdi->chrg_irq_n, bqdi);
	if(bqdi->fg_irq_n)
		free_irq(bqdi->fg_irq_n, bqdi);

	dev_info(bqdi->dev, "%s: The delay work and ISR was released for FG\n",__func__);

	return NOTIFY_DONE;

}

#ifdef QPNP_KPDPWR_BARK_NOTIFIER
static int fg_bark_notifier_call(struct notifier_block *notifier,
					unsigned long what, void *data)
{
	printk("%s\n", __func__);

	return NOTIFY_DONE;
}
#endif

static int uart_switch_write(const char *val, struct kernel_param *kp)
{
	param_set_int(val, kp);

	if(uart_switch == 1){
        usb_switch_to_uart(bqdi,true);
	}else{
        usb_switch_to_uart(bqdi,false);
	}

	return 0;
}
module_param_call(uart_switch,uart_switch_write,NULL,&uart_switch,0644);
MODULE_PARM_DESC(uart_switch, "switch usb to uart");

static int fw_write(const char *val, struct kernel_param *kp)
{
	int ret;

	param_set_int(val, kp);

	if(bc_fw == 1){
		ret = firmware_write(bqdi);
		if (!ret){
			dev_info(bqdi->dev, "FG FW download complete\n");
		}else{
			dev_err(bqdi->dev, "failed to download FW: %d\n", ret);
		}
	}else{
		dev_err(bqdi->dev, "Error!Only write 1 to download: %d\n", bc_fw);
	}

    printk("bq27530 %s end.\n",__func__);

	return 0;
}
module_param_call(bc_fw,fw_write,NULL,&bc_fw,0644);
MODULE_PARM_DESC(bc_fw, "Download FW in FG and Charger");

static ssize_t bc_fw_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	if(cmd == 1){
		ret = firmware_write(bqdi);
		if (!ret){
			dev_info(bqdi->dev, "FG FW download complete\n");
		}else{
			dev_err(bqdi->dev, "failed to download FW: %d\n", ret);
		}
	}else{
		dev_err(bqdi->dev, "Error!Only write 1 to download: %d\n", bc_fw);
	}

	return count;
}

ssize_t  chrg_ver_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    int ret;
    
    ret = chrg_get_vendor(bqdi); 

	if(ret == BQ24190_IC_VERSION)
	    ret = sprintf(buf, "%s\n", "bq24190");
	else if (ret == BQ24192_IC_VERSION)
	    ret = sprintf(buf, "%s\n", "bq24192");
	else if (ret == BQ24292I_IC_VERSION)
	    ret = sprintf(buf, "%s\n", "bq24292i");
	else
	    ret = sprintf(buf, "%s\n", "unknown");
    
    return ret;

}

ssize_t  fg_ver_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    int ret;
    ret = fg_version_get(bqdi);

    if(ret == 0x0531){
	    ret = sprintf(buf, "%s\n", "bq27531");
    }else{
	    ret = sprintf(buf, "%s\n", "unknown");
	}
    
    return ret;
}

ssize_t  fg_info_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    int ret;
    int max_capa;

    if (!bqdi) {
	printk("%s: bqdi is NULL, return \n",__func__);
	return -1;
    }

    max_capa = bq27x00_read(bqdi, 0x62, false);
    ret = sprintf(buf, "%4X,%X,%d,%d,%d,%d,%d\n",
			0xD006,
			df_ver_get(bqdi),
			gpio_get_value(bqdi->board->chg_psel_gpio),
			bqdi->vbus_ovp,
			bqdi->chrg_type,
			bqdi->battery_vendor_index,
			max_capa);

    return ret;
}

ssize_t thermal_mitigation_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    int ret;
    struct bq27x00_device_info *di = dev_get_drvdata(dev);

    ret = sprintf(buf, "%d\n", di->thermal_mitigation);

    return ret;
}

static ssize_t thermal_mitigation_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
        struct bq27x00_device_info *di = dev_get_drvdata(dev);
	int cmd;

	sscanf(buf, "%d", &cmd);
	di->thermal_mitigation = !!cmd;
	bq27531_op_thermal_mitigation(di, di->thermal_mitigation);
	printk("%s: %d\n", __func__, di->thermal_mitigation);

	return count;
}

ssize_t force_check_thermal_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    int ret;

    ret = sprintf(buf, "%d\n", force_check_thermal);

    return ret;
}

static ssize_t force_check_thermal_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd;

	sscanf(buf, "%d", &cmd);
	if (cmd == 1) {
		force_check_thermal = cmd;
	} else if (cmd == 0) {
		if (boot_done)
			printk("%s: cannot disable thermal check after boot done\n", __func__);
		else
			force_check_thermal = cmd;
	} else
		printk("%s: invalid value %d\n", __func__, cmd);

	printk("%s: %d\n", __func__, force_check_thermal);
	return count;
}

static DEVICE_ATTR(fw_write, S_IRUGO|S_IWUSR, NULL,bc_fw_write);
static DEVICE_ATTR(chrg_version, S_IRUGO|S_IWUSR, chrg_ver_get,NULL);
static DEVICE_ATTR(fg_version, S_IRUGO|S_IWUSR, fg_ver_get,NULL);
static DEVICE_ATTR(fg_info, S_IRUGO|S_IWUSR, fg_info_get,NULL);
static DEVICE_ATTR(thermal_mitigation, S_IRUGO|S_IWUSR, thermal_mitigation_get, thermal_mitigation_set);
static DEVICE_ATTR(force_check_thermal, S_IRUGO|S_IWUSR, force_check_thermal_get, force_check_thermal_set);
static struct attribute *fs_attrs[] = {
	&dev_attr_fw_write.attr,
	&dev_attr_chrg_version.attr,
	&dev_attr_fg_version.attr,
	&dev_attr_fg_info.attr,
	&dev_attr_thermal_mitigation.attr,
	&dev_attr_force_check_thermal.attr,
	NULL,
};

static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static unsigned int ic_reg = 0;
static int bc_reg_show(const char *val, struct kernel_param *kp)
{
	int ret;
	param_set_int(val, kp);

	if(ic_reg < 0)
		return -1;

	if(ic_reg <= 0x73)
	{
		ret = bq27x00_read(bqdi,ic_reg,false);
		dev_info(bqdi->dev, "FG reg: 0x%x = 0x%x\n",ic_reg,ret);
		return 0;
	}
	else
	{
		ret = bq27x00_read(bqdi,ic_reg,true);
		dev_info(bqdi->dev, "CHRG reg: 0x%x = 0x%x\n",ic_reg,ret);
		return 0;
	}
}
module_param_call(ic_reg,bc_reg_show,NULL,&ic_reg,0644);
MODULE_PARM_DESC(ic_reg, "Download FW in FG and Charger");

static unsigned int fw_version = 0;
static int fw_reg_show(const char *val, struct kernel_param *kp)
{

	param_set_int(val, kp);

	fw_ver_get(bqdi);
	df_ver_get(bqdi);

	return 0;
}
module_param_call(fw_version,NULL,fw_reg_show,&fw_version,0644);
MODULE_PARM_DESC(fw_version, "Download FW in FG and Charger");

#define BLOCK_DATA_CHECK_SUM_REG 0x60
#define BLOCK_DATA_CONTROL_REG 0x61
#define DATA_FLASH_CLASS_REG_SUBCLS 0x3E
#define DATA_FLASH_CLASS_REG_OFFSET 0x3F
#define mod(n, div) ((n) % (div))
#ifdef BQ_WRITE_TEMP
static int bq27531_data_flash_write(struct bq27x00_device_info *di,unsigned char sub_class,
				unsigned char offset,unsigned char new_value)
{
	unsigned char old_reg_value,new_reg_value,old_check_sum,new_check_sum;
	unsigned char data,sub_class_id,data_offset,reg_addr;
	int ret;
	/*unsigned char data_rst[2] = {0};*/

	if(di == NULL){
		printk("%s:di is NULL.\n",__func__);
		return -2;
	}
	sub_class_id = sub_class;
	data_offset = offset;
	new_reg_value = new_value;
	//enable data flash control
	data = 0x00;
	bq27x00_write_i2c(di,BLOCK_DATA_CONTROL_REG,1,&data,0);
	usleep(500);
	//find sub class
	bq27x00_write_i2c(di,DATA_FLASH_CLASS_REG_SUBCLS,1,&sub_class_id,0);
	usleep(500);
	//find sub class offset
	data = data_offset/32;
	bq27x00_write_i2c(di,DATA_FLASH_CLASS_REG_OFFSET,1,&data,0);
	usleep(500);
	//read old_reg_value
	reg_addr = 0x40 + mod(data_offset,32);
	old_reg_value = bq27x00_read(di, reg_addr, false);
	usleep(500);
	if(old_reg_value != new_reg_value && !read_only){
		//read old check sum
		old_check_sum = bq27x00_read(di,BLOCK_DATA_CHECK_SUM_REG, false);
		usleep(500);
		//calculate new chekc sum
		data = mod(255-old_check_sum-old_reg_value,256);
		new_check_sum = 255-mod(data + new_reg_value,256);
		//write new reg value
		bq27x00_write_i2c(di,reg_addr,1,&new_reg_value,0);
		usleep(500);
		//write new reg value to data flash actually
		bq27x00_write_i2c(di,BLOCK_DATA_CHECK_SUM_REG,1,&new_check_sum,0);
		usleep(500);
	}
	new_reg_value = bq27x00_read(di, reg_addr, false);
	usleep(500);
	if(new_reg_value != new_value){
		dev_info(di->dev, "sbc=%d,ofst=%d,raddr=%d,oval=%d,nval=%d,ocks=%x,ncks=%x,\n",
			sub_class_id,data_offset,reg_addr,old_reg_value,new_reg_value,old_check_sum,new_check_sum);
		return -1;
	}

	return 0;
}
#endif
static int bq27530_parse_dt(struct device *dev,
                struct bq27530_platform_data *bq27530_pdata)
{
	struct device_node *np = dev->of_node;

	/* reset, irq gpio info */
	bq27530_pdata->fg_int_gpio = of_get_named_gpio_flags(np,
			"bq27530,fg-int-gpio", 0, 0);
	bq27530_pdata->chg_int_gpio = of_get_named_gpio_flags(np,
			"bq27530,chg-int-gpio", 0, 0);
    bq27530_pdata->usb_sw_gpio = of_get_named_gpio_flags(np,
            "bq27530,usb-sw-gpio", 0, 0);
    bq27530_pdata->chg_en_gpio = of_get_named_gpio_flags(np,
            "bq27530,chg-en-gpio", 0, 0);
    bq27530_pdata->chg_otg_gpio = of_get_named_gpio_flags(np,
            "bq27530,chg-otg-gpio", 0, 0);
    bq27530_pdata->chg_psel_gpio = of_get_named_gpio_flags(np,
            "bq27530,chg-psel-gpio", 0, 0);
    bq27530_pdata->usb_sw_en_gpio = of_get_named_gpio_flags(np,
            "bq27530,usb-sw-en-gpio", 0, 0);

	return 0;
}

static int bq27x00_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct bq27x00_device_info *di;
	int num;
	int usb_present;
	int retval = 0;
	int bat_flag;
    struct bq27530_platform_data *platform_data = NULL;

    printk("%s.\n",__func__);

	usb_present = bq24192_is_usbin_present();
	if(usb_present < 0){
		printk("%s deffered.\n",__func__);
		return -EPROBE_DEFER;
	}
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
			sizeof(*platform_data),
			GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		retval = bq27530_parse_dt(&client->dev, platform_data);
		if (retval)
			return retval;
	} else {
		platform_data = client->dev.platform_data;
	}

	/* Get new ID for the new battery device */
	retval = idr_pre_get(&battery_id, GFP_KERNEL);
	if (retval == 0)
		return -ENOMEM;
	mutex_lock(&battery_mutex);
	retval = idr_get_new(&battery_id, client, &num);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}

	di->id = num;
	di->dev = &client->dev;
	di->bat.name = "battery";
	di->bus.read = &bq27x00_read_i2c;
  	di->board = platform_data;
  	di->board->chg_current = CURRENT_UNDEFINE;

	di->is_suspend = 0;
	di->is_rom_mode = 0;
	di->fw_dw_done = 1;
	di->soc_reset = 0;
	di->reset_p = 0;
	di->present_check_count = 0;
	di->thermal_mitigation = 0;

	i2c_set_clientdata(client, di);

	if (gpio_is_valid(platform_data->usb_sw_gpio)) {
		retval = gpio_request(platform_data->usb_sw_gpio,
				"chg_sw_gpio");
		if (retval) {
			dev_err(di->dev, "unable to request gpio [%d]\n",
						platform_data->usb_sw_gpio);
		}

        retval = gpio_direction_output(platform_data->usb_sw_gpio, 0);
		if (retval) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				platform_data->usb_sw_gpio);
		}
	}

	if (gpio_is_valid(platform_data->chg_en_gpio)) {
        retval = gpio_request(platform_data->chg_en_gpio, "CHRG_EN");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->chg_en_gpio, retval);
        }
    }

	if (gpio_is_valid(platform_data->chg_otg_gpio)) {
        retval = gpio_request(platform_data->chg_otg_gpio, "CHRG_OTG");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->chg_otg_gpio, retval);
        }
    }

	if (gpio_is_valid(platform_data->chg_psel_gpio)) {
        retval = gpio_request(platform_data->chg_psel_gpio, "CHRG_PSEL");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->chg_psel_gpio, retval);
        }
    }else
        pr_err("bq27530 chg psel gpio is not valid.\n");

    dev_err(di->dev, "power supply_init\n");
    if(bq27x00_powersupply_init(di))
		goto batt_failed_3;

	if (gpio_is_valid(platform_data->chg_int_gpio)) {
        retval = gpio_request(platform_data->chg_int_gpio, "CHG_INT");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->chg_int_gpio, retval);
        }else{
            di->chrg_irq_n = gpio_to_irq(platform_data->chg_int_gpio);
            retval = request_threaded_irq(di->chrg_irq_n,NULL,
                charge_irq,IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND, "bq24192",di);
            if (retval) {
                dev_err(di->dev, "cannot get IRQ:%d\n", di->chrg_irq_n);
            } else {
                enable_irq_wake(di->chrg_irq_n);
            }
        }
    }


	if (gpio_is_valid(platform_data->fg_int_gpio)) {
        retval = gpio_request(platform_data->fg_int_gpio, "FG_INT");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->fg_int_gpio, retval);
        }else{
            di->fg_irq_n = gpio_to_irq(platform_data->fg_int_gpio);
            retval = request_threaded_irq(di->fg_irq_n,
                    fg_irq,fg_thread_handler,
                    IRQF_TRIGGER_FALLING|IRQF_NO_SUSPEND , "bq27531",di);
            if (retval) {
                dev_err(di->dev, "cannot get IRQ:%d\n", di->fg_irq_n);
            } else {
                enable_irq_wake(di->fg_irq_n);
            }
        }
    }
	if (gpio_is_valid(platform_data->usb_sw_en_gpio)) {

        retval = gpio_request(platform_data->usb_sw_en_gpio, "USB_SW_EN");
        if (retval) {
            dev_err(di->dev,
                "Failed to request gpio %d with error %d\n",
                platform_data->usb_sw_en_gpio, retval);
        }else{
			gpio_direction_output(platform_data->usb_sw_en_gpio, 0);
        }
    }

	schedule_delayed_work(&di->boot_work, msecs_to_jiffies(300000));//300s

	mdelay(100);
	bat_flag = bq27x00_read(di, BQ27x00_REG_FLAGS, false);
	dev_info(di->dev, "0x0A REG ret = 0x%x\n",bat_flag);

	di->df_ver = df_ver_get(di);
  	di->chg_pn = chrg_get_vendor(di); 
	di->battery_vendor_index = (qpnp_batt_id < 1000000) ? BATT_VENDOR_LG : BATT_VENDOR_ATL_AND_DUMY;
	printk("bq27530 Battery ID: %ld, Vendor: [ %d ]  %s\n", qpnp_batt_id, di->battery_vendor_index,
				(di->battery_vendor_index == BATT_VENDOR_LG) ? "LG":"ATL or Dumy");

	printk("bq27530 check firmware step1 chrg_pn=0x%x.\n",di->chg_pn);
	if(di->df_ver < 0 || di->chg_pn < 0){
		retval = firmware_write(di);
		if (!retval){
			dev_info(di->dev, "FG FW update step1 complete\n");
		}
		else{
			dev_err(di->dev, "step1 failed to update FW: %d\n", retval);
		}
	}
	di->df_ver = df_ver_get(di);
  	di->chg_pn = chrg_get_vendor(di); 
	printk("bq27530 check firmware step2 df_ver=0x%x, chrg_pn=0x%x.\n",di->df_ver, di->chg_pn);
	if(di->chg_pn == BQ24292I_IC_VERSION){
		if(((di->battery_vendor_index == BATT_VENDOR_ATL_AND_DUMY) && (di->df_ver != BATT_FW_VER_ATL))
				|| ((di->battery_vendor_index == BATT_VENDOR_LG) && (di->df_ver != BATT_FW_VER_LG))){
			retval = firmware_write(di);
			if (!retval){
				dev_info(di->dev, "FG FW update step2 complete\n");
			}
			else{
				dev_err(di->dev, "step 2 failed to update FW:chg_pn:0x%x,df_ver:0x%x\n", di->chg_pn,di->df_ver);
			}
		}
	}else if(di->chg_pn == BQ24192_IC_VERSION){
		if(di->df_ver != 0xAA01){
			retval = firmware_write(di);
			if (!retval){
				dev_info(di->dev, "FG FW update complete\n");
			}
			else{
				dev_err(di->dev, "step 2 failed to update FW:chg_pn:0x%x,df_ver:0x%x\n", di->chg_pn,di->df_ver);
			}
		}
	}else{
		pr_err("%s:unknow chip version.\n",__func__);
	}

	retval = sysfs_create_group(&di->dev->kobj,&fs_attr_group);
	if (retval) {
		dev_err(di->dev, "failed to setup sysfs ret = %d\n", retval);
	}

	bqdi = di;
#ifdef CONFIG_FB
	configure_fb_notifiler(di);
#endif
  	bq27530_enable_charging(di, true);
	schedule_delayed_work(&di->work, msecs_to_jiffies(delay_t/30));//30s
#ifdef SUPPORT_QPNP_VBUS_OVP
	schedule_delayed_work(&di->vbus_work, msecs_to_jiffies(1000));
#endif

	return 0;

batt_failed_3:
	kfree(di);
batt_failed_2:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);
    kfree(platform_data);

	return retval;
}

static int bq27x00_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di = i2c_get_clientdata(client);

	bq27x00_powersupply_unregister(di);

	kfree(di->bat.name);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	wake_lock_destroy(&di->wakelock);
	if(di->chrg_irq_n)
		free_irq(di->chrg_irq_n, di);
	if(di->fg_irq_n)
		free_irq(di->fg_irq_n, di);
	kfree(di);
	kfree(bqdi);

	return 0;
}

static const struct i2c_device_id bq27x00_id[] = {
	{ "bq27530", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27x00_id);

#ifdef CONFIG_PM
static int bq27x00_battery_suspend(struct device *dev)
{

	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	dev_info(di->dev, "Bq27531 suspend\n");
	di->is_suspend = 1;
	if (!boot_done) {
		if (force_check_thermal == -1)
			force_check_thermal = 1;
		boot_done = 1;
		printk("%s: boot_done=%d, force_check_thermal=%d\n", __func__, boot_done, force_check_thermal);
	}

	disable_irq(bqdi->chrg_irq_n);
	disable_irq(bqdi->fg_irq_n);
	cancel_delayed_work(&di->work);
	cancel_delayed_work(&di->ovp_work);
#ifdef SUPPORT_QPNP_VBUS_OVP
	//cancel_delayed_work(&di->vbus_work);
#endif

	return 0;

}
static int bq27x00_battery_resume(struct device *dev)
{
	struct bq27x00_device_info *di = dev_get_drvdata(dev);

	dev_info(di->dev, "Bq27531 resume\n");
#ifdef SUPPORT_QPNP_VBUS_OVP
	//schedule_delayed_work(&di->vbus_work, 0);
#endif
	//schedule_delayed_work(&di->ovp_work, 0);
	schedule_delayed_work(&di->work, 0);
	enable_irq(bqdi->fg_irq_n);
	enable_irq(bqdi->chrg_irq_n);
	di->is_suspend = 0;

	return 0;
}

static int bq27531_op_write_reg(struct bq27x00_device_info *di, u8 addr, u8 value, u8 mask)
{
	int ret, i, old_data;
	unsigned char data;

        for (i=0; i<5; i++) {
		ret = bq27x00_read_i2c(di, addr, 1);
		if(ret > 0)
			break;
	}

	if (ret < 0) {
		printk("%s: read register 0x%x error, %d\n", __func__, addr, ret);
		return ret;
	}

	old_data = ret;
        data = (ret & mask) | value;

        for (i=0; i<5; i++) {
                ret = bq27x00_write_i2c(di, addr, 1, &data, 0);
		if(ret > 0)
                	break;
        }
        if (ret <0){
		printk("%s: write register 0x%x error(%d), read data is 0x%x\n", __func__, addr, ret, old_data);
		return ret;
	} else
       		printk("%s: write register 0x%x from 0x%x to 0x%x\n", __func__, addr, old_data, data);

	return 0;
}

/* IINLIM */
static int bq27531_op_set_input_limit(struct bq27x00_device_info *di, int value)
{
	printk("%s: %d\n", __func__, value);
	return bq27531_op_write_reg(di, BQ24192_REG_CTL_0, value, 0xf8);
}

/* CHG_/CE */
static int bq27531_op_thermal_mitigation(struct bq27x00_device_info *di, int level)
{
	int value = level;

	if(di->board->chg_en_gpio <= 0){
		printk("di->board->chg_en_gpio is <= 0.\n");
		return -1;
	}

        printk("%s: bq27530 set CHG_/CE gpio %s .\n", __func__, !level ? "Enable":"Disabled");
        gpio_set_value_cansleep(di->board->chg_en_gpio, value);

	return 0;
}

static int bq27531_config_charging_current(struct bq27x00_device_info *di, int level)
{
	printk("%s: chrg_type=%d, level=%d, vbus_ovp=%d, call=%d\n",
		__func__, di->chrg_type, level, di->vbus_ovp, g_call_status);

	if (di->vbus_ovp == 1)
		return 0;

	switch(di->chrg_type) {
        case POWER_SUPPLY_TYPE_USB_CDP:
		bq27531_op_set_input_limit(di, IINLIM_1500);
		break;
        case POWER_SUPPLY_TYPE_USB_ACA:
        case POWER_SUPPLY_TYPE_USB_DCP:
        case POWER_SUPPLY_TYPE_UNKNOWN:
		if((level && !g_call_status) || (!boot_done))
			bq27531_op_set_input_limit(di, IINLIM_2000);
		else {
			if (g_call_status)
				bq27531_op_set_input_limit(di, IINLIM_900);
			else
				bq27531_op_set_input_limit(di, IINLIM_1500);
		}
		break;
        case POWER_SUPPLY_TYPE_USB:
		bq27531_op_set_input_limit(di, IINLIM_500);
		break;
        case POWER_SUPPLY_TYPE_BATTERY:
		break;
	default:
		printk("%s: defalt chg_type %d", __func__, di->chrg_type);
		break;
	}
	
	return 0;
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	unsigned long *blank;
	struct bq27x00_device_info *di =
		container_of(self, struct bq27x00_device_info, fb_notif);

	printk("%s: event is %ld\n", __func__, event);
	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		printk("%s: blank is %ld\n", __func__, *blank);
		if (*blank == FB_BLANK_UNBLANK)
			bq27531_config_charging_current(di, 0);
		else if (*blank == FB_BLANK_POWERDOWN)
			bq27531_config_charging_current(di, 1);
		fb_status = *blank;
	}

	return 0;
}

static void configure_fb_notifiler(struct bq27x00_device_info *di)
{
	int retval = 0;

	di->fb_notif.notifier_call = fb_notifier_callback;

	retval = fb_register_client(&di->fb_notif);
	if (retval)
		dev_err(di->dev,
			"Unable to register fb_notifier: %d\n", retval);
	return;
}
#endif
#else
#define bq27x00_battery_suspend NULL
#define bq27x00_battery_resume NULL
#endif
static const struct dev_pm_ops bq27x00_pm_ops = {
	.suspend		= bq27x00_battery_suspend,
	.resume			= bq27x00_battery_resume,
};

static struct notifier_block fg_reboot_notifier = {
	.notifier_call = fg_reboot_notifier_call,
};

#ifdef QPNP_KPDPWR_BARK_NOTIFIER
static struct notifier_block fg_kpdpwr_bark_notifier = {
	.notifier_call	= fg_bark_notifier_call,
};
#endif

static const struct of_device_id bq27530_match[] = {
	{ .compatible = "ti,bq27530-battery" },
	{ },
};

static struct i2c_driver bq27x00_battery_driver = {
	.driver = {
		.name = "bq27x00-battery",
		.owner	= THIS_MODULE,
		.pm = &bq27x00_pm_ops,
		.of_match_table = of_match_ptr(bq27530_match),
	},
	.probe = bq27x00_battery_probe,
	.remove = bq27x00_battery_remove,
	.id_table = bq27x00_id,
};

static inline int bq27x00_battery_i2c_init(void)
{
	int ret = i2c_add_driver(&bq27x00_battery_driver);
	printk("%s:BQ27530 register_i2c driver\n",__func__);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27x00 i2c driver\n");

	return ret;
}

static inline void bq27x00_battery_i2c_exit(void)
{
	i2c_del_driver(&bq27x00_battery_driver);
}

/*
 * Module stuff
 */

static int __init bq27x00_battery_init(void)
{
	int ret;
	ret = bq27x00_battery_i2c_init();

	if (register_reboot_notifier(&fg_reboot_notifier))
		pr_warning("fg: unable to register reboot notifier");

#ifdef QPNP_KPDPWR_BARK_NOTIFIER
	ret = blocking_notifier_chain_register(&qpnp_kpdpwr_bark_list, &fg_kpdpwr_bark_notifier);
	if (ret)
		pr_err("Unable to register qpnp kpdpwr bark notifier: %d\n", ret);
#endif
	return ret;
}
module_init(bq27x00_battery_init);

static void __exit bq27x00_battery_exit(void)
{
	bq27x00_battery_i2c_exit();
	unregister_reboot_notifier(&fg_reboot_notifier);
#ifdef QPNP_KPDPWR_BARK_NOTIFIER
	blocking_notifier_chain_register(&qpnp_kpdpwr_bark_list, &fg_kpdpwr_bark_notifier);
#endif
}
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("BQ27x00 battery monitor driver");
MODULE_LICENSE("GPL");
