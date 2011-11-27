/* arch/arm/mach-msm/htc_battery_wince.c
 * Based on: htc_battery.c by HTC and Google
 * also based on Xandroid algorithm and HTCLeo (thank you guys!)
 *
 * updated by r0bin and photon community (2011)
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2008 Google, Inc.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <linux/io.h>

#include "proc_comm_wince.h"

#include <mach/msm_iomap.h>
#include <mach/htc_battery_wince.h>

static struct wake_lock vbus_wake_lock;
static int bat_suspended = 0;
static int batt_vref = 1238;
static int batt_vref_half = 615;
static int g_usb_online;
#undef NO_BATTERY_COMPUTATION_IN_SUSPEND_MODE 
#ifdef NO_BATTERY_COMPUTATION_IN_SUSPEND_MODE
DECLARE_COMPLETION(batt_thread_can_start);
#endif

enum {
	DEBUG_BATT	= 1<<0,
	DEBUG_CABLE	= 1<<1,
	DEBUG_LOG	= 1<<2,
};
static int debug_mask = 0;
module_param_named(debug, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

/* Default battery will be the photon 1200mAh.
 * batt_param will be set after battery detection but must be initialized
 * because it may be used before battery is correctly detected
 * sBattery_Parameters is defined in htc_battery_wince.h.
 * To add a new battery profile, just add it in the htc_battery_wince.h and modify the battery 
 * detection routine of the device
 */
static struct sBattery_Parameters* batt_param = (struct sBattery_Parameters*)&sBatParams_photon[0];

#define MODULE_NAME "htc_battery"

//#define TRACE_BATT	1
#undef TRACE_BATT

#if TRACE_BATT
 #define BATT(x...) printk(KERN_INFO "[BATT] " x)
#else
 #define BATT(x...) do {} while (0)
#endif

#define BATT_ERR(x...) printk(KERN_ERR "[BATT_ERR] " x)


/* battery detail logger */
//#define HTC_BATTERY_BATTLOGGER	1
#undef HTC_BATTERY_BATTLOGGER

#if HTC_BATTERY_BATTLOGGER
 #include <linux/rtc.h>
 #define BATTLOG(x...) do { \
 struct timespec ts; \
 struct rtc_time tm; \
 getnstimeofday(&ts); \
 rtc_time_to_tm(ts.tv_sec, &tm); \
 printk(KERN_INFO "[BATTLOG];%d-%02d-%02d %02d:%02d:%02d", \
 tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
 tm.tm_hour, tm.tm_min, tm.tm_sec); \
 printk(";" x); \
 } while (0)
#else
 #define BATTLOG(x...) do {} while (0)
#endif

struct htc_battery_info {
	int present;
	unsigned long update_time;

	/* lock to protect the battery info */
	struct mutex lock;

	struct battery_info_reply rep;
	smem_batt_t *resources;
};

static struct htc_battery_info htc_batt_info;
static struct battery_info_reply old_batt_info;

static unsigned int cache_time = 1000;

static int is_battery_initialized = 0;
static bool not_yet_started = true;
static unsigned int time_stamp = 0;

/* simple maf filter stuff - how much old values should be used for recalc ...*/
#define BATT_MAF_SIZE 15
static short volt_maf_buffer[BATT_MAF_SIZE];
static short volt_maf_size = 0;
static short volt_maf_last = 0;

static void maf_add_value( short volt )
{
	// check if we need to correct the index
	if ( volt_maf_last == BATT_MAF_SIZE-1 )
		volt_maf_last = 0;

	// add value to filter buffer
	volt_maf_buffer[volt_maf_last] = volt;
	volt_maf_last++;

	if ( volt_maf_size != BATT_MAF_SIZE-1 )
		volt_maf_size++;	
}

/* calculated on the fly.... no caching */
static short maf_get_avarage(void)
{
	int i;
	int maf_temp;

	// make sure we only do it when we have data
	if ( volt_maf_size == 0 )
		return 0;

	// no need todo the avaraging
	if ( volt_maf_size == 1 )
		return volt_maf_buffer[0];

	// our start value is the first sample
	maf_temp = volt_maf_buffer[0];

	for (i=1; i < volt_maf_size; i++) {
		maf_temp = ( maf_temp + volt_maf_buffer[i] ) / 2;		
	}

	return maf_temp;
}

static void maf_clear(void)
{
	int i;
	for ( i = 0; i < BATT_MAF_SIZE;i++ )
		volt_maf_buffer[i] = 0;

	volt_maf_size = 0;
	volt_maf_last = 0;
}


/* ADC linear correction numbers.
 */
static u32 htc_adc_a = 0;					// Account for Divide Resistors
static u32 htc_adc_b = 0;
static u32 htc_adc_range = 0x1000;	// 12 bit adc range correction.
static u32 batt_vendor = 0;

#define GET_BATT_ID         readl(MSM_SHARED_RAM_BASE + 0xFC0DC) 
#define GET_ADC_VREF        readl(MSM_SHARED_RAM_BASE + 0xFC0E0) 
#define GET_ADC_0_5_VREF    readl(MSM_SHARED_RAM_BASE + 0xFC0E4) 

static int get_battery_id_detection( struct battery_info_reply *buffer );
static int htc_get_batt_info( struct battery_info_reply *buffer );

static int init_battery_settings( struct battery_info_reply *buffer ) {

	if ( buffer == NULL )
		return -EINVAL;

	mutex_lock( &htc_batt_info.lock );

	batt_vref = GET_ADC_VREF;
	batt_vref_half = GET_ADC_0_5_VREF;

	if ( batt_vref - batt_vref_half >= 500 ) {
		// set global correction var
		htc_adc_a = 625000 / ( batt_vref - batt_vref_half );
		htc_adc_b = 1250000 - ( batt_vref * htc_adc_a );
	}

	// calculate the current adc range correction.
	htc_adc_range = ( batt_vref * 0x1000 ) / 1250;

	if ( get_battery_id_detection( buffer ) < 0 ) {
		mutex_unlock(&htc_batt_info.lock);
		if(debug_mask&DEBUG_LOG)
			BATT_ERR("Critical Error on: get_battery_id_detection: VREF=%d; 0.5-VREF=%d; ADC_A=%d; ADC_B=%d; htc_adc_range=%d; batt_id=%d; batt_vendor=%d; full_bat=%d\n", \
			batt_vref, batt_vref_half, htc_adc_a, htc_adc_b, htc_adc_range, buffer->batt_id, batt_vendor, buffer->full_bat);
		return -EINVAL;
	}

	mutex_unlock(&htc_batt_info.lock);
	
	if ( htc_get_batt_info( buffer ) < 0 )
		return -EINVAL;

	if(debug_mask&DEBUG_LOG)
		BATT("init_battery_settings: VREF=%d; 0.5-VREF=%d; ADC_A=%d; ADC_B=%d; htc_adc_range=%d; batt_id=%d; batt_vendor=%d; full_bat=%d\n", \
		batt_vref, batt_vref_half, htc_adc_a, htc_adc_b, htc_adc_range, buffer->batt_id, batt_vendor, buffer->full_bat);

	return 0;
}

static int get_battery_id_detection( struct battery_info_reply *buffer ) {
	u32 batt_id;
	struct msm_dex_command dex;

	dex.cmd = PCOM_GET_BATTERY_ID;
	msm_proc_comm_wince( &dex, 0 );

	batt_id = GET_BATT_ID;

	/* buffer->batt_id will be overwritten on next battery reading so we can use it as
	 * a temp variable to pass it to machine specific battery detection
	 */
	buffer->batt_id = batt_id;
	// apply the adc range correction
	buffer->batt_id = ( buffer->batt_id * 0xA28 ) / htc_adc_range;  
        //photon batt capacity = 1200Mah
	buffer->full_bat = 1200000;

	//update battery params
	batt_param = (struct sBattery_Parameters*) sBatParams_photon[0];	
	return 0;
}

static enum power_supply_property htc_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property htc_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

/* HTC dedicated attributes */
static ssize_t htc_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val);

static struct power_supply htc_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = htc_battery_properties,
		.num_properties = ARRAY_SIZE(htc_battery_properties),
		.get_property = htc_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = htc_power_properties,
		.num_properties = ARRAY_SIZE(htc_power_properties),
		.get_property = htc_power_get_property,
	},
};

/* -------------------------------------------------------------------------- */
/* Workqueue for vbus notifier */
int htc_cable_status_update(const char *sfrom);
static struct workqueue_struct *g_vbus_notifier_work_queue;
static void vbusnotifier_work(struct work_struct *work)
{
	//as soon as we got an event from vbus, update cable status
	htc_cable_status_update(__func__);
}
static DECLARE_WORK(g_vbus_notifier_work, vbusnotifier_work);

/* -------------------------------------------------------------------------- */
/* HTCLeo Dex Functions. */
static int get_vbus_state(void)
{
	if (readl(MSM_SHARED_RAM_BASE + 0xfc00c))
		return 1;
	else
		return 0;
}

void notify_cable_status(int status)
{
	printk("%s, VBUS IRQ triggered, VBUS=%d)\n", __func__,status);
	//activate VBUS for usb driver
	msm_hsusb_set_vbus_state(status);
	//queue work to avoid blocking irq
	queue_work(g_vbus_notifier_work_queue, &g_vbus_notifier_work);	
}

// called from DEX intrrupt
void notify_vbus_change_intr(void)
{
	if (!is_battery_initialized)
		return;
	notify_cable_status(get_vbus_state());
}
/* -------------------------------------------------------------------------- */

#if defined(CONFIG_DEBUG_FS)
int htc_battery_set_charging(batt_ctl_t ctl);
static int batt_debug_set(void *data, u64 val)
{
	return htc_battery_set_charging((batt_ctl_t) val);
}

static int batt_debug_get(void *data, u64 *val)
{
	return -ENOSYS;
}

DEFINE_SIMPLE_ATTRIBUTE(batt_debug_fops, batt_debug_get, batt_debug_set, "%llu\n");
static int __init batt_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("htc_battery", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("charger_state", 0644, dent, NULL, &batt_debug_fops);

	return 0;
}

device_initcall(batt_debug_init);
#endif

static int init_batt_gpio(void)
{
	//r0bin: A9 will shutdown the phone if battery is pluged out, so don't bother
	//if (gpio_request(htc_batt_info.resources->gpio_battery_detect, "batt_detect") < 0)
	//	goto gpio_failed;
	
	//charge control
	if (gpio_request(htc_batt_info.resources->gpio_charger_enable, "charger_en") < 0)
	{
		printk("%s: gpio request charger_en failed!\n",__FUNCTION__);
		goto gpio_failed;
	}
	
	//high speed or low speed charge
	if (gpio_request(htc_batt_info.resources->gpio_charger_fast_dis, "fast_charge_dis") < 0){
		printk("%s: gpio request gpio_charger_fast_dis no%d failed!\n",__FUNCTION__,htc_batt_info.resources->gpio_charger_fast_dis );
		goto gpio_failed;
	}
	if (gpio_request(htc_batt_info.resources->gpio_charger_fast_en, "fast_charge_en") < 0){
		printk("%s: gpio request gpio_charger_fast_en no%d failed!\n",__FUNCTION__,htc_batt_info.resources->gpio_charger_fast_en );
		goto gpio_failed;
	}
	//r0bin: no need of another gpio, we detect AC through other means
	//if ( machine_is_htckovsky() || machine_is_htctopaz() )
	//	if (gpio_request(htc_batt_info.resources->gpio_ac_detect, "ac_detect") < 0)
	//		goto gpio_failed;
	
	return 0;

gpio_failed:
	return -EINVAL;
}

/*
 *	battery_charging_ctrl - battery charing control.
 * 	@ctl:			battery control command
 *
 */
static int battery_charging_ctrl(batt_ctl_t ctl)
{
	int result = 0;

	switch (ctl) {
	case DISABLE:
		if(debug_mask&DEBUG_CABLE)
			BATT("CTRL charger OFF\n");
		/* 0 for enable; 1 disable */
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_dis, 1);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_en, 0);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 1);
		break;
	case ENABLE_SLOW_CHG:
		if(debug_mask&DEBUG_CABLE)
			BATT("CTRL charger ON (SLOW)\n");
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_dis, 1);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_en, 0);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 0);
		break;
	case ENABLE_FAST_CHG:
		if(debug_mask&DEBUG_CABLE)
			BATT("CTRL charger ON (FAST)\n");
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_dis, 0);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_fast_en, 1);
		result = gpio_direction_output(htc_batt_info.resources->gpio_charger_enable, 0);
		break;
	default:
		BATT_ERR("Not supported battery ctr called.!\n");
		result = -EINVAL;
		break;
	}

	return result;
}

int htc_battery_set_charging(batt_ctl_t ctl)
{
	int rc;

	if ((rc = battery_charging_ctrl(ctl)) < 0)
		goto result;

	if (!is_battery_initialized) {
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
	} else {
		mutex_lock(&htc_batt_info.lock);
		htc_batt_info.rep.charging_enabled = ctl & 0x3;
		mutex_unlock(&htc_batt_info.lock);
	}
result:
	return rc;
}


//JH //this is for packet filter (notify port list while USB in/out)
int update_port_list_charging_state(int enable);

int htc_cable_status_update(const char *sfrom)
{
	int rc = 0;
	unsigned source;
	int status;
	unsigned last_source;
	unsigned vbus_status;

	vbus_status = get_vbus_state();

	if ( debug_mask&DEBUG_LOG )
		printk("%s called from %s\n",__func__,sfrom);
	
	if (!is_battery_initialized)
		return 0;
	
	if (vbus_status && (!g_usb_online) && (htc_batt_info.rep.charging_source !=CHARGER_AC))
	{
		//usually, it takes less than 1sec for usb gadget to detect usb cable
		//we give time to usb notifier to modify g_usb_online status
		printk("%s: detected USB cable insertion (AC charger). Wait 1sec to see if it's USB charger or AC charger\n",__func__);
		msleep(1000);
	}
	
	mutex_lock(&htc_batt_info.lock);
	if(vbus_status && g_usb_online) {
		status=CHARGER_USB;	/* vbus present, usb connection online = USB CHARGER */
		printk("%s vbus present, usb connection=%d => USB CHARGER\n",__func__,g_usb_online);
	} else if (vbus_status && !g_usb_online) {
		status=CHARGER_AC;	/* vbus present, no usb = AC CHARGER */
		printk("%s vbus present, usb connection=%d =>AC CHARGER\n",__func__,g_usb_online);
	} else {
		printk("%s no vbus, no usb, usb connection=%d => ON BATTERY\n",__func__,g_usb_online);
		g_usb_online = 0;
		status=CHARGER_BATTERY;
	}
	last_source = htc_batt_info.rep.charging_source;

	switch(status) {
	case CHARGER_BATTERY:
		htc_batt_info.rep.charging_source = CHARGER_BATTERY;
		htc_batt_info.rep.charging_enabled = 0;
		break;
	case CHARGER_USB:
		htc_batt_info.rep.charging_source = CHARGER_USB;
		htc_batt_info.rep.charging_enabled = 1;
		break;
	case CHARGER_AC:
		htc_batt_info.rep.charging_source = CHARGER_AC;
		htc_batt_info.rep.charging_enabled = 1;
		break;
	default:
		BATT_ERR("%s - Not supported cable status received!\n", __FUNCTION__);
		rc = -EINVAL;
	}
	source = htc_batt_info.rep.charging_source;
	
	//maf reset if we change source
	if(source != last_source)
	{
		maf_clear();
		//JH //this is for packet filter (notify port list while USB in/out)
		update_port_list_charging_state(!(htc_batt_info.rep.charging_source == CHARGER_BATTERY));
	}
	
	mutex_unlock(&htc_batt_info.lock);
	
    htc_battery_set_charging(status);
	//r0bin: fix battery drain issue & keep usb connection stable!
	if(!((source==CHARGER_USB) || (source==CHARGER_AC)))
		msm_hsusb_set_vbus_state(0);
	
	if (  source == CHARGER_USB || source==CHARGER_AC ) {
		wake_lock(&vbus_wake_lock);
	} else if(last_source != source) {
		/* give userspace some time to see the uevent and update
		 * LED state or whatnot...
		 */
		wake_lock_timeout(&vbus_wake_lock, HZ);
	} else {
		wake_unlock(&vbus_wake_lock);
	}

	/* make sure that we only change the powersupply state if we really have to */
	if (source == CHARGER_BATTERY || last_source == CHARGER_BATTERY)
		power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
	if (source == CHARGER_USB || last_source == CHARGER_USB)
		power_supply_changed(&htc_power_supplies[CHARGER_USB]);
	if (source == CHARGER_AC || last_source == CHARGER_AC)
		power_supply_changed(&htc_power_supplies[CHARGER_AC]);

	return rc;
}


struct htc_batt_info_u16 {
	volatile u16 batt_id;
	volatile u16 batt_temp;
	volatile u16 batt_vol;
	volatile s16 batt_charge;
	volatile u16 batt_discharge;
};

/* Common routine to calculate temperature */
static int htc_battery_temperature_lut( int av_index )
{
	return (637-av_index*338/1155);
}

//used for last charge_status
#ifdef USE_AGING_ALGORITHM
#define CHARGE_STATUS_AGESIZE 6
static int charge_status_age[CHARGE_STATUS_AGESIZE];
#endif
static int old_level = -1;
static int charge_curr_ref = 0;
static long current_loaded_mAs = 0;
static int max_curr = 0;
static int stop_charge_counter = 0;
static int old_current = 0;
static unsigned int old_time = 0;
//to be removed when debug finished
unsigned long mcurr_jiffies = 0;
unsigned long mold_jiffies = 0;
static int nbComputationsOnBattery = 0;
#define BATT_CAPACITY_PHOTON 1200

/* Common routine to compute the battery level */
static void htc_battery_level_compute( struct battery_info_reply *buffer )
{
	int result = 0;
	int volt, ccurrent, volt_discharge_resistor, corrected_volt;
	int temp, temp_correct_volt = 0;	
	
	temp =  buffer->batt_temp;
	volt = buffer->batt_vol;	
	ccurrent = buffer->batt_current;	
	
//r0bin: do we really need this? our algo is different
#ifdef USE_AGING_ALGORITHM
	/* aging, not to calc with first values after charging status will be changed */
	int i;
	for ( ( i = CHARGE_STATUS_AGESIZE - 1); i > 0; i--) {
		charge_status_age[i] = charge_status_age[(i - 1)];
	}
	charge_status_age[0] = buffer->charging_enabled+1;// 0 will be used on empty values / 1 = batt / 2 = charging
	for (i=1; i < CHARGE_STATUS_AGESIZE; i++) {
		if ( charge_status_age[i] < 1 )
			charge_status_age[i] = charge_status_age[0];
		if ( charge_status_age[0] != charge_status_age[i] ) {
			if ( debug_mask&DEBUG_LOG )
				BATT("Charger status changed: Charge_New=%d; Charge_Old[%d/%d]=%d\n",
				charge_status_age[0], (i + 1), (CHARGE_STATUS_AGESIZE - 1), charge_status_age[i] );
			buffer->level = old_level;
			return;
		}
	}
#endif
	//Algorithm for discharge
	if (  !buffer->charging_enabled  )
	{
		if(nbComputationsOnBattery <= 2)
			nbComputationsOnBattery++;
		charge_curr_ref = 0;
		//discharge resistor correction
		volt_discharge_resistor = ( abs( ccurrent ) * batt_param->volt_discharge_res_coeff ) / 100;
		corrected_volt = volt + volt_discharge_resistor;
		
		//low temperature correction
		if(temp > 250)
			temp_correct_volt = 0;
		else
			temp_correct_volt = -( temp_correct_volt + ( ( batt_param->temp_correction_const * ( ( 250 - temp ) * abs( ccurrent ) ) ) / 10000 ) );

		//compute battery level
		if ( (corrected_volt - temp_correct_volt ) >= batt_param->full_volt_threshold ) {
			result = 100;
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->max_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->max_volt_threshold ) * 10 ) / batt_param->max_volt_dynslope ) + ( batt_param->max_volt_perc_start / 10 ) );
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->med_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->med_volt_threshold ) * 10 ) / batt_param->med_volt_dynslope ) + ( batt_param->med_volt_perc_start / 10 ) );
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->mid_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->mid_volt_threshold ) * 10 ) / batt_param->mid_volt_dynslope ) + ( batt_param->mid_volt_perc_start / 10 ) );
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->min_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->min_volt_threshold ) * 10 ) / batt_param->min_volt_dynslope ) + ( batt_param->min_volt_perc_start / 10 ) );
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->low_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->low_volt_threshold ) * 10 ) / batt_param->low_volt_dynslope ) + ( batt_param->low_volt_perc_start / 10 ) );
		} else if ( ( corrected_volt - temp_correct_volt ) >= batt_param->cri_volt_threshold ) {
			result = ( ( ( ( corrected_volt - temp_correct_volt - batt_param->cri_volt_threshold ) * 10 ) / batt_param->cri_volt_dynslope ) + ( batt_param->cri_volt_perc_start / 10 ) );
		} else {
			result = 0;
		}
	}
	//Algorithm for charge
	else
	{
		//first time: take last percentage
		if(charge_curr_ref == 0)
		{
			if(old_level<0)
				old_level = 50;
			charge_curr_ref = (old_level*(BATT_CAPACITY_PHOTON-100))/100;
			current_loaded_mAs=0;
			max_curr = 0;
			stop_charge_counter = 0;
		}else{
			unsigned int udelta_msec;
			mcurr_jiffies = jiffies;
			//increment charge current (convert to mAh)
			udelta_msec = (jiffies_to_msecs(mcurr_jiffies)-old_time);
			//if time delta > 20sec, dont compute there is an error!
			if(udelta_msec < 20000)
			{
				int delta_msec= (int)udelta_msec;
				int delta_mAs = ((old_current * delta_msec)/1000);
				current_loaded_mAs += delta_mAs;

				if ( debug_mask&DEBUG_LOG )
					printk("%s udelta_msec=%u delta_msec=%d old_current=%d delta_mAs=%d\n",
							__func__,udelta_msec,delta_msec,old_current,delta_mAs);
			}else{
				printk("%s udelta_msec=%u: jiffies corruption\n",__func__,udelta_msec);
			}
		}
		old_current = ccurrent;
		mold_jiffies = jiffies;
		old_time = jiffies_to_msecs(mold_jiffies); 
		
		if(ccurrent > max_curr)
			max_curr = ccurrent;
			
		//compute percentage VS total battery capacity
		result = ((charge_curr_ref + (current_loaded_mAs/3600))*100)/(BATT_CAPACITY_PHOTON-100);
		if ( debug_mask&DEBUG_LOG )
			printk("%s: charging, raw level=%d, ccurrent=%d, charge_curr_ref=%d, current_loaded_mAh=%ld, TOTAL charged=%ld\n",__func__,result,ccurrent,charge_curr_ref,(current_loaded_mAs/3600),(charge_curr_ref + (current_loaded_mAs/3600)));

		//if we compute it wrong, at least don't charge too much! (don't go below 100mA)
		if((ccurrent < 100) && (max_curr > 100))
			stop_charge_counter++;
		else
			stop_charge_counter = 0;
		//10 samples below 100mAh: time to stop charging!
		if(stop_charge_counter >= 10)
		{
			result = 100;
			printk("%s: charging, emergency stop, batt is full!\n",__func__);
		}
		nbComputationsOnBattery = 0;
	}

	//avoid out of bound values
	if (result > 99) {
		result = 100;
	} else if (result < 0) {
		result = 0;
	}
	//allocate raw result
	buffer->level = result;
	
	//general rule: dont allow variations more than 2% per sample
	if ( ( result > (old_level + 2) ) && (result < 98) && (old_level>0) ) {
		buffer->level = old_level + 2;
	}else if ( result < (old_level - 2) && (old_level>0) ) {
		buffer->level = old_level - 2;
	}
	//general rule: if result is below 5%, do like WinMo and advise Android to switch off!
	if( buffer->level <= 5)
		buffer->level = 0;

	//backup 
	old_level = buffer->level;
}

static void fix_batt_values(struct battery_info_reply *buffer) {                   

	/*if there are wrong values */                                              
	if ( buffer->batt_vol > 4250 )
		buffer->batt_vol = 4250;                                            
	if ( buffer->batt_vol < 2600 )
		buffer->batt_vol = 2600;
	if ( buffer->batt_current > 1000 )
		buffer->batt_current = 1000;                                        
	if ( buffer->batt_current < -1000 )
		buffer->batt_current = -1000;                                                     
	if ( buffer->batt_tempRAW > 637 )
		buffer->batt_temp = 637;                                        
	else if ( buffer->batt_tempRAW < 0 )
		buffer->batt_temp = 0;
	else
		buffer->batt_temp = buffer->batt_tempRAW;
}  

void printBattBuff(struct battery_info_reply *buffer,char *txt)
{
#if 0
	printk( "r0bin %s: batt_id=%d;volt=%d;tempRaw=%dC;temp=%dC;current=%d;discharge=%d;LEVEL=%d;charging src=%d;charging?%d;adc_range=%d\n",
			txt,buffer->batt_id,buffer->batt_vol,buffer->batt_tempRAW,buffer->batt_temp,
			buffer->batt_current,buffer->batt_discharge,buffer->level,buffer->charging_source,buffer->charging_enabled,htc_adc_range);
#endif
}


/* Photon battery data corrections */
/* r0bin: algorithms by pwel and munjeni */
static int htc_photon_batt_corr( struct battery_info_reply *buffer )
{
	int av_index;

	/* battery voltage, pwel's algorithm */
	buffer->batt_vol = (15871*buffer->batt_vol)/(batt_vref*10);  //( ( buffer->batt_vol * 5200 ) / htc_adc_range );  
	
	/* convert readed value to mA, pwel's algorithm */
	buffer->batt_current = (237* ( buffer->batt_current - ((3025*buffer->batt_discharge)/1000)))/1000;

	/* cardsharing algo on temp */
	av_index = ( buffer->batt_tempRAW );
	buffer->batt_tempRAW = htc_battery_temperature_lut( av_index );

	return 0;
}


static int htc_get_batt_smem_info(struct battery_info_reply *buffer)
{
	volatile struct htc_batt_info_u16 *batt_16 = NULL;
	struct msm_dex_command dex;
	
	//send DEX to update smem values
	dex.cmd = PCOM_GET_BATTERY_DATA;
	msm_proc_comm_wince(&dex, 0);
	mutex_lock(&htc_batt_info.lock);

	//now read latest values
	batt_16 = (void *)(MSM_SHARED_RAM_BASE + htc_batt_info.resources->smem_offset);
	buffer->batt_vol = batt_16->batt_vol;
	buffer->batt_current = batt_16->batt_charge;
	buffer->batt_tempRAW = batt_16->batt_temp;
	buffer->batt_id = batt_16->batt_id;
	buffer->batt_discharge = batt_16->batt_discharge;
	//printBattBuff(buffer,"RAW VALUES");
	
	return 0;
}

/* usage: backup batteryinfo data */
void memcpyBattInfo(struct battery_info_reply *dest, struct battery_info_reply *source)
{
	dest->batt_id = source->batt_id;
	dest->batt_vol = source->batt_vol;
	dest->batt_temp = source->batt_temp;
	dest->batt_current = source->batt_current;
	dest->batt_discharge = source->batt_discharge;
	dest->level = source->level;
	dest->charging_source = source->charging_source;
	dest->charging_enabled = source->charging_enabled;
	dest->full_bat = source->full_bat;
	dest->batt_tempRAW = source->batt_tempRAW;
}

/* this is the main function that computes battery percentage */
static int htc_get_batt_info(struct battery_info_reply *buffer_answer)
{
	unsigned int time_now;
	struct battery_info_reply buffer;
	
	// sanity checks
	if ( buffer_answer == NULL )
		return -EINVAL;
	if ( !htc_batt_info.resources || !htc_batt_info.resources->smem_offset ) {
		BATT_ERR("smem_offset not set\n" );
		return -EINVAL;
	}

	//don't stress our driver! minimum interval = 1sec
	time_now = jiffies_to_msecs(jiffies);
	if ( debug_mask&DEBUG_LOG )
		printk("%s: diff=%u ms, time_now=%u ms, time_stamp=%u ms, jiffies=%lu\n",__func__,(time_now - time_stamp),time_now, time_stamp,jiffies);

	if(time_stamp && ((time_now - time_stamp)<1000))
	{
		//if yes, no need to compute again: just copy previous computed values and exit!
		memcpyBattInfo(buffer_answer,&old_batt_info);
		if ( debug_mask&DEBUG_LOG )
			printk("%s: don't stress DEX, time diff only %u ms\n",__func__,(time_now - time_stamp));
		return 0;
	}

	//read raw SMEM values 
	htc_get_batt_smem_info(&buffer);
	//htc_get_batt_smem_info_5times(&buffer);

	time_stamp = jiffies_to_msecs(jiffies);
/*
	//check if values are similar to previous poll 
	if( (buffer->batt_vol == old_batt_info.batt_vol) && 
		(buffer->batt_tempRAW == old_batt_info.batt_tempRAW) &&
		(buffer->batt_current == old_batt_info.batt_current) &&
		(buffer->batt_discharge == old_batt_info.batt_discharge) &&
		(buffer->batt_id == old_batt_info.batt_id)){
		//if yes, no need to compute again: just copy previous computed values and exit!
		memcpyBattInfo(buffer,&old_batt_info);
		printk("%s: smem values identical, no need to compute again, level=%d\n",__func__,buffer->level);
		return 0;
	}
*/
	//if not, START computing!
	//1st, calculate the avarage raw voltage, in order to avoid strong voltage variationst, 
	maf_add_value( buffer.batt_vol );
	buffer.batt_vol = maf_get_avarage();
	
	//check if charger is enabled
	if (gpio_get_value(htc_batt_info.resources->gpio_charger_enable) == 0) {
		buffer.charging_enabled = 1;
		if(gpio_get_value(htc_batt_info.resources->gpio_charger_fast_dis) == 0)
		{
			buffer.charging_source = CHARGER_AC;
		}else{
			buffer.charging_source = CHARGER_USB;
		}
	} else {
		buffer.charging_enabled = 0;
		buffer.charging_source = CHARGER_BATTERY;
	}

	/* update cable status */
	mutex_unlock(&htc_batt_info.lock);
	htc_cable_status_update(__func__);
	mutex_lock(&htc_batt_info.lock);
	//should never happens
	if(buffer.charging_source != htc_batt_info.rep.charging_source)
	{
		printk("%s: cable status really needed an update: gpio detected source=%d, cable detected source=%d\n",__func__,buffer.charging_source, htc_batt_info.rep.charging_source);	
		buffer.charging_source = htc_batt_info.rep.charging_source;
		buffer.charging_enabled = htc_batt_info.rep.charging_enabled;
	}

	/* get real volt, temp and current values */
	htc_photon_batt_corr( &buffer );
	printBattBuff( &buffer,"DECODED VALUES");
	/* fix values in case they are out of bounds */
	fix_batt_values( &buffer);
	printBattBuff( &buffer,"FIX VALUES");
	/* compute battery level based on battery values */
	htc_battery_level_compute( &buffer );
	printBattBuff( &buffer,"COMPUTATION FINISHED");
	
	/* backup values for next time */
	memcpyBattInfo( &old_batt_info, &buffer);
	/* return computed values to caller */
	memcpyBattInfo( buffer_answer, &buffer);
	
/*	printk( "%s called: batt_id=%d;volt=%d;tempRaw=%dC;temp=%dC;current=%d;discharge=%d;LEVEL=%d;charging src=%d;charging?%d;adc_range=%d\n",
			__func__,buffer.batt_id,buffer.batt_vol,buffer.batt_tempRAW,buffer.batt_temp,
			buffer.batt_current,buffer.batt_discharge,buffer.level,buffer.charging_source,buffer.charging_enabled,htc_adc_range);
*/
	mutex_unlock(&htc_batt_info.lock);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int htc_power_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;

	mutex_lock(&htc_batt_info.lock);
	charger = htc_batt_info.rep.charging_source;
	mutex_unlock(&htc_batt_info.lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int htc_battery_get_charging_status(void)
{
	u32 level;
	charger_type_t charger;
	int ret;

	mutex_lock(&htc_batt_info.lock);
	charger = htc_batt_info.rep.charging_source;

	switch (charger) {
	case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:
	case CHARGER_AC:
		level = htc_batt_info.rep.level;
		if (level == 100)
			ret = POWER_SUPPLY_STATUS_FULL;
		else
			ret = POWER_SUPPLY_STATUS_CHARGING;
		break;
	default:
		ret = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	mutex_unlock(&htc_batt_info.lock);
	return ret;
}

static int htc_battery_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = htc_battery_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		if (htc_batt_info.rep.batt_temp >= 500 ||  htc_batt_info.rep.batt_temp <= 0)
		{
			val->intval =  POWER_SUPPLY_HEALTH_OVERHEAT;
			printk("%s, Warning: power supply overheating: %d°C\n",__func__,htc_batt_info.rep.batt_temp);
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = htc_batt_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&htc_batt_info.lock);
		val->intval = htc_batt_info.rep.level;
		mutex_unlock(&htc_batt_info.lock);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define HTC_BATTERY_ATTR(_name)							\
{										\
	.attr = { .name = #_name, .mode = S_IRUGO, .owner = THIS_MODULE },	\
	.show = htc_battery_show_property,					\
	.store = NULL,								\
}

static struct device_attribute htc_battery_attrs[] = {
	HTC_BATTERY_ATTR(batt_id),
	HTC_BATTERY_ATTR(batt_vol),
	HTC_BATTERY_ATTR(batt_temp),
	HTC_BATTERY_ATTR(batt_current),
	HTC_BATTERY_ATTR(charging_source),
	HTC_BATTERY_ATTR(charging_enabled),
	HTC_BATTERY_ATTR(full_bat),
};

enum {
	BATT_ID = 0,
	BATT_VOL,
	BATT_TEMP,
	BATT_CURRENT,
	CHARGING_SOURCE,
	CHARGING_ENABLED,
	FULL_BAT,
};

static int htc_battery_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(htc_battery_attrs); i++) {
		rc = device_create_file(dev, &htc_battery_attrs[i]);
		if (rc)
			goto htc_attrs_failed;
	}

	goto succeed;

htc_attrs_failed:
	while (i--)
		device_remove_file(dev, &htc_battery_attrs[i]);
succeed:
	return rc;
}

static ssize_t htc_battery_show_property(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - htc_battery_attrs;

	/* check cache time to decide if we need to update */
	if (htc_batt_info.update_time &&
            time_before(jiffies, htc_batt_info.update_time +
                                msecs_to_jiffies(cache_time)))
                goto dont_need_update;

	if (htc_get_batt_info(&htc_batt_info.rep) < 0) {
		BATT_ERR("%s: get_batt_info failed!!!\n", __FUNCTION__);
	} else {
		htc_batt_info.update_time = jiffies;
	}
dont_need_update:

	mutex_lock(&htc_batt_info.lock);
	switch (off) {
	case BATT_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_id);
		break;
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_vol);
		break;
	case BATT_TEMP:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_temp);
		break;
	case BATT_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.batt_current);
		break;
	case CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_source);
		break;
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.charging_enabled);
		break;
	case FULL_BAT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			       htc_batt_info.rep.full_bat);
		break;
	default:
		i = -EINVAL;
	}
	mutex_unlock(&htc_batt_info.lock);

	return i;
}


static int htc_battery_thread(void *data)
{
#define BATT_SAMPLES_NUMBER	30
	struct battery_info_reply htc_batt_data_smooth;
	int percent_tab[BATT_SAMPLES_NUMBER];
	int tab_index = 0;
	int i;
	
	//struct battery_info_reply buffer;
	daemonize("battery");
	allow_signal(SIGKILL);

	while ((!signal_pending((struct task_struct *)current))) {
		//when battery is suspended, work slower to save power
		if(bat_suspended)
			msleep(10000);
		else
			msleep(1000);

#ifdef NO_BATTERY_COMPUTATION_IN_SUSPEND_MODE		
		//dont work in sleep mode!
		if (bat_suspended)
		{
			//update the batt level with the samples we have
			if(tab_index >= 1)
			{
				int smooth_lvl = 0;
				//compute average level
				for(i=0;i<tab_index;i++)
					smooth_lvl += percent_tab[i];		
				smooth_lvl = smooth_lvl / tab_index;
				printk("%s, %d samples only, average percentage=%d\n",__func__,tab_index,smooth_lvl);
				//smooth batt level
				htc_batt_info.rep.level = smooth_lvl;
				htc_batt_info.update_time = jiffies;
				//update power supply class
				power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
			}
			tab_index = 0;
			//put on sleep, waiting for signal to wake up
			printk("%s, put batt thread on sleep!\n",__func__);
			wait_for_completion(&batt_thread_can_start);
			printk("%s, batt_thread_can_start!\n",__func__);
		}else
#endif
		{
			//read raw SMEM values
			htc_get_batt_smem_info(&htc_batt_data_smooth);
			mutex_unlock(&htc_batt_info.lock);
				
			//grab charging status from global var, it is updated by IRQ 
			htc_batt_data_smooth.charging_enabled = htc_batt_info.rep.charging_enabled;
			htc_batt_data_smooth.charging_source = htc_batt_info.rep.charging_source;
			
			//get real volt, temp and current values 
			htc_photon_batt_corr( &htc_batt_data_smooth );
			//fix values in case they are out of bounds 
			fix_batt_values( &htc_batt_data_smooth);
			//compute battery level based on battery values 
			htc_battery_level_compute( &htc_batt_data_smooth );
			//record this value
			percent_tab[tab_index] = htc_batt_data_smooth.level;
			tab_index++;
			//update official batt info, but not the level!
			mutex_lock(&htc_batt_info.lock);
			htc_batt_info.rep.batt_id	=	htc_batt_data_smooth.batt_id;
			htc_batt_info.rep.batt_vol	=	htc_batt_data_smooth.batt_vol;	
			htc_batt_info.rep.batt_temp	=	htc_batt_data_smooth.batt_temp;	
			htc_batt_info.rep.batt_current	=	htc_batt_data_smooth.batt_current;	
			htc_batt_info.rep.batt_discharge	=	htc_batt_data_smooth.batt_discharge;
			htc_batt_info.update_time = jiffies;
			mutex_unlock(&htc_batt_info.lock);
			//if we got our 30 samples, it's time to compute the average level
			if(tab_index >= BATT_SAMPLES_NUMBER)
			{
				int smooth_lvl = 0;
				tab_index = 0;
				//compute average level
				for(i=0;i<BATT_SAMPLES_NUMBER;i++)
					smooth_lvl += percent_tab[i];		
				smooth_lvl = smooth_lvl / BATT_SAMPLES_NUMBER;
				if ( debug_mask&DEBUG_LOG )
					printk("%s, %d samples ready, average percentage=%d\n",__func__,BATT_SAMPLES_NUMBER,smooth_lvl);
				//smooth batt level
				htc_batt_info.rep.level = smooth_lvl;
				htc_batt_info.update_time = jiffies;
			}
			//report batt info changed every 5 samples (update temp, volt, current)
			if(tab_index%5 == 0) 
				power_supply_changed(&htc_power_supplies[CHARGER_BATTERY]);
		}
	}

	return 0;
}
	
	
static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc;
	htc_batt_info.resources = (smem_batt_t *)pdev->dev.platform_data;

	/* sanity checks */
	if (!htc_batt_info.resources) {
		BATT_ERR("%s: no pdata resources!\n", __FUNCTION__);
		return -EINVAL;
	}

	/* init battery gpio */
	if ((rc = init_batt_gpio()) < 0) {
		BATT_ERR("%s: init battery gpio failed!\n", __FUNCTION__);
		return rc;
	}
		
	/* init structure data member */
	htc_batt_info.update_time 	= jiffies;
	/* A9 will shutdown the phone if battery is pluged out, so this value is always 1 */
	htc_batt_info.present 		= 1; 
	
	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(htc_power_supplies); i++) {
		rc = power_supply_register(&pdev->dev, &htc_power_supplies[i]);
		if (rc)
			BATT_ERR("%s: Failed to register power supply (%d)", __func__, rc);
	}

	/* create htc detail attributes */
	htc_battery_create_attrs(htc_power_supplies[CHARGER_BATTERY].dev);

	/* init static battery settings */
	if ( init_battery_settings( &htc_batt_info.rep ) < 0)
		BATT_ERR("%s: init battery settings failed\n", __FUNCTION__);

	is_battery_initialized = 1;

	htc_batt_info.update_time = jiffies;
	
	/* lauch thread to poll battery status every x seconds */
	kernel_thread(htc_battery_thread, NULL, CLONE_KERNEL);
	
	not_yet_started = false;
	return 0;
}

#if CONFIG_PM
static int htc_battery_suspend(struct platform_device* device, pm_message_t mesg)
{
#ifdef NO_BATTERY_COMPUTATION_IN_SUSPEND_MODE
	INIT_COMPLETION(batt_thread_can_start);
#endif
	bat_suspended = 1;
	return 0;
}

static int htc_battery_resume(struct platform_device* device)
{
	bat_suspended = 0;
	maf_clear();
#ifdef NO_BATTERY_COMPUTATION_IN_SUSPEND_MODE
	// notify ready for action
    complete(&batt_thread_can_start);
#endif
	return 0; 
}
#else
 #define htc_battery_suspend NULL
 #define htc_battery_resume NULL
#endif

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = htc_battery_suspend,
	.resume = htc_battery_resume,
};


static void usb_status_notifier_func(int online)
{
	printk("%s: NOTIFIER Connected usb == %d\n", __func__,online);
	if(online == 0)
		g_usb_online = 0;
	if(online == 1)
		g_usb_online = 1;
}
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery",
	.func = usb_status_notifier_func,
};


static int __init htc_battery_init(void)
{
	// this used to be WAKE_LOCK_SUSPEND, but make it an idle lock in order to
	// prevent msm_sleep() try to collapse arm11 (using idle_sleep mode) several
	// times a second which sooner or later get's the device to freeze when usb
	// is connected
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_IDLE, "vbus_present");
	mutex_init(&htc_batt_info.lock);
	usb_register_notifier(&usb_status_notifier);
	g_vbus_notifier_work_queue = create_workqueue("vbus-notifier");
	if (g_vbus_notifier_work_queue == NULL) {
		printk("%s error: vbus notifier workqueue creation failed\n",__func__);
	}
	platform_driver_register(&htc_battery_driver);
	BATT("HTC Battery Driver initialized\n");

	return 0;
}

late_initcall(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
