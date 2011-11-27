#ifndef _HTC_BATTERY_SMEM_DEF_H_
#define _HTC_BATTERY_SMEM_DEF_H_

struct smem_battery_resources {
	unsigned short gpio_battery_detect;
	unsigned short gpio_charger_enable;
	unsigned short gpio_charger_current_select;
	unsigned short gpio_ac_detect;
	unsigned short gpio_charger_fast_dis;
	unsigned short gpio_charger_fast_en;
	unsigned smem_offset;
	unsigned short smem_field_size;
};

typedef struct smem_battery_resources smem_batt_t;

typedef enum {
	DISABLE = 0,
	ENABLE_SLOW_CHG,
	ENABLE_FAST_CHG
} batt_ctl_t;

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

struct battery_info_reply {
	u32 batt_id;		/* Battery ID from ADC */
	u32 batt_vol;		/* Battery voltage from ADC */
	u32 batt_temp;		/* Battery Temperature (C)corrected value from formula and ADC */
	s32 batt_current;	/* Battery charge current from ADC */
	s32 batt_discharge;	/* Battery discharge current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 full_bat;		/* Full capacity of battery (mAh) */
	u32 batt_tempRAW;	/* Battery Temperature (C) from formula and ADC */
};
 
struct sBattery_Parameters
{
	int battery_capacity;               /* Battery capacity (mAh) */
	int termination_current;            /* Charge termination current (typically 3% of battery_capacity) */
	int temp_correction;
	int temp_correction_const;
	int volt_discharge_res_coeff;

        //definition of points
	int cri_volt_threshold;        	    /* Point 1 - Lowest voltage value the critical voltage part of the graph */
	int low_volt_threshold;             /* Point 2 Lowest voltage value the low voltage part of the graph */
	int min_volt_threshold;             /* Point 3 Mininimum voltage value the mid voltage part of the graph */
	int mid_volt_threshold;             /* Point 4 Lowest voltage value the mid voltage part of the graph */
	int med_volt_threshold;             /* Point 5 Lowest voltage value the mid voltage part of the graph */
	int max_volt_threshold;             /* Point 6 Lowest voltage value the max voltage part of the graph */
	int full_volt_threshold;     /* only for define when battery is on 99% or higher almost full voltage of the battery */
	// percent cure start min percentage (% x 10)
	int cri_volt_perc_start; 
	int low_volt_perc_start;
	int min_volt_perc_start;
	int mid_volt_perc_start;
	int med_volt_perc_start;
	int max_volt_perc_start;
	//dynamic slope (centi-unti) (unit x 10)
	int cri_volt_dynslope;
	int low_volt_dynslope;
	int min_volt_dynslope;
	int mid_volt_dynslope;
	int med_volt_dynslope;
	int max_volt_dynslope;
} ;

/*  Values for photon (valitated by photon community) */
static const struct sBattery_Parameters sBatParams_Photon_1200mAh =
{
    .battery_capacity =			1200,
    .termination_current =		36, //3% of batt capacity, but we will never charge until there
    .temp_correction =			0,	//not used
    .temp_correction_const =		26, //correction for low temp
    .volt_discharge_res_coeff =		27,	//correction for discharge	
    .cri_volt_threshold =		3300,	
    .low_volt_threshold =		3600,
    .min_volt_threshold =		3670,
    .mid_volt_threshold =		3750,
    .med_volt_threshold =		3815,
    .max_volt_threshold =		3950,
    .full_volt_threshold =		4100,
    .cri_volt_perc_start =		15,
    .low_volt_perc_start =		50,
    .min_volt_perc_start =		100,
    .mid_volt_perc_start =		300,
    .med_volt_perc_start =		570,
    .max_volt_perc_start =		765,
    .cri_volt_dynslope =		860,
    .low_volt_dynslope =		140,
    .min_volt_dynslope =		40,
    .mid_volt_dynslope =		24,
    .med_volt_dynslope =		69,
    .max_volt_dynslope =		64,
};

static const struct sBattery_Parameters* sBatParams_photon[] =
{
	&sBatParams_Photon_1200mAh,           /* batt_vendor = 1 */
};

#endif 