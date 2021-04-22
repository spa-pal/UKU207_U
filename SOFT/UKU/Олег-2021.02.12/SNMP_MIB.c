/*----------------------------------------------------------------------------
 *      RL-ARM - TCPnet
 *----------------------------------------------------------------------------
 *      Name:    SNMP_MIB.C
 *      Purpose: SNMP Agent Management Information Base Module
 *      Rev.:    V4.12
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2010 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <Net_Config.h>
#include "LPC17xx.H"
#include "main.H"
#include "control.H"
#include "snmp_data_file.h" 

/* snmp_demo.c */
extern U8   get_button (void);
extern void LED_out (U32 val);
extern BOOL LCDupdate;
extern U8   lcd_text[2][16+1];

/* System */
extern U32  snmp_SysUpTime;

/* Local variables */
//static U8   LedOut;
//static U8   KeyIn;

/* MIB Read Only integer constants */
static const U8 sysServices = 79;
static const U16 sysMainsVoltage = 220;
static const U8 displayPsuQauntity = 2;
static const U8 TestForTableValues = 57;

 char* const aaa = "Novosibirsk, Russia";

int a_;
char aa_;
char* aaa_="abc";

/* MIB Entry event Callback functions. */
//static void write_leds (int mode);
//static void read_key (int mode);
//static void upd_display (int mode);

#ifndef UKU_KONTUR
/*----------------------------------------------------------------------------
 *      MIB Data Table
 *---------------------------------------------------------------------------*/

const MIB_ENTRY snmp_mib[] = {

  /* ---------- System MIB ----------- */

  /* SysDescr Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 1, 0},      MIB_STR("First ARM SNMP agent for SibPromAutomatika"),     NULL },
  /* SysObjectID Entry */
  { MIB_OBJECT_ID | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 2, 0},	    MIB_STR("\x2b\x06\x01\x04\x01\x82\x83\x1F\x0e\x01"),    NULL },
  /* SysUpTime Entry */
  { MIB_TIME_TICKS | MIB_ATR_RO,     8, {OID0(1,3), 6, 1, 2, 1, 1, 3, 0},    4, &snmp_SysUpTime,    NULL },
  /* SysContact Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 4, 0},    MIB_STR("Skype:danilov_aa"),    NULL },
  /* SysName Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		    8, {OID0(1,3), 6, 1, 2, 1, 1, 5, 0},    MIB_STR("UKU203LAN"),    NULL },
  /* SysLocation Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		     8, {OID0(1,3), 6, 1, 2, 1, 1, 6, 0},    MIB_STR("Novosibirsk, Russia"),    NULL },
  /* SysServices Entry */
  { MIB_INTEGER | MIB_ATR_RO,			    8, {OID0(1,3), 6, 1, 2, 1, 1, 7, 0},    MIB_INT(sysServices),    NULL },

  /* ---------- Experimental MIB ----------- */

	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_0 , 0},			MIB_INT(snmp_spc_trap_value_0),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_1 , 0},			MIB_INT(snmp_spc_trap_value_1),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_2 , 0},			MIB_INT(snmp_spc_trap_value_2),     NULL},


  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_CODE, 0},  	MIB_INT(snmp_device_code),  		NULL},   				//код устройства
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_SERIAL, 0},	MIB_INT(snmp_sernum),	  		NULL },				//серийный номер	
  	{ MIB_OCTET_STR, 				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_LOCATION, 0},  	MIB_STR(snmp_location),  		snmp_location_write},	//местоположение устройства
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBAT, 0}, 	MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBPS, 0},	MIB_INT(snmp_numofbps),  		NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFINV, 0},	MIB_INT(snmp_numofinv),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFAVT, 0}, 	MIB_INT(snmp_numofavt),  			NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFDT, 0},	MIB_INT(snmp_numofdt),  			NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFSK, 0},	MIB_INT(snmp_numofsk),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFEVENTS, 0},MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей
//	{ MIB_OCTET_STR, 			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_MODEL_NAME, 0},	MIB_INT(snmp_model_name),  		snmp_model_name_write},	//название ИБЭПа


  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE, 0},  	MIB_INT(snmp_mains_power_voltage), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_FREQUENCY, 0},  MIB_INT(snmp_mains_power_frequency),NULL},	//частота сети
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_STATUS, 0},  	MIB_INT(snmp_mains_power_status),  NULL},	//состояние сети 
 	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_ALARM, 0},  	MIB_INT(snmp_mains_power_alarm),  	NULL},	//аварии сети
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEA, 0},  	MIB_INT(snmp_mains_power_voltage_phaseA), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEB, 0},  	MIB_INT(snmp_mains_power_voltage_phaseB), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEC, 0},  	MIB_INT(snmp_mains_power_voltage_phaseC), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_ENERGOMETR_VOLT, 0},  	MIB_INT(volta_short), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_ENERGOMETR_CURR, 0},  	MIB_INT(curr_short), NULL},	//напряжение сети	
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_ENERGOMETR_POWER, 0},  	MIB_INT(power_int), NULL},	//напряжение сети	

	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_VOLTAGE, 0},  				MIB_INT(snmp_load_voltage),  		NULL},	//напряжение нагрузки
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_CURRENT, 0},  				MIB_INT(snmp_load_current),  		NULL},	//ток нагрузки

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 1},  			MIB_INT(snmp_bps_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 2},  			MIB_INT(snmp_bps_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 3},  			MIB_INT(snmp_bps_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 4},  			MIB_INT(snmp_bps_number[3]),  	NULL},	//Номер БПСа
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 5},  			MIB_INT(snmp_bps_number[4]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 6},  			MIB_INT(snmp_bps_number[5]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 7},  			MIB_INT(snmp_bps_number[6]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 8},  			MIB_INT(snmp_bps_number[7]),  	NULL},	//Номер БПСа
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 1},  			MIB_INT(snmp_bps_voltage[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 2},  			MIB_INT(snmp_bps_voltage[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 3},  			MIB_INT(snmp_bps_voltage[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 4},  			MIB_INT(snmp_bps_voltage[3]),  	NULL},	//Напряжение БПС3
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 5},  			MIB_INT(snmp_bps_voltage[4]),  	NULL},	//Напряжение БПС4
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 6},  			MIB_INT(snmp_bps_voltage[5]),  	NULL},	//Напряжение БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 7},  			MIB_INT(snmp_bps_voltage[6]),  	NULL},	//Напряжение БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 8},  			MIB_INT(snmp_bps_voltage[7]),  	NULL},	//Напряжение БПС7
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 1},  			MIB_INT(snmp_bps_current[0]),  	NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 2},  			MIB_INT(snmp_bps_current[1]),  	NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 3},  			MIB_INT(snmp_bps_current[2]),  	NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 4},  			MIB_INT(snmp_bps_current[3]),  	NULL},	//Ток БПС4
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 5},  			MIB_INT(snmp_bps_current[4]),  	NULL},	//Ток БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 6},  			MIB_INT(snmp_bps_current[5]),  	NULL},	//Ток БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 7},  			MIB_INT(snmp_bps_current[6]),  	NULL},	//Ток БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 8},  			MIB_INT(snmp_bps_current[7]),  	NULL},	//Ток БПС8
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 1},  		MIB_INT(snmp_bps_temperature[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 2},  		MIB_INT(snmp_bps_temperature[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 3},  		MIB_INT(snmp_bps_temperature[2]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 4},  		MIB_INT(snmp_bps_temperature[3]),  NULL},	//Ток БПС4
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 5},  		MIB_INT(snmp_bps_temperature[4]),  NULL},	//Ток БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 6},  		MIB_INT(snmp_bps_temperature[5]),  NULL},	//Ток БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 7},  		MIB_INT(snmp_bps_temperature[6]),  NULL},	//Ток БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 8},  		MIB_INT(snmp_bps_temperature[7]),  NULL},	//Ток БПС8
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 1},  			MIB_INT(snmp_bps_stat[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 2},  			MIB_INT(snmp_bps_stat[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 3},  			MIB_INT(snmp_bps_stat[2]),  NULL},			//Состояние БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 4},  			MIB_INT(snmp_bps_stat[3]),  NULL},			//Состояние БПС4
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 5},  			MIB_INT(snmp_bps_stat[4]),  NULL},			//Состояние БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 6},  			MIB_INT(snmp_bps_stat[5]),  NULL},			//Состояние БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 7},  			MIB_INT(snmp_bps_stat[6]),  NULL},			//Состояние БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 8},  			MIB_INT(snmp_bps_stat[7]),  NULL},			//Состояние БПС8
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 1},  		MIB_INT(bps[0]._vent_resurs),  NULL},		//Ресурс вентилятора БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 2},  		MIB_INT(bps[1]._vent_resurs),  NULL},		//Ресурс вентилятора БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 3},  		MIB_INT(bps[2]._vent_resurs),  NULL},		//Ресурс вентилятора БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 4},  		MIB_INT(bps[3]._vent_resurs),  NULL},		//Ресурс вентилятора БПС4
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 5},  		MIB_INT(bps[4]._vent_resurs),  NULL},		//Ресурс вентилятора БПС5
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 6},  		MIB_INT(bps[5]._vent_resurs),  NULL},		//Ресурс вентилятора БПС6
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 7},  		MIB_INT(bps[6]._vent_resurs),  NULL},		//Ресурс вентилятора БПС7
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VENTRESURS, 8},  		MIB_INT(bps[7]._vent_resurs),  NULL},		//Ресурс вентилятора БПС8
#endif
 
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 1},  			MIB_INT(snmp_inv_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 2},  			MIB_INT(snmp_inv_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_NUMBER, 3},  			MIB_INT(snmp_inv_number[2]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 1},  			MIB_INT(snmp_inv_voltage[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 2},  			MIB_INT(snmp_inv_voltage[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_VOLTAGE, 3},  			MIB_INT(snmp_inv_voltage[2]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 1},  			MIB_INT(snmp_inv_current[0]),  	NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 2},  			MIB_INT(snmp_inv_current[1]),  	NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_CURRENT, 3},  			MIB_INT(snmp_inv_current[2]),  	NULL},	//Ток БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 1},  		MIB_INT(snmp_inv_temperature[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 2},  		MIB_INT(snmp_inv_temperature[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_TEMPERATURE, 3},  		MIB_INT(snmp_inv_temperature[2]),  NULL},	//Ток БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 1},  			MIB_INT(snmp_inv_stat[0]),  NULL},			//Состояние БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 2},  			MIB_INT(snmp_inv_stat[1]),  NULL},			//Состояние БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_INV, DISPLAY_INV_ENTRY_STATUS, 3},  			MIB_INT(snmp_inv_stat[2]),  NULL},			//Состояние БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 1},  					MIB_INT(snmp_bat_number[0]),  	NULL},	//Номер батареи №1
#ifndef UKU_ZVU
 	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 2},  					MIB_INT(snmp_bat_number[1]),  	NULL},	//Напряжение батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 1},  				MIB_INT(snmp_bat_voltage[0]),  	NULL},	//Напряжение батареи №1
#ifndef UKU_ZVU	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 2},  				MIB_INT(snmp_bat_voltage[1]),  	NULL},	//Напряжение батареи №2
#endif
 	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 1},  				MIB_INT(snmp_bat_current[0]),  	NULL},	//Ток батареи №1
#ifndef UKU_ZVU	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 2},  				MIB_INT(snmp_bat_current[1]),  	NULL},	//Ток батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 1},  			MIB_INT(snmp_bat_temperature[0]),	NULL},	//Температура батареи №1
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 2},  			MIB_INT(snmp_bat_temperature[1]),	NULL},	//Температура батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 1},  				MIB_INT(snmp_bat_capacity[0]),  	NULL},	//Ёмкость батареи №1 (номинальная)
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 2},  				MIB_INT(snmp_bat_capacity[1]),  	NULL},	//Ёмкость батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 1},  					MIB_INT(snmp_bat_charge[0]),  	NULL},	//Заряд батареи №1 (для ЗВУ в процентах)
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 2},  					MIB_INT(snmp_bat_charge[1]),  	NULL},	//Заряд батареи №2
#endif
 	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 1},  					MIB_INT(snmp_bat_status[0]),  	NULL},	//Статус батареи №1
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 2},  					MIB_INT(snmp_bat_status[1]),  	NULL},	//Статус батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_FLAG, 1},  				MIB_INT(snmp_bat_flag[0]),  	NULL},	//флаги батареи №1 //o_1
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_FLAG, 2},  				MIB_INT(snmp_bat_flag[1]),  	NULL},	//флаги батареи №2 //o_1
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_REMTIME, 1},  				MIB_INT(snmp_bat_rem_time[0]),  	NULL},	//Время прогнозируемого полного разряда батареи №1
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_REMTIME, 2},  				MIB_INT(snmp_bat_rem_time[1]),  	NULL},	//Время прогнозируемого полного разряда батареи №2
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_PART_VOLTAGE, 1},  			MIB_INT(snmp_bat_part_voltage[0]),	NULL},	//Напряжение средней точки батареи №1
#ifndef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_PART_VOLTAGE, 2},  			MIB_INT(snmp_bat_part_voltage[1]),	NULL},	//Напряжение средней точки батареи №2
#endif

//#ifdef UKU_ZVU
//#endif


#ifndef UKU_ZVU



#endif

	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_STAT , 0},					MIB_INT(snmp_spc_stat),     NULL},
//	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
//	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE , 0},			MIB_INT(snmp_spc_trap_value),     NULL},


	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_ANSWER, 0},					MIB_INT(snmp_command),  	snmp_command_execute},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_PARAMETR, 0},					MIB_INT(snmp_command_parametr),  	NULL},		//номер первого бпса

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 1},  			MIB_INT(snmp_avt_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 2},  			MIB_INT(snmp_avt_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 3},  			MIB_INT(snmp_avt_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 4},  			MIB_INT(snmp_avt_number[3]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 5},  			MIB_INT(snmp_avt_number[4]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 6},  			MIB_INT(snmp_avt_number[5]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 7},  			MIB_INT(snmp_avt_number[6]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 8},  			MIB_INT(snmp_avt_number[7]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 9},  			MIB_INT(snmp_avt_number[8]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 10},  			MIB_INT(snmp_avt_number[9]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 11},  			MIB_INT(snmp_avt_number[10]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 12},  			MIB_INT(snmp_avt_number[11]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 1},  			MIB_INT(snmp_avt_stat[0]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 2},  			MIB_INT(snmp_avt_stat[1]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 3},  			MIB_INT(snmp_avt_stat[2]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 4},  			MIB_INT(snmp_avt_stat[3]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 5},  			MIB_INT(snmp_avt_stat[4]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 6},  			MIB_INT(snmp_avt_stat[5]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 7},  			MIB_INT(snmp_avt_stat[6]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 8},  			MIB_INT(snmp_avt_stat[7]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 9},  			MIB_INT(snmp_avt_stat[8]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 10},  			MIB_INT(snmp_avt_stat[9]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 11},  			MIB_INT(snmp_avt_stat[10]),  		NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 12},  			MIB_INT(snmp_avt_stat[11]),  		NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_A, 0},		MIB_INT(snmp_energy_vvod_phase_a), NULL},	//напряжение фазы A ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_B, 0},		MIB_INT(snmp_energy_vvod_phase_b), NULL},	//напряжение фазы B ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_C, 0},		MIB_INT(snmp_energy_vvod_phase_c), NULL},	//напряжение фазы C ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_A, 0},		MIB_INT(snmp_energy_pes_phase_a), NULL},	//напряжение фазы A ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_B, 0},		MIB_INT(snmp_energy_pes_phase_b), NULL},	//напряжение фазы B ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_C, 0},		MIB_INT(snmp_energy_pes_phase_c), NULL},	//напряжение фазы C ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_TOTAL_ENERGY, 0},		MIB_INT(snmp_energy_total_energy), NULL},	//показания счетчика, потребленная энергия
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_CURRENT_ENERGY, 0},		MIB_INT(snmp_energy_current_energy), NULL},	//показания счетчика, потребляемая энергия
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_INPUT_VOLTAGE, 0},		MIB_INT(snmp_energy_input_voltage), NULL},	//входное напряжение при однофазном питании



	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 4, 1, 0},  MIB_INT(NUMBAT),  NULL},	//количество введенных батарей

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},	     MIB_STR("Novosibirsk, Russia"),     NULL},
	{ MIB_INTEGER, 			13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 5, 0},	     MIB_INT(displayPsuQauntity),     NULL},
 /* { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 1, 0},  MIB_INT(plazma_mib),  NULL},
  { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 2, 0},  MIB_INT(plazma_mib1),  NULL},
  { MIB_INTEGER,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 1, 2, 0},    MIB_INT(LPC_RTC->SEC),    NULL}, */
  
	


	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 1},  			MIB_STR(&snmp_log[0][0]),  	NULL},	//Первое событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 2},  			MIB_STR(&snmp_log[1][0]),  	NULL},	//2-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 3},  			MIB_STR(&snmp_log[2][0]),  	NULL},	//3-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 4},  			MIB_STR(&snmp_log[3][0]),  	NULL},	//4-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 5},  			MIB_STR(&snmp_log[4][0]),  	NULL},	//5-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 6},  			MIB_STR(&snmp_log[5][0]),  	NULL},	//6-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 7},  			MIB_STR(&snmp_log[6][0]),  	NULL},	//7-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 8},  			MIB_STR(&snmp_log[7][0]),  	NULL},	//8-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 9},  			MIB_STR(&snmp_log[8][0]),  	NULL},	//9-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 10},  			MIB_STR(&snmp_log[9][0]),  	NULL},	//10-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 11},  			MIB_STR(&snmp_log[10][0]),  	NULL},	//11-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 12},  			MIB_STR(&snmp_log[11][0]),  	NULL},	//12-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 13},  			MIB_STR(&snmp_log[12][0]),  	NULL},	//13-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 14},  			MIB_STR(&snmp_log[13][0]),  	NULL},	//14-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 15},  			MIB_STR(&snmp_log[14][0]),  	NULL},	//15-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 16},  			MIB_STR(&snmp_log[15][0]),  	NULL},	//16-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 17},  			MIB_STR(&snmp_log[16][0]),  	NULL},	//17-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 18},  			MIB_STR(&snmp_log[17][0]),  	NULL},	//18-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 19},  			MIB_STR(&snmp_log[18][0]),  	NULL},	//19-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 20},  			MIB_STR(&snmp_log[19][0]),  	NULL},	//20-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 21},  			MIB_STR(&snmp_log[20][0]),  	NULL},	//21-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 22},  			MIB_STR(&snmp_log[21][0]),  	NULL},	//22-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 23},  			MIB_STR(&snmp_log[22][0]),  	NULL},	//23-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 24},  			MIB_STR(&snmp_log[23][0]),  	NULL},	//24-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 25},  			MIB_STR(&snmp_log[24][0]),  	NULL},	//25-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 26},  			MIB_STR(&snmp_log[25][0]),  	NULL},	//26-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 27},  			MIB_STR(&snmp_log[26][0]),  	NULL},	//27-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 28},  			MIB_STR(&snmp_log[27][0]),  	NULL},	//28-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 29},  			MIB_STR(&snmp_log[28][0]),  	NULL},	//29-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 30},  			MIB_STR(&snmp_log[29][0]),  	NULL},	//30-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 31},  			MIB_STR(&snmp_log[30][0]),  	NULL},	//31-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 32},  			MIB_STR(&snmp_log[31][0]),  	NULL},	//32-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 33},  			MIB_STR(&snmp_log[32][0]),  	NULL},	//33-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 34},  			MIB_STR(&snmp_log[33][0]),  	NULL},	//34-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 35},  			MIB_STR(&snmp_log[34][0]),  	NULL},	//35-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 36},  			MIB_STR(&snmp_log[35][0]),  	NULL},	//36-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 37},  			MIB_STR(&snmp_log[36][0]),  	NULL},	//37-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 38},  			MIB_STR(&snmp_log[37][0]),  	NULL},	//38-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 39},  			MIB_STR(&snmp_log[38][0]),  	NULL},	//39-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 40},  			MIB_STR(&snmp_log[39][0]),  	NULL},	//40-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 41},  			MIB_STR(&snmp_log[40][0]),  	NULL},	//41-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 42},  			MIB_STR(&snmp_log[41][0]),  	NULL},	//42-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 43},  			MIB_STR(&snmp_log[42][0]),  	NULL},	//43-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 44},  			MIB_STR(&snmp_log[43][0]),  	NULL},	//44-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 45},  			MIB_STR(&snmp_log[44][0]),  	NULL},	//45-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 46},  			MIB_STR(&snmp_log[45][0]),  	NULL},	//46-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 47},  			MIB_STR(&snmp_log[46][0]),  	NULL},	//47-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 48},  			MIB_STR(&snmp_log[47][0]),  	NULL},	//48-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 49},  			MIB_STR(&snmp_log[48][0]),  	NULL},	//49-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 50},  			MIB_STR(&snmp_log[49][0]),  	NULL},	//50-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 51},  			MIB_STR(&snmp_log[50][0]),  	NULL},	//51-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 52},  			MIB_STR(&snmp_log[51][0]),  	NULL},	//52-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 53},  			MIB_STR(&snmp_log[52][0]),  	NULL},	//53-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 54},  			MIB_STR(&snmp_log[53][0]),  	NULL},	//54-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 55},  			MIB_STR(&snmp_log[54][0]),  	NULL},	//55-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 56},  			MIB_STR(&snmp_log[55][0]),  	NULL},	//56-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 57},  			MIB_STR(&snmp_log[56][0]),  	NULL},	//57-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 58},  			MIB_STR(&snmp_log[57][0]),  	NULL},	//58-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 59},  			MIB_STR(&snmp_log[58][0]),  	NULL},	//59-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 60},  			MIB_STR(&snmp_log[59][0]),  	NULL},	//60-е событие из журнала

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 61},  			MIB_STR(&snmp_log[60][0]),  	NULL},	//61-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 62},  			MIB_STR(&snmp_log[61][0]),  	NULL},	//62-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 63},  			MIB_STR(&snmp_log[62][0]),  	NULL},	//63-е событие из журнала
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 64},  			MIB_STR(&snmp_log[63][0]),  	NULL},	//64-е событие из журнала


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_ENTRY_NUMBER, 1},  			MIB_INT(snmp_makb_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_ENTRY_NUMBER, 2},  			MIB_INT(snmp_makb_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_ENTRY_NUMBER, 3},  			MIB_INT(snmp_makb_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_ENTRY_NUMBER, 4}, 			MIB_INT(snmp_makb_number[3]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_CONNECT_STATUS, 1},  		MIB_INT(snmp_makb_connect_status[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_CONNECT_STATUS, 2},  		MIB_INT(snmp_makb_connect_status[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_CONNECT_STATUS, 3},  		MIB_INT(snmp_makb_connect_status[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_CONNECT_STATUS, 4}, 		MIB_INT(snmp_makb_connect_status[3]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE0, 1},  			MIB_INT(snmp_makb_voltage0[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE0, 2},  			MIB_INT(snmp_makb_voltage0[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE0, 3},  			MIB_INT(snmp_makb_voltage0[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE0, 4},  			MIB_INT(snmp_makb_voltage0[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE1, 1},  			MIB_INT(snmp_makb_voltage1[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE1, 2},  			MIB_INT(snmp_makb_voltage1[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE1, 3},  			MIB_INT(snmp_makb_voltage1[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE1, 4},  			MIB_INT(snmp_makb_voltage1[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE2, 1},  			MIB_INT(snmp_makb_voltage2[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE2, 2},  			MIB_INT(snmp_makb_voltage2[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE2, 3},  			MIB_INT(snmp_makb_voltage2[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE2, 4},  			MIB_INT(snmp_makb_voltage2[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE3, 1},  			MIB_INT(snmp_makb_voltage3[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE3, 2},  			MIB_INT(snmp_makb_voltage3[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE3, 3},  			MIB_INT(snmp_makb_voltage3[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE3, 4},  			MIB_INT(snmp_makb_voltage3[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE4, 1},  			MIB_INT(snmp_makb_voltage4[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE4, 2},  			MIB_INT(snmp_makb_voltage4[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE4, 3},  			MIB_INT(snmp_makb_voltage4[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_VOLTAGE4, 4},  			MIB_INT(snmp_makb_voltage4[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0, 1},  				MIB_INT(snmp_makb_temper0[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0, 2},  				MIB_INT(snmp_makb_temper0[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0, 3},  				MIB_INT(snmp_makb_temper0[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0, 4},  				MIB_INT(snmp_makb_temper0[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1, 1},  				MIB_INT(snmp_makb_temper1[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1, 2},  				MIB_INT(snmp_makb_temper1[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1, 3},  				MIB_INT(snmp_makb_temper1[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1, 4},  				MIB_INT(snmp_makb_temper1[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2, 1},  				MIB_INT(snmp_makb_temper2[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2, 2},  				MIB_INT(snmp_makb_temper2[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2, 3},  				MIB_INT(snmp_makb_temper2[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2, 4},  				MIB_INT(snmp_makb_temper2[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3, 1},  				MIB_INT(snmp_makb_temper3[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3, 2},  				MIB_INT(snmp_makb_temper3[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3, 3},  				MIB_INT(snmp_makb_temper3[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3, 4},  				MIB_INT(snmp_makb_temper3[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4, 1},  				MIB_INT(snmp_makb_temper4[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4, 2},  				MIB_INT(snmp_makb_temper4[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4, 3},  				MIB_INT(snmp_makb_temper4[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4, 4},  				MIB_INT(snmp_makb_temper4[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0_STAT, 1},  			MIB_INT(snmp_makb_temper0_stat[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0_STAT, 2},  			MIB_INT(snmp_makb_temper0_stat[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0_STAT, 3},  			MIB_INT(snmp_makb_temper0_stat[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER0_STAT, 4},  			MIB_INT(snmp_makb_temper0_stat[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1_STAT, 1},  			MIB_INT(snmp_makb_temper1_stat[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1_STAT, 2},  			MIB_INT(snmp_makb_temper1_stat[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1_STAT, 3},  			MIB_INT(snmp_makb_temper1_stat[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER1_STAT, 4},  			MIB_INT(snmp_makb_temper1_stat[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2_STAT, 1},  			MIB_INT(snmp_makb_temper2_stat[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2_STAT, 2},  			MIB_INT(snmp_makb_temper2_stat[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2_STAT, 3},  			MIB_INT(snmp_makb_temper2_stat[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER2_STAT, 4},  			MIB_INT(snmp_makb_temper2_stat[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3_STAT, 1},  			MIB_INT(snmp_makb_temper3_stat[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3_STAT, 2},  			MIB_INT(snmp_makb_temper3_stat[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3_STAT, 3},  			MIB_INT(snmp_makb_temper3_stat[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER3_STAT, 4},  			MIB_INT(snmp_makb_temper3_stat[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4_STAT, 1},  			MIB_INT(snmp_makb_temper4_stat[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4_STAT, 2},  			MIB_INT(snmp_makb_temper4_stat[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4_STAT, 3},  			MIB_INT(snmp_makb_temper4_stat[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAKB, DISPLAY_MAKB_TEMPER4_STAT, 4},  			MIB_INT(snmp_makb_temper4_stat[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSSOUNDALARMEN, 0},				MIB_INT(snmp_zv_en),  			snmp_zv_on_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSALARMAUTODISABLE, 0},				MIB_INT(snmp_alarm_auto_disable),	snmp_alarm_auto_disable_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_TEST_TIME, 0},				MIB_INT(snmp_bat_test_time),		snmp_bat_test_time_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MAX, 0},						MIB_INT(snmp_u_max),			snmp_u_max_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN, 0},						MIB_INT(snmp_u_min),			snmp_u_min_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_0_GRAD, 0},					MIB_INT(snmp_u_0_grad),			snmp_u_0_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_20_GRAD, 0},					MIB_INT(snmp_u_20_grad),			snmp_u_20_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_SIGN, 0},					MIB_INT(snmp_u_sign),			snmp_u_sign_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN_POWER, 0},				MIB_INT(snmp_u_min_power),		snmp_u_min_power_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_WITHOUT_BAT, 0},				MIB_INT(snmp_u_withouth_bat),		snmp_u_withouth_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IBK, 0},						MIB_INT(snmp_control_current),	snmp_control_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IZMAX, 0},						MIB_INT(snmp_max_charge_current),	snmp_max_charge_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMAX, 0},						MIB_INT(snmp_max_current),		snmp_max_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMIN, 0},						MIB_INT(snmp_min_current),		snmp_min_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_UVZ, 0},						MIB_INT(snmp_uvz),				snmp_uvz_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TZAS, 0},						MIB_INT(snmp_powerup_psu_timeout),	snmp_powerup_psu_timeout_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BAT, 0},					MIB_INT(snmp_tsign_bat),			snmp_tsign_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BAT, 0},					MIB_INT(snmp_tmax_bat),			snmp_tmax_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BPS, 0},					MIB_INT(snmp_tsign_bps),			snmp_tsign_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BPS, 0},					MIB_INT(snmp_tmax_bps),			snmp_tmax_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_PART_ALARM, 0},				MIB_INT(snmp_bat_part_alarm),		snmp_bat_part_alarm_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_POWER_CNT_ADRESS, 0},				MIB_INT(snmp_power_cnt_adress),	snmp_power_cnt_adress_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_IPS_SET, 0},					MIB_INT(snmp_u_0_grad),			snmp_u_ips_set_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_OUT_KONTR_MAX, 0},				MIB_INT(U_OUT_KONTR_MAX),		snmp_u_out_kontr_max_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_OUT_KONTR_MIN, 0},				MIB_INT(U_OUT_KONTR_MIN),		snmp_u_out_kontr_min_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_OUT_KONTR_DELAY, 0},			MIB_INT(U_OUT_KONTR_DELAY),		snmp_u_out_kontr_delay_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_VZ_U, 0},							MIB_INT(UVZ),					snmp_uvz_write},					//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_VZ_I_MAX, 0},						MIB_INT(IMAX_VZ),				snmp_imax_vz_write},				//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_VZ_TIME, 0},						MIB_INT(VZ_HR),					snmp_vz_hr_write},					//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_VZ_VENTBLOCKING, 0},				MIB_INT(VZ_CH_VENT_BLOK),		snmp_vz_ch_vent_block_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_I_MAX, 0},					MIB_INT(speedChrgCurr),			snmp_spz_i_max_write},				//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_U, 0},						MIB_INT(speedChrgVolt),			snmp_spz_u_write},					//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_TIME, 0},						MIB_INT(speedChrgTimeInHour),	snmp_spz_time_write},				//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_AVT_EN, 0},					MIB_INT(speedChrgAvtEn),		snmp_spz_avt_en_write},				//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_AVT_DELTA, 0},				MIB_INT(speedChrgDU),			snmp_spz_delta_write},				//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_BLOCK_EN_SRC, 0},				MIB_INT(speedChrgBlckSrc),		snmp_spz_block_en_src_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_BLOCK_SIGN, 0},				MIB_INT(speedChrgBlckLog),		snmp_spz_block_log_write},			//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_SPZ_VENTBLOCKING, 0},				MIB_INT(SP_CH_VENT_BLOK),		snmp_spz_vent_block_write},			//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MAX_POWER, 0},					MIB_INT(snmp_u_max_power),		snmp_u_max_power_write},			// максимальное напряжение сети //o_10

#ifdef UKU_ZVU
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 1},  			MIB_INT(snmp_sk_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 2},  			MIB_INT(snmp_sk_number[1]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 1},  				MIB_INT(snmp_sk_aktiv[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 2},  				MIB_INT(snmp_sk_aktiv[1]),  	NULL},	//Напряжение БПС2

	{ MIB_INTEGER ,  		  		13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 1},  		MIB_INT(snmp_sk_alarm_aktiv[0]),  	snmp_alarm_aktiv_write1},	//Ток БПС1
	{ MIB_INTEGER ,					13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 2},  		MIB_INT(snmp_sk_alarm_aktiv[1]),  	snmp_alarm_aktiv_write2},	//Ток БПС2

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 1},  					MIB_INT(snmp_sk_alarm[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 2},  					MIB_INT(snmp_sk_alarm[1]),  NULL},	//Ток БПС2

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 1},  		MIB_INT(snmp_dt_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 2},  		MIB_INT(snmp_dt_number[1]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 1},  			MIB_INT(snmp_dt_temper[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 2},  			MIB_INT(snmp_dt_temper[1]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 1},  			MIB_INT(snmp_dt_error[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 2},  			MIB_INT(snmp_dt_error[1]),  	NULL},	//Номер БПСа
#else
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 1},  			MIB_INT(snmp_sk_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 2},  			MIB_INT(snmp_sk_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 3},  			MIB_INT(snmp_sk_number[2]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 4},  			MIB_INT(snmp_sk_number[3]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 1},  				MIB_INT(snmp_sk_aktiv[0]),  	NULL},	//Напряжение БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 2},  				MIB_INT(snmp_sk_aktiv[1]),  	NULL},	//Напряжение БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 3},  				MIB_INT(snmp_sk_aktiv[2]),  	NULL},	//Напряжение БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 4},  				MIB_INT(snmp_sk_aktiv[3]),  	NULL},	//Напряжение БПС3

	{ MIB_INTEGER ,  		  		13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 1},  		MIB_INT(snmp_sk_alarm_aktiv[0]),  	snmp_alarm_aktiv_write1},	//Ток БПС1
	{ MIB_INTEGER ,					13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 2},  		MIB_INT(snmp_sk_alarm_aktiv[1]),  	snmp_alarm_aktiv_write2},	//Ток БПС2
	{ MIB_INTEGER ,					13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 3},  		MIB_INT(snmp_sk_alarm_aktiv[2]),  	snmp_alarm_aktiv_write3},	//Ток БПС3
	{ MIB_INTEGER ,					13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 4},  		MIB_INT(snmp_sk_alarm_aktiv[3]),  	snmp_alarm_aktiv_write4},	//Ток БПС3

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 1},  					MIB_INT(snmp_sk_alarm[0]),  NULL},	//Ток БПС1
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 2},  					MIB_INT(snmp_sk_alarm[1]),  NULL},	//Ток БПС2
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 3},  					MIB_INT(snmp_sk_alarm[2]),  NULL},	//Ток БПС3
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 4},  					MIB_INT(snmp_sk_alarm[3]),  NULL},	//Ток БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 1},  		MIB_INT(snmp_dt_number[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 2},  		MIB_INT(snmp_dt_number[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ENTRY_NUMBER, 3},  		MIB_INT(snmp_dt_number[2]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 1},  			MIB_INT(snmp_dt_temper[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 2},  			MIB_INT(snmp_dt_temper[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_TEMPER, 3},  			MIB_INT(snmp_dt_temper[2]),  	NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 1},  			MIB_INT(snmp_dt_error[0]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 2},  			MIB_INT(snmp_dt_error[1]),  	NULL},	//Номер БПСа
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_ERROR, 3},  			MIB_INT(snmp_dt_error[2]),  	NULL},	//Номер БПСа
#endif

	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 1},    MIB_INT(LPC_RTC->HOUR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},				  
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 1},    MIB_INT(LPC_RTC->MIN),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 1},     MIB_INT(LPC_RTC->SEC),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 2},    MIB_INT(LPC_RTC->MONTH),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 2},    MIB_INT(LPC_RTC->HOUR),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 1},     MIB_INT(sysMainsVoltage),    NULL},	    //-----------------------------------------------
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 2},    MIB_INT(sysServices),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 2},    MIB_INT(sysServices),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 1},    MIB_INT(sysMainsVoltage),     NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 1},     MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 1},    MIB_INT(sysMainsVoltage),    NULL},
	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 2},    MIB_INT(TestForTableValues),    NULL},
	{ MIB_OCTET_STR, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},  MIB_STR("Proverka sviazi. Проверка связи."),   NULL},

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 1},  			MIB_INT(snmp_lakb_number[0]),  	NULL},	//Номер ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 2},  			MIB_INT(snmp_lakb_number[1]),  	NULL},	//Номер ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 3},  			MIB_INT(snmp_lakb_number[2]),  	NULL},	//Номер ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 4},  			MIB_INT(snmp_lakb_number[3]),  	NULL},	//Номер ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 5},  			MIB_INT(snmp_lakb_number[4]),  	NULL},	//Номер ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 6},  			MIB_INT(snmp_lakb_number[5]),  	NULL},	//Номер ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_ENTRY_NUMBER, 7},  			MIB_INT(snmp_lakb_number[6]),  	NULL},	//Номер ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 1},  		MIB_INT(snmp_lakb_max_cell_voltage[0]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 2},  		MIB_INT(snmp_lakb_max_cell_voltage[1]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 3},  		MIB_INT(snmp_lakb_max_cell_voltage[2]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 4},  		MIB_INT(snmp_lakb_max_cell_voltage[3]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 5},  		MIB_INT(snmp_lakb_max_cell_voltage[4]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 6},  		MIB_INT(snmp_lakb_max_cell_voltage[5]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_VOLTAGE, 7},  		MIB_INT(snmp_lakb_max_cell_voltage[6]),  	NULL},	//Максимальное напряжение ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 1},  		MIB_INT(snmp_lakb_min_cell_voltage[0]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 2},  		MIB_INT(snmp_lakb_min_cell_voltage[1]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 3},  		MIB_INT(snmp_lakb_min_cell_voltage[2]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 4},  		MIB_INT(snmp_lakb_min_cell_voltage[3]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 5},  		MIB_INT(snmp_lakb_min_cell_voltage[4]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 6},  		MIB_INT(snmp_lakb_min_cell_voltage[5]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_VOLTAGE, 7},  		MIB_INT(snmp_lakb_min_cell_voltage[6]),  	NULL},	//Минимальное напряжение ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 1},  	MIB_INT(snmp_lakb_max_cell_temperature[0]),  	NULL},	//Максимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 2},  	MIB_INT(snmp_lakb_max_cell_temperature[1]),  	NULL},	//Максимальная температура ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 3},  	MIB_INT(snmp_lakb_max_cell_temperature[2]),  	NULL},	//Максимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 4},  	MIB_INT(snmp_lakb_max_cell_temperature[3]),  	NULL},	//Максимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 5},  	MIB_INT(snmp_lakb_max_cell_temperature[4]),  	NULL},	//Максимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 6},  	MIB_INT(snmp_lakb_max_cell_temperature[5]),  	NULL},	//Максимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MAX_CELL_TEMPERATURE, 7},  	MIB_INT(snmp_lakb_max_cell_temperature[6]),  	NULL},	//Максимальная температура ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 1},  	MIB_INT(snmp_lakb_min_cell_temperature[0]),  	NULL},	//Минимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 2},  	MIB_INT(snmp_lakb_min_cell_temperature[1]),  	NULL},	//Минимальная температура ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 3},  	MIB_INT(snmp_lakb_min_cell_temperature[2]),  	NULL},	//Минимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 4},  	MIB_INT(snmp_lakb_min_cell_temperature[3]),  	NULL},	//Минимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 5},  	MIB_INT(snmp_lakb_min_cell_temperature[4]),  	NULL},	//Минимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 6},  	MIB_INT(snmp_lakb_min_cell_temperature[5]),  	NULL},	//Минимальная температура ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_MIN_CELL_TEMPERATURE, 7},  	MIB_INT(snmp_lakb_min_cell_temperature[6]),  	NULL},	//Минимальная температура ячейки ЛАКБ
#endif
	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 1},  	MIB_INT(snmp_lakb_cell_temperature_1[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 2},  	MIB_INT(snmp_lakb_cell_temperature_1[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 3},  	MIB_INT(snmp_lakb_cell_temperature_1[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 4},  	MIB_INT(snmp_lakb_cell_temperature_1[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 5},  	MIB_INT(snmp_lakb_cell_temperature_1[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 6},  	MIB_INT(snmp_lakb_cell_temperature_1[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_1, 7},  	MIB_INT(snmp_lakb_cell_temperature_1[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 1},  	MIB_INT(snmp_lakb_cell_temperature_2[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 2},  	MIB_INT(snmp_lakb_cell_temperature_2[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 3},  	MIB_INT(snmp_lakb_cell_temperature_2[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 4},  	MIB_INT(snmp_lakb_cell_temperature_2[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 5},  	MIB_INT(snmp_lakb_cell_temperature_2[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 6},  	MIB_INT(snmp_lakb_cell_temperature_2[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_2, 7},  	MIB_INT(snmp_lakb_cell_temperature_2[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 1},  	MIB_INT(snmp_lakb_cell_temperature_3[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 2},  	MIB_INT(snmp_lakb_cell_temperature_3[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 3},  	MIB_INT(snmp_lakb_cell_temperature_3[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 4},  	MIB_INT(snmp_lakb_cell_temperature_3[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 5},  	MIB_INT(snmp_lakb_cell_temperature_3[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 6},  	MIB_INT(snmp_lakb_cell_temperature_3[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_3, 7},  	MIB_INT(snmp_lakb_cell_temperature_3[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 1},  	MIB_INT(snmp_lakb_cell_temperature_4[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 2},  	MIB_INT(snmp_lakb_cell_temperature_4[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 3},  	MIB_INT(snmp_lakb_cell_temperature_4[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 4},  	MIB_INT(snmp_lakb_cell_temperature_4[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 5},  	MIB_INT(snmp_lakb_cell_temperature_4[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 6},  	MIB_INT(snmp_lakb_cell_temperature_4[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_4, 7},  	MIB_INT(snmp_lakb_cell_temperature_4[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 1},  	MIB_INT(snmp_lakb_cell_temperature_ambient[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 2},  	MIB_INT(snmp_lakb_cell_temperature_ambient[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 3},  	MIB_INT(snmp_lakb_cell_temperature_ambient[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 4},  	MIB_INT(snmp_lakb_cell_temperature_ambient[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 5},  	MIB_INT(snmp_lakb_cell_temperature_ambient[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 6},  	MIB_INT(snmp_lakb_cell_temperature_ambient[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_AMBIENT, 7},  	MIB_INT(snmp_lakb_cell_temperature_ambient[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 1},  	MIB_INT(snmp_lakb_cell_temperature_power[0]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 2},  	MIB_INT(snmp_lakb_cell_temperature_power[1]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 3},  	MIB_INT(snmp_lakb_cell_temperature_power[2]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 4},  	MIB_INT(snmp_lakb_cell_temperature_power[3]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 5},  	MIB_INT(snmp_lakb_cell_temperature_power[4]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 6},  	MIB_INT(snmp_lakb_cell_temperature_power[5]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CELL_TEMPERATURE_POWER, 7},  	MIB_INT(snmp_lakb_cell_temperature_power[6]),  	  	NULL},	//Температура 1-й ячейки ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 1},  				MIB_INT(snmp_lakb_voltage[0]),  	NULL},	//Напряжение ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 2},  				MIB_INT(snmp_lakb_voltage[1]),  	NULL},	//Напряжение ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 3},  				MIB_INT(snmp_lakb_voltage[2]),  	NULL},	//Напряжение ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 4},  				MIB_INT(snmp_lakb_voltage[3]),  	NULL},	//Напряжение ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 5},  				MIB_INT(snmp_lakb_voltage[4]),  	NULL},	//Напряжение ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 6},  				MIB_INT(snmp_lakb_voltage[5]),  	NULL},	//Напряжение ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE, 7},  				MIB_INT(snmp_lakb_voltage[6]),  	NULL},	//Напряжение ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 1},  				MIB_INT(snmp_lakb_ch_curr[0]),  	NULL},	//Ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 2},  				MIB_INT(snmp_lakb_ch_curr[1]),  	NULL},	//Ток заряда ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 3},  				MIB_INT(snmp_lakb_ch_curr[2]),  	NULL},	//Ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 4},  				MIB_INT(snmp_lakb_ch_curr[3]),  	NULL},	//Ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 5},  				MIB_INT(snmp_lakb_ch_curr[4]),  	NULL},	//Ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 6},  				MIB_INT(snmp_lakb_ch_curr[5]),  	NULL},	//Ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CH_CURR, 7},  				MIB_INT(snmp_lakb_ch_curr[6]),  	NULL},	//Ток заряда ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 1},  			MIB_INT(snmp_lakb_dsch_curr[0]),  	NULL},	//Ток разряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 2},  			MIB_INT(snmp_lakb_dsch_curr[1]),  	NULL},	//Ток разряда ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 3},  			MIB_INT(snmp_lakb_dsch_curr[2]),  	NULL},	//Ток разряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 4},  			MIB_INT(snmp_lakb_dsch_curr[3]),  	NULL},	//Ток разряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 5},  			MIB_INT(snmp_lakb_dsch_curr[4]),  	NULL},	//Ток разряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 6},  			MIB_INT(snmp_lakb_dsch_curr[5]),  	NULL},	//Ток разряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DSCH_CURR, 7},  			MIB_INT(snmp_lakb_dsch_curr[6]),  	NULL},	//Ток разряда ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 1},  				MIB_INT(snmp_lakb_rat_cap[0]),  	NULL},	//Номинальная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 2},  				MIB_INT(snmp_lakb_rat_cap[1]),  	NULL},	//Номинальная емкость ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 3},  				MIB_INT(snmp_lakb_rat_cap[2]),  	NULL},	//Номинальная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 4},  				MIB_INT(snmp_lakb_rat_cap[3]),  	NULL},	//Номинальная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 5},  				MIB_INT(snmp_lakb_rat_cap[4]),  	NULL},	//Номинальная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 6},  				MIB_INT(snmp_lakb_rat_cap[5]),  	NULL},	//Номинальная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RAT_CAP, 7},  				MIB_INT(snmp_lakb_rat_cap[6]),  	NULL},	//Номинальная емкость ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 1},  				MIB_INT(snmp_lakb_soh[0]),  		NULL},	//Остаточная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 2},  				MIB_INT(snmp_lakb_soh[1]),  		NULL},	//Остаточная емкость ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 3},  				MIB_INT(snmp_lakb_soh[2]),  		NULL},	//Остаточная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 4},  				MIB_INT(snmp_lakb_soh[3]),  		NULL},	//Остаточная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 5},  				MIB_INT(snmp_lakb_soh[4]),  		NULL},	//Остаточная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 6},  				MIB_INT(snmp_lakb_soh[5]),  		NULL},	//Остаточная емкость ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOH, 7},  				MIB_INT(snmp_lakb_soh[6]),  		NULL},	//Остаточная емкость ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 1},  				MIB_INT(snmp_lakb_soc[0]),  		NULL},	//Заряд ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 2},  				MIB_INT(snmp_lakb_soc[1]),  		NULL},	//Заряд ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 3},  				MIB_INT(snmp_lakb_soc[2]),  		NULL},	//Заряд ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 4},  				MIB_INT(snmp_lakb_soc[3]),  		NULL},	//Заряд ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 5},  				MIB_INT(snmp_lakb_soc[4]),  		NULL},	//Заряд ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 6},  				MIB_INT(snmp_lakb_soc[5]),  		NULL},	//Заряд ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SOC, 7},  				MIB_INT(snmp_lakb_soc[6]),  		NULL},	//Заряд ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 1},  				MIB_INT(snmp_lakb_cclv[0]),  		NULL},	//Максимальный ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 2},  				MIB_INT(snmp_lakb_cclv[1]),  		NULL},	//Максимальный ток заряда ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 3},  				MIB_INT(snmp_lakb_cclv[2]),  		NULL},	//Максимальный ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 4},  				MIB_INT(snmp_lakb_cclv[3]),  		NULL},	//Максимальный ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 5},  				MIB_INT(snmp_lakb_cclv[4]),  		NULL},	//Максимальный ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 6},  				MIB_INT(snmp_lakb_cclv[5]),  		NULL},	//Максимальный ток заряда ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CCLV, 7},  				MIB_INT(snmp_lakb_cclv[6]),  		NULL},	//Максимальный ток заряда ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 1},  				MIB_INT(snmp_lakb_rbt[0]),  		NULL},	//Оцениваемое время работы ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 2},  				MIB_INT(snmp_lakb_rbt[1]),  		NULL},	//Оцениваемое время работы ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 3},  				MIB_INT(snmp_lakb_rbt[2]),  		NULL},	//Оцениваемое время работы ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 4},  				MIB_INT(snmp_lakb_rbt[3]),  		NULL},	//Оцениваемое время работы ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 5},  				MIB_INT(snmp_lakb_rbt[4]),  		NULL},	//Оцениваемое время работы ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 6},  				MIB_INT(snmp_lakb_rbt[5]),  		NULL},	//Оцениваемое время работы ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_RBT, 7},  				MIB_INT(snmp_lakb_rbt[6]),  		NULL},	//Оцениваемое время работы ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 1},  				MIB_INT(snmp_lakb_flags1[0]),  		NULL},	//Первый флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 2},  				MIB_INT(snmp_lakb_flags1[1]),  		NULL},	//Первый флаг состояния ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 3},  				MIB_INT(snmp_lakb_flags1[0]),  		NULL},	//Первый флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 4},  				MIB_INT(snmp_lakb_flags1[1]),  		NULL},	//Первый флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 5},  				MIB_INT(snmp_lakb_flags1[0]),  		NULL},	//Первый флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 6},  				MIB_INT(snmp_lakb_flags1[1]),  		NULL},	//Первый флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS1, 7},  				MIB_INT(snmp_lakb_flags1[0]),  		NULL},	//Первый флаг состояния ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 1},  				MIB_INT(snmp_lakb_flags2[0]),  		NULL},	//Второй флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 2},  				MIB_INT(snmp_lakb_flags2[1]),  		NULL},	//Второй флаг состояния ЛАКБ
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 3},  				MIB_INT(snmp_lakb_flags2[0]),  		NULL},	//Второй флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 4},  				MIB_INT(snmp_lakb_flags2[1]),  		NULL},	//Второй флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 5},  				MIB_INT(snmp_lakb_flags2[0]),  		NULL},	//Второй флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 6},  				MIB_INT(snmp_lakb_flags2[1]),  		NULL},	//Второй флаг состояния ЛАКБ
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FLAGS2, 7},  				MIB_INT(snmp_lakb_flags2[0]),  		NULL},	//Второй флаг состояния ЛАКБ
#endif

	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CHARGE_AND_DISCHARGE_CURRENT_ALARM_STATUS, 1},  				MIB_INT(lakb[0]._charge_and_discharge_current_alarm_status),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CHARGE_AND_DISCHARGE_CURRENT_ALARM_STATUS, 2},  				MIB_INT(lakb[1]._charge_and_discharge_current_alarm_status),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CHARGE_AND_DISCHARGE_CURRENT_ALARM_STATUS, 3},  				MIB_INT(lakb[2]._charge_and_discharge_current_alarm_status),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BATTERY_TOTAL_VOLTAGE_ALARM_STATUS, 1},  						MIB_INT(lakb[0]._battery_total_voltage_alarm_status),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BATTERY_TOTAL_VOLTAGE_ALARM_STATUS, 2},  						MIB_INT(lakb[1]._battery_total_voltage_alarm_status),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BATTERY_TOTAL_VOLTAGE_ALARM_STATUS, 3},  						MIB_INT(lakb[2]._battery_total_voltage_alarm_status),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CUSTOM_ALARM_QUANTITY, 1},  									MIB_INT(lakb[0]._custom_alarm_quantity),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CUSTOM_ALARM_QUANTITY, 2},  									MIB_INT(lakb[1]._custom_alarm_quantity),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CUSTOM_ALARM_QUANTITY, 3},  									MIB_INT(lakb[2]._custom_alarm_quantity),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_EVENT_CODE, 1},  										MIB_INT(lakb[0]._balanced_event_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_EVENT_CODE, 2},  										MIB_INT(lakb[1]._balanced_event_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_EVENT_CODE, 3},  										MIB_INT(lakb[2]._balanced_event_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE_EVENT_CODE, 1},  										MIB_INT(lakb[0]._voltage_event_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE_EVENT_CODE, 2},  										MIB_INT(lakb[1]._voltage_event_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_VOLTAGE_EVENT_CODE, 3},  										MIB_INT(lakb[2]._voltage_event_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_TEMPERATURE_EVENT_CODE, 1},  									MIB_INT(lakb[0]._temperature_event_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_TEMPERATURE_EVENT_CODE, 2},  									MIB_INT(lakb[1]._temperature_event_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_TEMPERATURE_EVENT_CODE, 3},  									MIB_INT(lakb[2]._temperature_event_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CURRENT_EVENT_CODE, 1},  										MIB_INT(lakb[0]._current_event_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CURRENT_EVENT_CODE, 2},  										MIB_INT(lakb[1]._current_event_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_CURRENT_EVENT_CODE, 3},  										MIB_INT(lakb[2]._current_event_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FET_STATUS_CODE, 1},  											MIB_INT(lakb[0]._fet_status_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FET_STATUS_CODE, 2},  											MIB_INT(lakb[1]._fet_status_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_FET_STATUS_CODE, 3},  											MIB_INT(lakb[2]._fet_status_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_STATUS_CODE, 1},  										MIB_INT(lakb[0]._balanced_status_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_STATUS_CODE, 2},  										MIB_INT(lakb[1]._balanced_status_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_BALANCED_STATUS_CODE, 3},  										MIB_INT(lakb[2]._balanced_status_code),  		NULL},	
#endif
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SYSTEM_STATUS_CODE, 1},  										MIB_INT(lakb[0]._system_status_code),  		NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SYSTEM_STATUS_CODE, 2},  										MIB_INT(lakb[1]._system_status_code),  		NULL},	
#ifndef UKU_TELECORE2017
	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_SYSTEM_STATUS_CODE, 3},  										MIB_INT(lakb[2]._system_status_code),  		NULL},	
#endif

#ifndef UKU_TELECORE2017
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 1},  				MIB_INT(snmp_lakb_damp1[0][0]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 2},  				MIB_INT(snmp_lakb_damp1[1][0]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 3},  				MIB_INT(snmp_lakb_damp1[2][0]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 4},  				MIB_INT(snmp_lakb_damp1[3][0]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 5},  				MIB_INT(snmp_lakb_damp1[4][0]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 6},  				MIB_INT(snmp_lakb_damp1[5][0]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP1, 7},  				MIB_INT(snmp_lakb_damp1[6][0]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 1},  				MIB_INT(snmp_lakb_damp1[0][30]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 2},  				MIB_INT(snmp_lakb_damp1[1][30]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 3},  				MIB_INT(snmp_lakb_damp1[2][30]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 4},  				MIB_INT(snmp_lakb_damp1[3][30]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 5},  				MIB_INT(snmp_lakb_damp1[4][30]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 6},  				MIB_INT(snmp_lakb_damp1[5][30]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP2, 7},  				MIB_INT(snmp_lakb_damp1[6][30]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 1},  				MIB_INT(snmp_lakb_damp1[0][60]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 2},  				MIB_INT(snmp_lakb_damp1[1][60]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 3},  				MIB_INT(snmp_lakb_damp1[2][60]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 4},  				MIB_INT(snmp_lakb_damp1[3][60]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 5},  				MIB_INT(snmp_lakb_damp1[4][60]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 6},  				MIB_INT(snmp_lakb_damp1[5][60]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP3, 7},  				MIB_INT(snmp_lakb_damp1[6][60]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 1},  				MIB_INT(snmp_lakb_damp1[0][90]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 2},  				MIB_INT(snmp_lakb_damp1[1][90]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 3},  				MIB_INT(snmp_lakb_damp1[2][90]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 4},  				MIB_INT(snmp_lakb_damp1[3][90]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 5},  				MIB_INT(snmp_lakb_damp1[4][90]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 6},  				MIB_INT(snmp_lakb_damp1[5][90]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP4, 7},  				MIB_INT(snmp_lakb_damp1[6][90]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 1},  				MIB_INT(snmp_lakb_damp1[0][120]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 2},  				MIB_INT(snmp_lakb_damp1[1][120]),  		NULL},	//Отладка
	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 3},  				MIB_INT(snmp_lakb_damp1[2][120]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 4},  				MIB_INT(snmp_lakb_damp1[3][120]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 5},  				MIB_INT(snmp_lakb_damp1[4][120]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 6},  				MIB_INT(snmp_lakb_damp1[5][120]),  		NULL},	//Отладка
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LAKB, DISPLAY_LAKB_DAMP5, 7}, 				MIB_INT(snmp_lakb_damp1[6][120]),  		NULL}
#endif
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_WARM_SIGNAL, 0},		MIB_INT(snmp_warm_sign),  				snmp_warm_sign_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_SIGNAL, 0},		MIB_INT(snmp_cool_sign),  				snmp_cool_sign_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_WARM_ON_TEMPER, 0},		MIB_INT(snmp_warm_on_temper),  			snmp_warm_on_temper_write},	//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_WARM_OFF_TEMPER, 0},	MIB_INT(snmp_warm_off_temper),  		snmp_warm_off_temper_write},//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_WARM_Q, 0},				MIB_INT(snmp_warm_q),  					snmp_warm_q_write},			//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_100_TEMPER, 0},	MIB_INT(snmp_cool_100_temper),  		snmp_cool_100_temper_write},//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_80_TEMPER, 0},		MIB_INT(snmp_cool_80_temper),  			snmp_cool_80_temper_write},	//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_60_TEMPER, 0},		MIB_INT(snmp_cool_60_temper),  			snmp_cool_60_temper_write},	//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_40_TEMPER, 0},		MIB_INT(snmp_cool_40_temper),  			snmp_cool_40_temper_write},	//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_20_TEMPER, 0},		MIB_INT(snmp_cool_20_temper),  			snmp_cool_20_temper_write},	//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_100_DTEMPER, 0},	MIB_INT(snmp_cool_100_dtemper),  		snmp_cool_100_dtemper_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_80_DTEMPER, 0},	MIB_INT(snmp_cool_80_dtemper),  		snmp_cool_80_dtemper_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_60_DTEMPER, 0},	MIB_INT(snmp_cool_60_dtemper),  		snmp_cool_60_dtemper_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_40_DTEMPER, 0},	MIB_INT(snmp_cool_40_dtemper),  		snmp_cool_40_dtemper_write},		//^^номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_COOL_20_DTEMPER, 0},	MIB_INT(snmp_cool_20_dtemper),  		snmp_cool_20_dtemper_write},		//^^номер первого бпса 
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_WARM_STAT, 0},			MIB_INT(snmp_warm_stat),				NULL},		//^^номер первого бпса 
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_INT_VENT_PWM_STAT, 0},	MIB_INT(TELECORE2017_INT_VENT_PWM),  	NULL},		//^^номер первого бпса 
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_EXT_VENT_PWM_STAT, 0},	MIB_INT(TELECORE2017_EXT_VENT_PWM),		NULL},		//^^номер первого бпса 



	//o_2_s	   данные с модуля ЭНМВ-1
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 1}, MIB_INT(snmp_enmv_number[0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 2}, MIB_INT(snmp_enmv_number[1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 3}, MIB_INT(snmp_enmv_number[2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 4}, MIB_INT(snmp_enmv_number[3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 5}, MIB_INT(snmp_enmv_number[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 6}, MIB_INT(snmp_enmv_number[5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 7}, MIB_INT(snmp_enmv_number[6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 8}, MIB_INT(snmp_enmv_number[7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 9}, MIB_INT(snmp_enmv_number[8]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 10}, MIB_INT(snmp_enmv_number[9]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 11}, MIB_INT(snmp_enmv_number[10]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 12}, MIB_INT(snmp_enmv_number[11]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 13}, MIB_INT(snmp_enmv_number[12]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 14}, MIB_INT(snmp_enmv_number[13]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 15}, MIB_INT(snmp_enmv_number[14]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 16}, MIB_INT(snmp_enmv_number[15]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 17}, MIB_INT(snmp_enmv_number[16]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 18}, MIB_INT(snmp_enmv_number[17]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 19}, MIB_INT(snmp_enmv_number[18]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 20}, MIB_INT(snmp_enmv_number[19]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 21}, MIB_INT(snmp_enmv_number[20]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 22}, MIB_INT(snmp_enmv_number[21]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 23}, MIB_INT(snmp_enmv_number[22]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 24}, MIB_INT(snmp_enmv_number[23]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 25}, MIB_INT(snmp_enmv_number[24]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 26}, MIB_INT(snmp_enmv_number[25]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 27}, MIB_INT(snmp_enmv_number[26]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 28}, MIB_INT(snmp_enmv_number[27]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 29}, MIB_INT(snmp_enmv_number[28]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 30}, MIB_INT(snmp_enmv_number[29]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 31}, MIB_INT(snmp_enmv_number[30]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 32}, MIB_INT(snmp_enmv_number[31]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 33}, MIB_INT(snmp_enmv_number[32]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 34}, MIB_INT(snmp_enmv_number[33]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 35}, MIB_INT(snmp_enmv_number[34]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 36}, MIB_INT(snmp_enmv_number[35]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 37}, MIB_INT(snmp_enmv_number[36]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 38}, MIB_INT(snmp_enmv_number[37]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 39}, MIB_INT(snmp_enmv_number[38]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 40}, MIB_INT(snmp_enmv_number[39]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 41}, MIB_INT(snmp_enmv_number[40]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 42}, MIB_INT(snmp_enmv_number[41]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 43}, MIB_INT(snmp_enmv_number[42]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 44}, MIB_INT(snmp_enmv_number[43]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 45}, MIB_INT(snmp_enmv_number[44]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 46}, MIB_INT(snmp_enmv_number[45]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 47}, MIB_INT(snmp_enmv_number[46]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 48}, MIB_INT(snmp_enmv_number[47]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 49}, MIB_INT(snmp_enmv_number[48]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 50}, MIB_INT(snmp_enmv_number[49]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 51}, MIB_INT(snmp_enmv_number[50]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 52}, MIB_INT(snmp_enmv_number[51]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 53}, MIB_INT(snmp_enmv_number[52]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 54}, MIB_INT(snmp_enmv_number[53]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 55}, MIB_INT(snmp_enmv_number[54]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 56}, MIB_INT(snmp_enmv_number[55]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 57}, MIB_INT(snmp_enmv_number[56]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 58}, MIB_INT(snmp_enmv_number[57]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 59}, MIB_INT(snmp_enmv_number[58]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 60}, MIB_INT(snmp_enmv_number[59]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 61}, MIB_INT(snmp_enmv_number[60]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 62}, MIB_INT(snmp_enmv_number[61]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 63}, MIB_INT(snmp_enmv_number[62]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_NUMBER, 64}, MIB_INT(snmp_enmv_number[63]),	NULL},
	
//o_12_s
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 1}, MIB_INT(snmp_enmv_data[0][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 2}, MIB_INT(snmp_enmv_data[1][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 3}, MIB_INT(snmp_enmv_data[2][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 4}, MIB_INT(snmp_enmv_data[3][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 5}, MIB_INT(snmp_enmv_data[4][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 6}, MIB_INT(snmp_enmv_data[5][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 7}, MIB_INT(snmp_enmv_data[6][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 8}, MIB_INT(snmp_enmv_data[7][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 9}, MIB_INT(snmp_enmv_data[8][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 10}, MIB_INT(snmp_enmv_data[9][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 11}, MIB_INT(snmp_enmv_data[10][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 12}, MIB_INT(snmp_enmv_data[11][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 13}, MIB_INT(snmp_enmv_data[12][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 14}, MIB_INT(snmp_enmv_data[13][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 15}, MIB_INT(snmp_enmv_data[14][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 16}, MIB_INT(snmp_enmv_data[15][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 17}, MIB_INT(snmp_enmv_data[16][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 18}, MIB_INT(snmp_enmv_data[17][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 19}, MIB_INT(snmp_enmv_data[18][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 20}, MIB_INT(snmp_enmv_data[19][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 21}, MIB_INT(snmp_enmv_data[20][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 22}, MIB_INT(snmp_enmv_data[21][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 23}, MIB_INT(snmp_enmv_data[22][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 24}, MIB_INT(snmp_enmv_data[23][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 25}, MIB_INT(snmp_enmv_data[24][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 26}, MIB_INT(snmp_enmv_data[25][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 27}, MIB_INT(snmp_enmv_data[26][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 28}, MIB_INT(snmp_enmv_data[27][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 29}, MIB_INT(snmp_enmv_data[28][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 30}, MIB_INT(snmp_enmv_data[29][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 31}, MIB_INT(snmp_enmv_data[30][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 32}, MIB_INT(snmp_enmv_data[31][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 33}, MIB_INT(snmp_enmv_data[32][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 34}, MIB_INT(snmp_enmv_data[33][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 35}, MIB_INT(snmp_enmv_data[34][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 36}, MIB_INT(snmp_enmv_data[35][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 37}, MIB_INT(snmp_enmv_data[36][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 38}, MIB_INT(snmp_enmv_data[37][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 39}, MIB_INT(snmp_enmv_data[38][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 40}, MIB_INT(snmp_enmv_data[39][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 41}, MIB_INT(snmp_enmv_data[40][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 42}, MIB_INT(snmp_enmv_data[41][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 43}, MIB_INT(snmp_enmv_data[42][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 44}, MIB_INT(snmp_enmv_data[43][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 45}, MIB_INT(snmp_enmv_data[44][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 46}, MIB_INT(snmp_enmv_data[45][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 47}, MIB_INT(snmp_enmv_data[46][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 48}, MIB_INT(snmp_enmv_data[47][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 49}, MIB_INT(snmp_enmv_data[48][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 50}, MIB_INT(snmp_enmv_data[49][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 51}, MIB_INT(snmp_enmv_data[50][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 52}, MIB_INT(snmp_enmv_data[51][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 53}, MIB_INT(snmp_enmv_data[52][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 54}, MIB_INT(snmp_enmv_data[53][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 55}, MIB_INT(snmp_enmv_data[54][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 56}, MIB_INT(snmp_enmv_data[55][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 57}, MIB_INT(snmp_enmv_data[56][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 58}, MIB_INT(snmp_enmv_data[57][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 59}, MIB_INT(snmp_enmv_data[58][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 60}, MIB_INT(snmp_enmv_data[59][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 61}, MIB_INT(snmp_enmv_data[60][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 62}, MIB_INT(snmp_enmv_data[61][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 63}, MIB_INT(snmp_enmv_data[62][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_1, 64}, MIB_INT(snmp_enmv_data[63][0]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 1}, MIB_INT(snmp_enmv_data[0][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 2}, MIB_INT(snmp_enmv_data[1][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 3}, MIB_INT(snmp_enmv_data[2][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 4}, MIB_INT(snmp_enmv_data[3][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 5}, MIB_INT(snmp_enmv_data[4][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 6}, MIB_INT(snmp_enmv_data[5][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 7}, MIB_INT(snmp_enmv_data[6][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 8}, MIB_INT(snmp_enmv_data[7][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 9}, MIB_INT(snmp_enmv_data[8][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 10}, MIB_INT(snmp_enmv_data[9][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 11}, MIB_INT(snmp_enmv_data[10][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 12}, MIB_INT(snmp_enmv_data[11][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 13}, MIB_INT(snmp_enmv_data[12][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 14}, MIB_INT(snmp_enmv_data[13][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 15}, MIB_INT(snmp_enmv_data[14][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 16}, MIB_INT(snmp_enmv_data[15][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 17}, MIB_INT(snmp_enmv_data[16][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 18}, MIB_INT(snmp_enmv_data[17][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 19}, MIB_INT(snmp_enmv_data[18][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 20}, MIB_INT(snmp_enmv_data[19][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 21}, MIB_INT(snmp_enmv_data[20][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 22}, MIB_INT(snmp_enmv_data[21][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 23}, MIB_INT(snmp_enmv_data[22][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 24}, MIB_INT(snmp_enmv_data[23][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 25}, MIB_INT(snmp_enmv_data[24][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 26}, MIB_INT(snmp_enmv_data[25][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 27}, MIB_INT(snmp_enmv_data[26][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 28}, MIB_INT(snmp_enmv_data[27][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 29}, MIB_INT(snmp_enmv_data[28][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 30}, MIB_INT(snmp_enmv_data[29][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 31}, MIB_INT(snmp_enmv_data[30][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 32}, MIB_INT(snmp_enmv_data[31][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 33}, MIB_INT(snmp_enmv_data[32][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 34}, MIB_INT(snmp_enmv_data[33][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 35}, MIB_INT(snmp_enmv_data[34][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 36}, MIB_INT(snmp_enmv_data[35][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 37}, MIB_INT(snmp_enmv_data[36][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 38}, MIB_INT(snmp_enmv_data[37][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 39}, MIB_INT(snmp_enmv_data[38][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 40}, MIB_INT(snmp_enmv_data[39][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 41}, MIB_INT(snmp_enmv_data[40][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 42}, MIB_INT(snmp_enmv_data[41][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 43}, MIB_INT(snmp_enmv_data[42][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 44}, MIB_INT(snmp_enmv_data[43][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 45}, MIB_INT(snmp_enmv_data[44][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 46}, MIB_INT(snmp_enmv_data[45][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 47}, MIB_INT(snmp_enmv_data[46][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 48}, MIB_INT(snmp_enmv_data[47][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 49}, MIB_INT(snmp_enmv_data[48][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 50}, MIB_INT(snmp_enmv_data[49][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 51}, MIB_INT(snmp_enmv_data[50][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 52}, MIB_INT(snmp_enmv_data[51][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 53}, MIB_INT(snmp_enmv_data[52][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 54}, MIB_INT(snmp_enmv_data[53][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 55}, MIB_INT(snmp_enmv_data[54][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 56}, MIB_INT(snmp_enmv_data[55][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 57}, MIB_INT(snmp_enmv_data[56][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 58}, MIB_INT(snmp_enmv_data[57][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 59}, MIB_INT(snmp_enmv_data[58][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 60}, MIB_INT(snmp_enmv_data[59][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 61}, MIB_INT(snmp_enmv_data[60][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 62}, MIB_INT(snmp_enmv_data[61][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 63}, MIB_INT(snmp_enmv_data[62][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_2, 64}, MIB_INT(snmp_enmv_data[63][1]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 1}, MIB_INT(snmp_enmv_data[0][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 2}, MIB_INT(snmp_enmv_data[1][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 3}, MIB_INT(snmp_enmv_data[2][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 4}, MIB_INT(snmp_enmv_data[3][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 5}, MIB_INT(snmp_enmv_data[4][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 6}, MIB_INT(snmp_enmv_data[5][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 7}, MIB_INT(snmp_enmv_data[6][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 8}, MIB_INT(snmp_enmv_data[7][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 9}, MIB_INT(snmp_enmv_data[8][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 10}, MIB_INT(snmp_enmv_data[9][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 11}, MIB_INT(snmp_enmv_data[10][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 12}, MIB_INT(snmp_enmv_data[11][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 13}, MIB_INT(snmp_enmv_data[12][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 14}, MIB_INT(snmp_enmv_data[13][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 15}, MIB_INT(snmp_enmv_data[14][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 16}, MIB_INT(snmp_enmv_data[15][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 17}, MIB_INT(snmp_enmv_data[16][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 18}, MIB_INT(snmp_enmv_data[17][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 19}, MIB_INT(snmp_enmv_data[18][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 20}, MIB_INT(snmp_enmv_data[19][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 21}, MIB_INT(snmp_enmv_data[20][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 22}, MIB_INT(snmp_enmv_data[21][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 23}, MIB_INT(snmp_enmv_data[22][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 24}, MIB_INT(snmp_enmv_data[23][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 25}, MIB_INT(snmp_enmv_data[24][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 26}, MIB_INT(snmp_enmv_data[25][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 27}, MIB_INT(snmp_enmv_data[26][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 28}, MIB_INT(snmp_enmv_data[27][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 29}, MIB_INT(snmp_enmv_data[28][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 30}, MIB_INT(snmp_enmv_data[29][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 31}, MIB_INT(snmp_enmv_data[30][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 32}, MIB_INT(snmp_enmv_data[31][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 33}, MIB_INT(snmp_enmv_data[32][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 34}, MIB_INT(snmp_enmv_data[33][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 35}, MIB_INT(snmp_enmv_data[34][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 36}, MIB_INT(snmp_enmv_data[35][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 37}, MIB_INT(snmp_enmv_data[36][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 38}, MIB_INT(snmp_enmv_data[37][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 39}, MIB_INT(snmp_enmv_data[38][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 40}, MIB_INT(snmp_enmv_data[39][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 41}, MIB_INT(snmp_enmv_data[40][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 42}, MIB_INT(snmp_enmv_data[41][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 43}, MIB_INT(snmp_enmv_data[42][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 44}, MIB_INT(snmp_enmv_data[43][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 45}, MIB_INT(snmp_enmv_data[44][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 46}, MIB_INT(snmp_enmv_data[45][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 47}, MIB_INT(snmp_enmv_data[46][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 48}, MIB_INT(snmp_enmv_data[47][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 49}, MIB_INT(snmp_enmv_data[48][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 50}, MIB_INT(snmp_enmv_data[49][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 51}, MIB_INT(snmp_enmv_data[50][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 52}, MIB_INT(snmp_enmv_data[51][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 53}, MIB_INT(snmp_enmv_data[52][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 54}, MIB_INT(snmp_enmv_data[53][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 55}, MIB_INT(snmp_enmv_data[54][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 56}, MIB_INT(snmp_enmv_data[55][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 57}, MIB_INT(snmp_enmv_data[56][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 58}, MIB_INT(snmp_enmv_data[57][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 59}, MIB_INT(snmp_enmv_data[58][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 60}, MIB_INT(snmp_enmv_data[59][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 61}, MIB_INT(snmp_enmv_data[60][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 62}, MIB_INT(snmp_enmv_data[61][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 63}, MIB_INT(snmp_enmv_data[62][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_3, 64}, MIB_INT(snmp_enmv_data[63][2]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 1}, MIB_INT(snmp_enmv_data[0][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 2}, MIB_INT(snmp_enmv_data[1][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 3}, MIB_INT(snmp_enmv_data[2][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 4}, MIB_INT(snmp_enmv_data[3][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 5}, MIB_INT(snmp_enmv_data[4][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 6}, MIB_INT(snmp_enmv_data[5][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 7}, MIB_INT(snmp_enmv_data[6][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 8}, MIB_INT(snmp_enmv_data[7][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 9}, MIB_INT(snmp_enmv_data[8][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 10}, MIB_INT(snmp_enmv_data[9][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 11}, MIB_INT(snmp_enmv_data[10][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 12}, MIB_INT(snmp_enmv_data[11][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 13}, MIB_INT(snmp_enmv_data[12][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 14}, MIB_INT(snmp_enmv_data[13][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 15}, MIB_INT(snmp_enmv_data[14][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 16}, MIB_INT(snmp_enmv_data[15][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 17}, MIB_INT(snmp_enmv_data[16][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 18}, MIB_INT(snmp_enmv_data[17][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 19}, MIB_INT(snmp_enmv_data[18][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 20}, MIB_INT(snmp_enmv_data[19][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 21}, MIB_INT(snmp_enmv_data[20][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 22}, MIB_INT(snmp_enmv_data[21][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 23}, MIB_INT(snmp_enmv_data[22][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 24}, MIB_INT(snmp_enmv_data[23][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 25}, MIB_INT(snmp_enmv_data[24][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 26}, MIB_INT(snmp_enmv_data[25][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 27}, MIB_INT(snmp_enmv_data[26][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 28}, MIB_INT(snmp_enmv_data[27][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 29}, MIB_INT(snmp_enmv_data[28][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 30}, MIB_INT(snmp_enmv_data[29][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 31}, MIB_INT(snmp_enmv_data[30][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 32}, MIB_INT(snmp_enmv_data[31][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 33}, MIB_INT(snmp_enmv_data[32][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 34}, MIB_INT(snmp_enmv_data[33][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 35}, MIB_INT(snmp_enmv_data[34][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 36}, MIB_INT(snmp_enmv_data[35][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 37}, MIB_INT(snmp_enmv_data[36][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 38}, MIB_INT(snmp_enmv_data[37][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 39}, MIB_INT(snmp_enmv_data[38][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 40}, MIB_INT(snmp_enmv_data[39][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 41}, MIB_INT(snmp_enmv_data[40][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 42}, MIB_INT(snmp_enmv_data[41][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 43}, MIB_INT(snmp_enmv_data[42][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 44}, MIB_INT(snmp_enmv_data[43][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 45}, MIB_INT(snmp_enmv_data[44][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 46}, MIB_INT(snmp_enmv_data[45][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 47}, MIB_INT(snmp_enmv_data[46][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 48}, MIB_INT(snmp_enmv_data[47][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 49}, MIB_INT(snmp_enmv_data[48][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 50}, MIB_INT(snmp_enmv_data[49][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 51}, MIB_INT(snmp_enmv_data[50][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 52}, MIB_INT(snmp_enmv_data[51][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 53}, MIB_INT(snmp_enmv_data[52][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 54}, MIB_INT(snmp_enmv_data[53][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 55}, MIB_INT(snmp_enmv_data[54][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 56}, MIB_INT(snmp_enmv_data[55][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 57}, MIB_INT(snmp_enmv_data[56][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 58}, MIB_INT(snmp_enmv_data[57][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 59}, MIB_INT(snmp_enmv_data[58][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 60}, MIB_INT(snmp_enmv_data[59][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 61}, MIB_INT(snmp_enmv_data[60][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 62}, MIB_INT(snmp_enmv_data[61][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 63}, MIB_INT(snmp_enmv_data[62][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_4, 64}, MIB_INT(snmp_enmv_data[63][3]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 1}, MIB_INT(snmp_enmv_data[0][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 2}, MIB_INT(snmp_enmv_data[1][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 3}, MIB_INT(snmp_enmv_data[2][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 4}, MIB_INT(snmp_enmv_data[3][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 5}, MIB_INT(snmp_enmv_data[4][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 6}, MIB_INT(snmp_enmv_data[5][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 7}, MIB_INT(snmp_enmv_data[6][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 8}, MIB_INT(snmp_enmv_data[7][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 9}, MIB_INT(snmp_enmv_data[8][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 10}, MIB_INT(snmp_enmv_data[9][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 11}, MIB_INT(snmp_enmv_data[10][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 12}, MIB_INT(snmp_enmv_data[11][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 13}, MIB_INT(snmp_enmv_data[12][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 14}, MIB_INT(snmp_enmv_data[13][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 15}, MIB_INT(snmp_enmv_data[14][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 16}, MIB_INT(snmp_enmv_data[15][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 17}, MIB_INT(snmp_enmv_data[16][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 18}, MIB_INT(snmp_enmv_data[17][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 19}, MIB_INT(snmp_enmv_data[18][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 20}, MIB_INT(snmp_enmv_data[19][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 21}, MIB_INT(snmp_enmv_data[20][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 22}, MIB_INT(snmp_enmv_data[21][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 23}, MIB_INT(snmp_enmv_data[22][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 24}, MIB_INT(snmp_enmv_data[23][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 25}, MIB_INT(snmp_enmv_data[24][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 26}, MIB_INT(snmp_enmv_data[25][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 27}, MIB_INT(snmp_enmv_data[26][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 28}, MIB_INT(snmp_enmv_data[27][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 29}, MIB_INT(snmp_enmv_data[28][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 30}, MIB_INT(snmp_enmv_data[29][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 31}, MIB_INT(snmp_enmv_data[30][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 32}, MIB_INT(snmp_enmv_data[31][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 33}, MIB_INT(snmp_enmv_data[32][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 34}, MIB_INT(snmp_enmv_data[33][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 35}, MIB_INT(snmp_enmv_data[34][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 36}, MIB_INT(snmp_enmv_data[35][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 37}, MIB_INT(snmp_enmv_data[36][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 38}, MIB_INT(snmp_enmv_data[37][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 39}, MIB_INT(snmp_enmv_data[38][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 40}, MIB_INT(snmp_enmv_data[39][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 41}, MIB_INT(snmp_enmv_data[40][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 42}, MIB_INT(snmp_enmv_data[41][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 43}, MIB_INT(snmp_enmv_data[42][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 44}, MIB_INT(snmp_enmv_data[43][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 45}, MIB_INT(snmp_enmv_data[44][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 46}, MIB_INT(snmp_enmv_data[45][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 47}, MIB_INT(snmp_enmv_data[46][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 48}, MIB_INT(snmp_enmv_data[47][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 49}, MIB_INT(snmp_enmv_data[48][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 50}, MIB_INT(snmp_enmv_data[49][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 51}, MIB_INT(snmp_enmv_data[50][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 52}, MIB_INT(snmp_enmv_data[51][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 53}, MIB_INT(snmp_enmv_data[52][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 54}, MIB_INT(snmp_enmv_data[53][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 55}, MIB_INT(snmp_enmv_data[54][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 56}, MIB_INT(snmp_enmv_data[55][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 57}, MIB_INT(snmp_enmv_data[56][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 58}, MIB_INT(snmp_enmv_data[57][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 59}, MIB_INT(snmp_enmv_data[58][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 60}, MIB_INT(snmp_enmv_data[59][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 61}, MIB_INT(snmp_enmv_data[60][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 62}, MIB_INT(snmp_enmv_data[61][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 63}, MIB_INT(snmp_enmv_data[62][4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_5, 64}, MIB_INT(snmp_enmv_data[63][4]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 1}, MIB_INT(snmp_enmv_data[0][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 2}, MIB_INT(snmp_enmv_data[1][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 3}, MIB_INT(snmp_enmv_data[2][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 4}, MIB_INT(snmp_enmv_data[3][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 5}, MIB_INT(snmp_enmv_data[4][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 6}, MIB_INT(snmp_enmv_data[5][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 7}, MIB_INT(snmp_enmv_data[6][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 8}, MIB_INT(snmp_enmv_data[7][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 9}, MIB_INT(snmp_enmv_data[8][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 10}, MIB_INT(snmp_enmv_data[9][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 11}, MIB_INT(snmp_enmv_data[10][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 12}, MIB_INT(snmp_enmv_data[11][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 13}, MIB_INT(snmp_enmv_data[12][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 14}, MIB_INT(snmp_enmv_data[13][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 15}, MIB_INT(snmp_enmv_data[14][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 16}, MIB_INT(snmp_enmv_data[15][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 17}, MIB_INT(snmp_enmv_data[16][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 18}, MIB_INT(snmp_enmv_data[17][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 19}, MIB_INT(snmp_enmv_data[18][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 20}, MIB_INT(snmp_enmv_data[19][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 21}, MIB_INT(snmp_enmv_data[20][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 22}, MIB_INT(snmp_enmv_data[21][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 23}, MIB_INT(snmp_enmv_data[22][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 24}, MIB_INT(snmp_enmv_data[23][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 25}, MIB_INT(snmp_enmv_data[24][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 26}, MIB_INT(snmp_enmv_data[25][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 27}, MIB_INT(snmp_enmv_data[26][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 28}, MIB_INT(snmp_enmv_data[27][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 29}, MIB_INT(snmp_enmv_data[28][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 30}, MIB_INT(snmp_enmv_data[29][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 31}, MIB_INT(snmp_enmv_data[30][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 32}, MIB_INT(snmp_enmv_data[31][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 33}, MIB_INT(snmp_enmv_data[32][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 34}, MIB_INT(snmp_enmv_data[33][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 35}, MIB_INT(snmp_enmv_data[34][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 36}, MIB_INT(snmp_enmv_data[35][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 37}, MIB_INT(snmp_enmv_data[36][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 38}, MIB_INT(snmp_enmv_data[37][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 39}, MIB_INT(snmp_enmv_data[38][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 40}, MIB_INT(snmp_enmv_data[39][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 41}, MIB_INT(snmp_enmv_data[40][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 42}, MIB_INT(snmp_enmv_data[41][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 43}, MIB_INT(snmp_enmv_data[42][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 44}, MIB_INT(snmp_enmv_data[43][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 45}, MIB_INT(snmp_enmv_data[44][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 46}, MIB_INT(snmp_enmv_data[45][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 47}, MIB_INT(snmp_enmv_data[46][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 48}, MIB_INT(snmp_enmv_data[47][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 49}, MIB_INT(snmp_enmv_data[48][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 50}, MIB_INT(snmp_enmv_data[49][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 51}, MIB_INT(snmp_enmv_data[50][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 52}, MIB_INT(snmp_enmv_data[51][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 53}, MIB_INT(snmp_enmv_data[52][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 54}, MIB_INT(snmp_enmv_data[53][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 55}, MIB_INT(snmp_enmv_data[54][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 56}, MIB_INT(snmp_enmv_data[55][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 57}, MIB_INT(snmp_enmv_data[56][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 58}, MIB_INT(snmp_enmv_data[57][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 59}, MIB_INT(snmp_enmv_data[58][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 60}, MIB_INT(snmp_enmv_data[59][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 61}, MIB_INT(snmp_enmv_data[60][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 62}, MIB_INT(snmp_enmv_data[61][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 63}, MIB_INT(snmp_enmv_data[62][5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_6, 64}, MIB_INT(snmp_enmv_data[63][5]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 1}, MIB_INT(snmp_enmv_data[0][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 2}, MIB_INT(snmp_enmv_data[1][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 3}, MIB_INT(snmp_enmv_data[2][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 4}, MIB_INT(snmp_enmv_data[3][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 5}, MIB_INT(snmp_enmv_data[4][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 6}, MIB_INT(snmp_enmv_data[5][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 7}, MIB_INT(snmp_enmv_data[6][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 8}, MIB_INT(snmp_enmv_data[7][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 9}, MIB_INT(snmp_enmv_data[8][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 10}, MIB_INT(snmp_enmv_data[9][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 11}, MIB_INT(snmp_enmv_data[10][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 12}, MIB_INT(snmp_enmv_data[11][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 13}, MIB_INT(snmp_enmv_data[12][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 14}, MIB_INT(snmp_enmv_data[13][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 15}, MIB_INT(snmp_enmv_data[14][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 16}, MIB_INT(snmp_enmv_data[15][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 17}, MIB_INT(snmp_enmv_data[16][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 18}, MIB_INT(snmp_enmv_data[17][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 19}, MIB_INT(snmp_enmv_data[18][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 20}, MIB_INT(snmp_enmv_data[19][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 21}, MIB_INT(snmp_enmv_data[20][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 22}, MIB_INT(snmp_enmv_data[21][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 23}, MIB_INT(snmp_enmv_data[22][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 24}, MIB_INT(snmp_enmv_data[23][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 25}, MIB_INT(snmp_enmv_data[24][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 26}, MIB_INT(snmp_enmv_data[25][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 27}, MIB_INT(snmp_enmv_data[26][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 28}, MIB_INT(snmp_enmv_data[27][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 29}, MIB_INT(snmp_enmv_data[28][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 30}, MIB_INT(snmp_enmv_data[29][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 31}, MIB_INT(snmp_enmv_data[30][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 32}, MIB_INT(snmp_enmv_data[31][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 33}, MIB_INT(snmp_enmv_data[32][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 34}, MIB_INT(snmp_enmv_data[33][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 35}, MIB_INT(snmp_enmv_data[34][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 36}, MIB_INT(snmp_enmv_data[35][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 37}, MIB_INT(snmp_enmv_data[36][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 38}, MIB_INT(snmp_enmv_data[37][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 39}, MIB_INT(snmp_enmv_data[38][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 40}, MIB_INT(snmp_enmv_data[39][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 41}, MIB_INT(snmp_enmv_data[40][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 42}, MIB_INT(snmp_enmv_data[41][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 43}, MIB_INT(snmp_enmv_data[42][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 44}, MIB_INT(snmp_enmv_data[43][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 45}, MIB_INT(snmp_enmv_data[44][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 46}, MIB_INT(snmp_enmv_data[45][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 47}, MIB_INT(snmp_enmv_data[46][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 48}, MIB_INT(snmp_enmv_data[47][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 49}, MIB_INT(snmp_enmv_data[48][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 50}, MIB_INT(snmp_enmv_data[49][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 51}, MIB_INT(snmp_enmv_data[50][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 52}, MIB_INT(snmp_enmv_data[51][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 53}, MIB_INT(snmp_enmv_data[52][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 54}, MIB_INT(snmp_enmv_data[53][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 55}, MIB_INT(snmp_enmv_data[54][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 56}, MIB_INT(snmp_enmv_data[55][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 57}, MIB_INT(snmp_enmv_data[56][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 58}, MIB_INT(snmp_enmv_data[57][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 59}, MIB_INT(snmp_enmv_data[58][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 60}, MIB_INT(snmp_enmv_data[59][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 61}, MIB_INT(snmp_enmv_data[60][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 62}, MIB_INT(snmp_enmv_data[61][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 63}, MIB_INT(snmp_enmv_data[62][6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_7, 64}, MIB_INT(snmp_enmv_data[63][6]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 1}, MIB_INT(snmp_enmv_data[0][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 2}, MIB_INT(snmp_enmv_data[1][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 3}, MIB_INT(snmp_enmv_data[2][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 4}, MIB_INT(snmp_enmv_data[3][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 5}, MIB_INT(snmp_enmv_data[4][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 6}, MIB_INT(snmp_enmv_data[5][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 7}, MIB_INT(snmp_enmv_data[6][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 8}, MIB_INT(snmp_enmv_data[7][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 9}, MIB_INT(snmp_enmv_data[8][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 10}, MIB_INT(snmp_enmv_data[9][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 11}, MIB_INT(snmp_enmv_data[10][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 12}, MIB_INT(snmp_enmv_data[11][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 13}, MIB_INT(snmp_enmv_data[12][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 14}, MIB_INT(snmp_enmv_data[13][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 15}, MIB_INT(snmp_enmv_data[14][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 16}, MIB_INT(snmp_enmv_data[15][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 17}, MIB_INT(snmp_enmv_data[16][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 18}, MIB_INT(snmp_enmv_data[17][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 19}, MIB_INT(snmp_enmv_data[18][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 20}, MIB_INT(snmp_enmv_data[19][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 21}, MIB_INT(snmp_enmv_data[20][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 22}, MIB_INT(snmp_enmv_data[21][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 23}, MIB_INT(snmp_enmv_data[22][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 24}, MIB_INT(snmp_enmv_data[23][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 25}, MIB_INT(snmp_enmv_data[24][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 26}, MIB_INT(snmp_enmv_data[25][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 27}, MIB_INT(snmp_enmv_data[26][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 28}, MIB_INT(snmp_enmv_data[27][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 29}, MIB_INT(snmp_enmv_data[28][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 30}, MIB_INT(snmp_enmv_data[29][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 31}, MIB_INT(snmp_enmv_data[30][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 32}, MIB_INT(snmp_enmv_data[31][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 33}, MIB_INT(snmp_enmv_data[32][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 34}, MIB_INT(snmp_enmv_data[33][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 35}, MIB_INT(snmp_enmv_data[34][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 36}, MIB_INT(snmp_enmv_data[35][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 37}, MIB_INT(snmp_enmv_data[36][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 38}, MIB_INT(snmp_enmv_data[37][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 39}, MIB_INT(snmp_enmv_data[38][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 40}, MIB_INT(snmp_enmv_data[39][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 41}, MIB_INT(snmp_enmv_data[40][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 42}, MIB_INT(snmp_enmv_data[41][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 43}, MIB_INT(snmp_enmv_data[42][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 44}, MIB_INT(snmp_enmv_data[43][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 45}, MIB_INT(snmp_enmv_data[44][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 46}, MIB_INT(snmp_enmv_data[45][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 47}, MIB_INT(snmp_enmv_data[46][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 48}, MIB_INT(snmp_enmv_data[47][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 49}, MIB_INT(snmp_enmv_data[48][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 50}, MIB_INT(snmp_enmv_data[49][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 51}, MIB_INT(snmp_enmv_data[50][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 52}, MIB_INT(snmp_enmv_data[51][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 53}, MIB_INT(snmp_enmv_data[52][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 54}, MIB_INT(snmp_enmv_data[53][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 55}, MIB_INT(snmp_enmv_data[54][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 56}, MIB_INT(snmp_enmv_data[55][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 57}, MIB_INT(snmp_enmv_data[56][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 58}, MIB_INT(snmp_enmv_data[57][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 59}, MIB_INT(snmp_enmv_data[58][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 60}, MIB_INT(snmp_enmv_data[59][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 61}, MIB_INT(snmp_enmv_data[60][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 62}, MIB_INT(snmp_enmv_data[61][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 63}, MIB_INT(snmp_enmv_data[62][7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENMV, DISPLAY_ENMV_DATA_8, 64}, MIB_INT(snmp_enmv_data[63][7]),	NULL},

	//----------  Данные LVBD
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_STATUS}, 		MIB_INT(LVBD_status),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_Uips}, 			MIB_INT(lvbd_Uload),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_Uakb}, 			MIB_INT(lvbd_Uakb),	NULL},
	{ MIB_INTEGER ,				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_Uips_enable}, 	MIB_INT(LVBD_Uload_rele_en),	snmp_LVBD_Uload_rele_en},
	{ MIB_INTEGER ,				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_Uakb_enable}, 	MIB_INT(LVBD_Uakb_rele_en),	snmp_LVBD_Uakb_rele_en},
	{ MIB_INTEGER ,				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_U_disable}, 	MIB_INT(LVBD_porog_U1),	snmp_LVBD_porog_U1},
	{ MIB_INTEGER ,				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_U_disable_alarm},MIB_INT(LVBD_porog_U2),	snmp_LVBD_porog_U2},
	{ MIB_INTEGER ,				12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LVBD, DISPLAY_LVBD_DELAY}, 		MIB_INT(LVBD_num_meas),	snmp_LVBD_num_meas},
	
//o_12_e
	//o_2_e
	//o_3_s
	//------------  данные РКИ
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 1, 0}, MIB_INT(ver_soft),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 2, 0}, MIB_INT(status_izm_r),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 3, 0}, MIB_INT(r_iz_plus),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 4, 0}, MIB_INT(r_iz_minus),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 5, 0}, MIB_INT(r_iz_porog_pred),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 6, 0}, MIB_INT(r_iz_porog_error),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 7, 0}, MIB_INT(v_plus),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 8, 0}, MIB_INT(v_minus),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 9, 0}, MIB_INT(Ubus),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 10,0}, MIB_INT(porog_u_in),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 11,0}, MIB_INT(asymmetry),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 12,0}, MIB_INT(u_asymmetry),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 13,0}, MIB_INT(asymmetry_porog),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 14,0}, MIB_INT(u_asymmetry_porog_up),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 15,0}, MIB_INT(u_asymmetry_porog),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 16,0}, MIB_INT(u_asymmetry_porog_down),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 17,0}, MIB_INT(Iddt_porog_pred),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 18,0}, MIB_INT(Iddt_porog_error),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI, 19,0}, MIB_INT(n_error_ddt_uku),	NULL},
	//---------------- таблица сухих контактов
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,1}, MIB_INT(snmp_enmv_number[0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,2}, MIB_INT(snmp_enmv_number[1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,3}, MIB_INT(snmp_enmv_number[2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,4}, MIB_INT(snmp_enmv_number[3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,5}, MIB_INT(snmp_enmv_number[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,6}, MIB_INT(snmp_enmv_number[5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,7}, MIB_INT(snmp_enmv_number[6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,8}, MIB_INT(snmp_enmv_number[7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,9}, MIB_INT(snmp_enmv_number[8]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,10}, MIB_INT(snmp_enmv_number[9]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,11}, MIB_INT(snmp_enmv_number[10]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,12}, MIB_INT(snmp_enmv_number[11]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,13}, MIB_INT(snmp_enmv_number[12]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,14}, MIB_INT(snmp_enmv_number[13]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,15}, MIB_INT(snmp_enmv_number[14]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,16}, MIB_INT(snmp_enmv_number[15]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,17}, MIB_INT(snmp_enmv_number[16]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,18}, MIB_INT(snmp_enmv_number[17]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,19}, MIB_INT(snmp_enmv_number[18]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,20}, MIB_INT(snmp_enmv_number[19]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,21}, MIB_INT(snmp_enmv_number[20]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,22}, MIB_INT(snmp_enmv_number[21]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,23}, MIB_INT(snmp_enmv_number[22]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_NUMBER,24}, MIB_INT(snmp_enmv_number[23]),	NULL},
		
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,1}, MIB_INT(sk1_24_table[0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,2}, MIB_INT(sk1_24_table[1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,3}, MIB_INT(sk1_24_table[2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,4}, MIB_INT(sk1_24_table[3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,5}, MIB_INT(sk1_24_table[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,6}, MIB_INT(sk1_24_table[5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,7}, MIB_INT(sk1_24_table[6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,8}, MIB_INT(sk1_24_table[7]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,9}, MIB_INT(sk1_24_table[8]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,10}, MIB_INT(sk1_24_table[9]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,11}, MIB_INT(sk1_24_table[10]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,12}, MIB_INT(sk1_24_table[11]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,13}, MIB_INT(sk1_24_table[12]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,14}, MIB_INT(sk1_24_table[13]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,15}, MIB_INT(sk1_24_table[14]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,16}, MIB_INT(sk1_24_table[15]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,17}, MIB_INT(sk1_24_table[16]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,18}, MIB_INT(sk1_24_table[17]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,19}, MIB_INT(sk1_24_table[18]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,20}, MIB_INT(sk1_24_table[19]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,21}, MIB_INT(sk1_24_table[20]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,22}, MIB_INT(sk1_24_table[21]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,23}, MIB_INT(sk1_24_table[22]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_DATA,24}, MIB_INT(sk1_24_table[23]),	NULL},
	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,1}, MIB_INT(sk_alarm_table[0]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,2}, MIB_INT(sk_alarm_table[1]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,3}, MIB_INT(sk_alarm_table[2]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,4}, MIB_INT(sk_alarm_table[3]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,5}, MIB_INT(sk_alarm_table[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,6}, MIB_INT(sk_alarm_table[5]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,7}, MIB_INT(sk_alarm_table[6]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,8}, MIB_INT(sk_alarm_table[7]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,9}, MIB_INT(sk_alarm_table[8]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,10}, MIB_INT(sk_alarm_table[9]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,11}, MIB_INT(sk_alarm_table[10]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,12}, MIB_INT(sk_alarm_table[11]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,13}, MIB_INT(sk_alarm_table[12]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,14}, MIB_INT(sk_alarm_table[13]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,15}, MIB_INT(sk_alarm_table[14]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,16}, MIB_INT(sk_alarm_table[15]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,17}, MIB_INT(sk_alarm_table[16]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,18}, MIB_INT(sk_alarm_table[17]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,19}, MIB_INT(sk_alarm_table[18]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,20}, MIB_INT(sk_alarm_table[19]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,21}, MIB_INT(sk_alarm_table[20]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,22}, MIB_INT(sk_alarm_table[21]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,23}, MIB_INT(sk_alarm_table[22]),	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_SK, DISPLAY_RKI_SK_ERROR,24}, MIB_INT(sk_alarm_table[23]),	NULL},	
	//-------------- Таблица аварий датчиков тока
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,1}, MIB_INT(snmp_enmv_number[0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,2}, MIB_INT(snmp_enmv_number[1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,3}, MIB_INT(snmp_enmv_number[2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,4}, MIB_INT(snmp_enmv_number[3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,5}, MIB_INT(snmp_enmv_number[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,6}, MIB_INT(snmp_enmv_number[5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,7}, MIB_INT(snmp_enmv_number[6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUMBER,8}, MIB_INT(snmp_enmv_number[7]),	NULL},
	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,1}, MIB_INT(Rddt[0][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,2}, MIB_INT(Rddt[1][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,3}, MIB_INT(Rddt[2][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,4}, MIB_INT(Rddt[3][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,5}, MIB_INT(Rddt[4][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,6}, MIB_INT(Rddt[5][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,7}, MIB_INT(Rddt[6][0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_NUM_ALARM,8}, MIB_INT(Rddt[7][0]),	NULL},	
	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,1}, MIB_INT(ddt_error_table[0]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,2}, MIB_INT(ddt_error_table[1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,3}, MIB_INT(ddt_error_table[2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,4}, MIB_INT(ddt_error_table[3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,5}, MIB_INT(ddt_error_table[4]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,6}, MIB_INT(ddt_error_table[5]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,7}, MIB_INT(ddt_error_table[6]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_ALARM,8}, MIB_INT(ddt_error_table[7]),	NULL},
	
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,1}, MIB_INT(Rddt[0][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,2}, MIB_INT(Rddt[1][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,3}, MIB_INT(Rddt[2][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,4}, MIB_INT(Rddt[3][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,5}, MIB_INT(Rddt[4][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,6}, MIB_INT(Rddt[5][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,7}, MIB_INT(Rddt[6][3]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rplus,8}, MIB_INT(Rddt[7][3]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,1}, MIB_INT(Rddt[0][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,2}, MIB_INT(Rddt[1][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,3}, MIB_INT(Rddt[2][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,4}, MIB_INT(Rddt[3][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,5}, MIB_INT(Rddt[4][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,6}, MIB_INT(Rddt[5][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,7}, MIB_INT(Rddt[6][2]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rminus,8}, MIB_INT(Rddt[7][2]),	NULL},

	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,1}, MIB_INT(Rddt[0][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,2}, MIB_INT(Rddt[1][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,3}, MIB_INT(Rddt[2][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,4}, MIB_INT(Rddt[3][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,5}, MIB_INT(Rddt[4][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,6}, MIB_INT(Rddt[5][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,7}, MIB_INT(Rddt[6][1]),	NULL},
	{ MIB_INTEGER | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_RKI_DDT, DISPLAY_RKI_DDT_Rparal,8}, MIB_INT(Rddt[7][1]),	NULL},

	//o_3_e

	};
#endif
																														  
#ifdef UKU_KONTUR																									 
/*----------------------------------------------------------------------------													 
 *      MIB Data Table
 *---------------------------------------------------------------------------*/

 MIB_ENTRY snmp_mib[] = {

  /* ---------- System MIB ----------- */

  /* SysDescr Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 1, 0},      MIB_STR("First ARM SNMP agent for SibPromAutomatika"),     NULL },
  /* SysObjectID Entry */
  { MIB_OBJECT_ID | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 2, 0},	    MIB_STR("\x2b\x06\x01\x04\x01\x82\x83\x1F"),    NULL },
  /* SysUpTime Entry */
  { MIB_TIME_TICKS | MIB_ATR_RO,     8, {OID0(1,3), 6, 1, 2, 1, 1, 3, 0},    4, &snmp_SysUpTime,    NULL },
  /* SysContact Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,	     8, {OID0(1,3), 6, 1, 2, 1, 1, 4, 0},    MIB_STR("Skype:danilov_aa"),    NULL },
  /* SysName Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		    8, {OID0(1,3), 6, 1, 2, 1, 1, 5, 0},    MIB_STR("UKU203LAN"),    NULL },
  /* SysLocation Entry */
  { MIB_OCTET_STR | MIB_ATR_RO,		     8, {OID0(1,3), 6, 1, 2, 1, 1, 6, 0},    MIB_STR("Novosibirsk, Russia"),    NULL },
  /* SysServices Entry */
  { MIB_INTEGER | MIB_ATR_RO,			    8, {OID0(1,3), 6, 1, 2, 1, 1, 7, 0},    MIB_INT(sysServices),    NULL },

  /* ---------- Experimental MIB ----------- */

	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_0 , 0},			MIB_INT(snmp_spc_trap_value_0),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_1 , 0},			MIB_INT(snmp_spc_trap_value_1),     NULL},
	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE_2 , 0},			MIB_INT(snmp_spc_trap_value_2),     NULL},


  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_CODE, 0},  	MIB_INT(snmp_device_code),  		NULL},   				//код устройства
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_SERIAL, 0},	MIB_INT(snmp_sernum),	  		NULL },				//серийный номер	
  	{ MIB_OCTET_STR, 			12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_LOCATION, 0},  	MIB_STR(snmp_location),  		snmp_location_write},	//местоположение устройства
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBAT, 0}, 	MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFBPS, 0},	MIB_INT(snmp_numofbps),  		NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFINV, 0},	MIB_INT(snmp_numofinv),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFAVT, 0}, 	MIB_INT(snmp_numofavt),  			NULL},				//количество введенных батарей
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFDT, 0},	MIB_INT(snmp_numofdt),  			NULL},				//количество введенных источников
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFSK, 0},	MIB_INT(snmp_numofsk),  			NULL},				//количество введенных источников
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DEVICE_INFO, DISPLAY_DEVICE_INFO_NUMOFEVENTS, 0},MIB_INT(snmp_numofbat),  		NULL},				//количество введенных батарей

/**/	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE, 0},  	MIB_INT(snmp_mains_power_voltage), NULL},	//напряжение сети	
/**/	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_FREQUENCY, 0},  MIB_INT(snmp_mains_power_frequency),NULL},	//частота сети
/**/	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_STATUS, 0},  	MIB_INT(snmp_mains_power_status),  NULL},	//состояние сети 
// 	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_ALARM, 0},  	MIB_INT(snmp_mains_power_alarm),  	NULL},	//аварии сети
//  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEA, 0},  	MIB_INT(snmp_mains_power_voltage_phaseA), NULL},	//напряжение сети	
//  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEB, 0},  	MIB_INT(snmp_mains_power_voltage_phaseB), NULL},	//напряжение сети	
//  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_MAINS_POWER, DISPLAY_MAINS_POWER_VOLTAGE_PHASEC, 0},  	MIB_INT(snmp_mains_power_voltage_phaseC), NULL},	//напряжение сети	


	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_VOLTAGE, 0},  				MIB_INT(snmp_load_voltage),  		NULL},	//напряжение нагрузки
  	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOAD, DISPLAY_LOAD_CURRENT, 0},  				MIB_INT(snmp_load_current),  		NULL},	//ток нагрузки

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 1},  			MIB_INT(snmp_bps_number[0]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 2},  			MIB_INT(snmp_bps_number[1]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 3},  			MIB_INT(snmp_bps_number[2]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 4},  			MIB_INT(snmp_bps_number[3]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 5},  			MIB_INT(snmp_bps_number[4]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 6},  			MIB_INT(snmp_bps_number[5]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 7},  			MIB_INT(snmp_bps_number[6]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_NUMBER, 8},  			MIB_INT(snmp_bps_number[7]),  	NULL},	//Номер БПСа

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 1},  			MIB_INT(snmp_bps_voltage[0]),  	NULL},	//Напряжение БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 2},  			MIB_INT(snmp_bps_voltage[1]),  	NULL},	//Напряжение БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 3},  			MIB_INT(snmp_bps_voltage[2]),  	NULL},	//Напряжение БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 4},  			MIB_INT(snmp_bps_voltage[3]),  	NULL},	//Напряжение БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 5},  			MIB_INT(snmp_bps_voltage[4]),  	NULL},	//Напряжение БПС4
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 6},  			MIB_INT(snmp_bps_voltage[5]),  	NULL},	//Напряжение БПС5
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 7},  			MIB_INT(snmp_bps_voltage[6]),  	NULL},	//Напряжение БПС6
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_VOLTAGE, 8},  			MIB_INT(snmp_bps_voltage[7]),  	NULL},	//Напряжение БПС7

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 1},  			MIB_INT(snmp_bps_current[0]),  	NULL},	//Ток БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 2},  			MIB_INT(snmp_bps_current[1]),  	NULL},	//Ток БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 3},  			MIB_INT(snmp_bps_current[2]),  	NULL},	//Ток БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 4},  			MIB_INT(snmp_bps_current[3]),  	NULL},	//Ток БПС4
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 5},  			MIB_INT(snmp_bps_current[4]),  	NULL},	//Ток БПС5
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 6},  			MIB_INT(snmp_bps_current[5]),  	NULL},	//Ток БПС6
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 7},  			MIB_INT(snmp_bps_current[6]),  	NULL},	//Ток БПС7
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_CURRENT, 8},  			MIB_INT(snmp_bps_current[7]),  	NULL},	//Ток БПС8

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 1},  		MIB_INT(snmp_bps_temperature[0]),  NULL},	//Ток БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 2},  		MIB_INT(snmp_bps_temperature[1]),  NULL},	//Ток БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 3},  		MIB_INT(snmp_bps_temperature[2]),  NULL},	//Ток БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 4},  		MIB_INT(snmp_bps_temperature[3]),  NULL},	//Ток БПС4
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 5},  		MIB_INT(snmp_bps_temperature[4]),  NULL},	//Ток БПС5
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 6},  		MIB_INT(snmp_bps_temperature[5]),  NULL},	//Ток БПС6
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 7},  		MIB_INT(snmp_bps_temperature[6]),  NULL},	//Ток БПС7
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_TEMPERATURE, 8},  		MIB_INT(snmp_bps_temperature[7]),  NULL},	//Ток БПС8

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 1},  			MIB_INT(snmp_bps_stat[0]),  NULL},			//Состояние БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 2},  			MIB_INT(snmp_bps_stat[1]),  NULL},			//Состояние БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 3},  			MIB_INT(snmp_bps_stat[2]),  NULL},			//Состояние БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 4},  			MIB_INT(snmp_bps_stat[3]),  NULL},			//Состояние БПС4
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 5},  			MIB_INT(snmp_bps_stat[4]),  NULL},			//Состояние БПС5
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 6},  			MIB_INT(snmp_bps_stat[5]),  NULL},			//Состояние БПС6
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 7},  			MIB_INT(snmp_bps_stat[6]),  NULL},			//Состояние БПС7
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_PSU, DISPLAY_PSU_ENTRY_STATUS, 8},  			MIB_INT(snmp_bps_stat[7]),  NULL},			//Состояние БПС8

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 1},  				MIB_INT(snmp_bat_number[0]),  	NULL},	//Напряжение батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_NUMBER, 2},  				MIB_INT(snmp_bat_number[1]),  	NULL},	//Напряжение батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 1},  				MIB_INT(snmp_bat_voltage[0]),  	NULL},	//Напряжение батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_VOLTAGE, 2},  				MIB_INT(snmp_bat_voltage[1]),  	NULL},	//Напряжение батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_PART_VOLTAGE, 1},  			MIB_INT(snmp_bat_part_voltage[0]), NULL},	//Напряжение средней точки батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_PART_VOLTAGE, 2},  			MIB_INT(snmp_bat_part_voltage[1]), NULL},	//Напряжение средней точки батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 1},  				MIB_INT(snmp_bat_current[0]),  	NULL},	//Ток батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CURRENT, 2},  				MIB_INT(snmp_bat_current[1]),  	NULL},	//Ток батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 1},  			MIB_INT(snmp_bat_temperature[0]),	NULL},	//Температура батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_TEMPERATURE, 2},  			MIB_INT(snmp_bat_temperature[1]),	NULL},	//Температура батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 1},  				MIB_INT(snmp_bat_capacity[0]),  	NULL},	//Ёмкость батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CAPACITY, 2},  				MIB_INT(snmp_bat_capacity[1]),  	NULL},	//Ёмкость батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 1},  				MIB_INT(snmp_bat_charge[0]),  	NULL},	//Заряд батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_CHARGE, 2},  				MIB_INT(snmp_bat_charge[1]),  	NULL},	//Заряд батареи №2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 1},  				MIB_INT(snmp_bat_status[0]),  	NULL},	//Статус батареи №1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_BAT, DISPLAY_BAT_STATUS, 2},  				MIB_INT(snmp_bat_status[1]),  	NULL},	//Статус батареи №2


//	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_STAT , 0},					MIB_INT(snmp_spc_stat),     NULL},
//	{ MIB_OCTET_STR | MIB_ATR_RO, 12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_MESSAGE , 0},			MIB_STR(snmp_spc_trap_message),     NULL},
//	{ MIB_INTEGER | MIB_ATR_RO, 	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SPEC,DISPLAY_SPEC_TRAP_VALUE , 0},			MIB_INT(snmp_spc_trap_value),     NULL},


	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_ANSWER, 0},					MIB_INT(snmp_command),  	snmp_command_execute},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SNMP_COMMAND, COMMAND_PARAMETR, 0},					MIB_INT(snmp_command_parametr),  	NULL},		//номер первого бпса

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 1},  			MIB_INT(snmp_avt_number[0]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 2},  			MIB_INT(snmp_avt_number[1]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 3},  			MIB_INT(snmp_avt_number[2]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 4},  			MIB_INT(snmp_avt_number[3]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 5},  			MIB_INT(snmp_avt_number[4]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 6},  			MIB_INT(snmp_avt_number[5]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 7},  			MIB_INT(snmp_avt_number[6]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 8},  			MIB_INT(snmp_avt_number[7]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 9},  			MIB_INT(snmp_avt_number[8]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 10},  			MIB_INT(snmp_avt_number[9]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 11},  			MIB_INT(snmp_avt_number[10]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_NUMBER, 12},  			MIB_INT(snmp_avt_number[11]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 1},  			MIB_INT(snmp_avt_stat[0]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 2},  			MIB_INT(snmp_avt_stat[1]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 3},  			MIB_INT(snmp_avt_stat[2]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 4},  			MIB_INT(snmp_avt_stat[3]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 5},  			MIB_INT(snmp_avt_stat[4]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 6},  			MIB_INT(snmp_avt_stat[5]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 7},  			MIB_INT(snmp_avt_stat[6]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 8},  			MIB_INT(snmp_avt_stat[7]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 9},  			MIB_INT(snmp_avt_stat[8]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 10},  			MIB_INT(snmp_avt_stat[9]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 11},  			MIB_INT(snmp_avt_stat[10]),  		NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_AVT, DISPLAY_AVT_ENTRY_STAT, 12},  			MIB_INT(snmp_avt_stat[11]),  		NULL},	//Номер БПСа

	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_A, 0},		MIB_INT(snmp_energy_vvod_phase_a), NULL},	//напряжение фазы A ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_B, 0},		MIB_INT(snmp_energy_vvod_phase_b), NULL},	//напряжение фазы B ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_VVOD_PHASE_C, 0},		MIB_INT(snmp_energy_vvod_phase_c), NULL},	//напряжение фазы C ввода
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_A, 0},		MIB_INT(snmp_energy_pes_phase_a), NULL},	//напряжение фазы A ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_B, 0},		MIB_INT(snmp_energy_pes_phase_b), NULL},	//напряжение фазы B ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_PES_PHASE_C, 0},		MIB_INT(snmp_energy_pes_phase_c), NULL},	//напряжение фазы C ПЭСа
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_TOTAL_ENERGY, 0},		MIB_INT(snmp_energy_total_energy), NULL},	//показания счетчика, потребленная энергия
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_ENERGY, DISPLAY_ENERGY_CURRENT_ENERGY, 0},		MIB_INT(snmp_energy_current_energy), NULL},	//показания счетчика, потребляемая энергия



	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 4, 1, 0},  MIB_INT(NUMBAT),  NULL},	//количество введенных батарей

	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},	     MIB_STR("Novosibirsk, Russia"),     NULL},
	{ MIB_INTEGER, 			13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 5, 0},	     MIB_INT(displayPsuQauntity),     NULL},
 /* { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 1, 0},  MIB_INT(plazma_mib),  NULL},
  { MIB_INTEGER | MIB_ATR_RO,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 2, 2, 3, 2, 0},  MIB_INT(plazma_mib1),  NULL},
  { MIB_INTEGER,  	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 1, 2, 0},    MIB_INT(LPC_RTC->SEC),    NULL}, */
  
	


//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 1},  			MIB_STR(&snmp_log[0][0]),  	NULL},	//Первое событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 2},  			MIB_STR(&snmp_log[1][0]),  	NULL},	//2-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 3},  			MIB_STR(&snmp_log[2][0]),  	NULL},	//3-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 4},  			MIB_STR(&snmp_log[3][0]),  	NULL},	//4-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 5},  			MIB_STR(&snmp_log[4][0]),  	NULL},	//5-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 6},  			MIB_STR(&snmp_log[5][0]),  	NULL},	//6-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 7},  			MIB_STR(&snmp_log[6][0]),  	NULL},	//7-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 8},  			MIB_STR(&snmp_log[7][0]),  	NULL},	//8-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 9},  			MIB_STR(&snmp_log[8][0]),  	NULL},	//9-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 10},  			MIB_STR(&snmp_log[9][0]),  	NULL},	//10-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 11},  			MIB_STR(&snmp_log[10][0]),  	NULL},	//11-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 12},  			MIB_STR(&snmp_log[11][0]),  	NULL},	//12-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 13},  			MIB_STR(&snmp_log[12][0]),  	NULL},	//13-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 14},  			MIB_STR(&snmp_log[13][0]),  	NULL},	//14-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 15},  			MIB_STR(&snmp_log[14][0]),  	NULL},	//15-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 16},  			MIB_STR(&snmp_log[15][0]),  	NULL},	//16-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 17},  			MIB_STR(&snmp_log[16][0]),  	NULL},	//17-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 18},  			MIB_STR(&snmp_log[17][0]),  	NULL},	//18-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 19},  			MIB_STR(&snmp_log[18][0]),  	NULL},	//19-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 20},  			MIB_STR(&snmp_log[19][0]),  	NULL},	//20-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 21},  			MIB_STR(&snmp_log[20][0]),  	NULL},	//21-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 22},  			MIB_STR(&snmp_log[21][0]),  	NULL},	//22-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 23},  			MIB_STR(&snmp_log[22][0]),  	NULL},	//23-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 24},  			MIB_STR(&snmp_log[23][0]),  	NULL},	//24-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 25},  			MIB_STR(&snmp_log[24][0]),  	NULL},	//25-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 26},  			MIB_STR(&snmp_log[25][0]),  	NULL},	//26-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 27},  			MIB_STR(&snmp_log[26][0]),  	NULL},	//27-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 28},  			MIB_STR(&snmp_log[27][0]),  	NULL},	//28-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 29},  			MIB_STR(&snmp_log[28][0]),  	NULL},	//29-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 30},  			MIB_STR(&snmp_log[29][0]),  	NULL},	//30-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 31},  			MIB_STR(&snmp_log[30][0]),  	NULL},	//31-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 32},  			MIB_STR(&snmp_log[31][0]),  	NULL},	//32-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 33},  			MIB_STR(&snmp_log[32][0]),  	NULL},	//33-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 34},  			MIB_STR(&snmp_log[33][0]),  	NULL},	//34-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 35},  			MIB_STR(&snmp_log[34][0]),  	NULL},	//35-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 36},  			MIB_STR(&snmp_log[35][0]),  	NULL},	//36-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 37},  			MIB_STR(&snmp_log[36][0]),  	NULL},	//37-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 38},  			MIB_STR(&snmp_log[37][0]),  	NULL},	//38-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 39},  			MIB_STR(&snmp_log[38][0]),  	NULL},	//39-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 40},  			MIB_STR(&snmp_log[39][0]),  	NULL},	//40-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 41},  			MIB_STR(&snmp_log[40][0]),  	NULL},	//41-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 42},  			MIB_STR(&snmp_log[41][0]),  	NULL},	//42-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 43},  			MIB_STR(&snmp_log[42][0]),  	NULL},	//43-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 44},  			MIB_STR(&snmp_log[43][0]),  	NULL},	//44-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 45},  			MIB_STR(&snmp_log[44][0]),  	NULL},	//45-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 46},  			MIB_STR(&snmp_log[45][0]),  	NULL},	//46-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 47},  			MIB_STR(&snmp_log[46][0]),  	NULL},	//47-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 48},  			MIB_STR(&snmp_log[47][0]),  	NULL},	//48-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 49},  			MIB_STR(&snmp_log[48][0]),  	NULL},	//49-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 50},  			MIB_STR(&snmp_log[49][0]),  	NULL},	//50-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 51},  			MIB_STR(&snmp_log[50][0]),  	NULL},	//51-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 52},  			MIB_STR(&snmp_log[51][0]),  	NULL},	//52-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 53},  			MIB_STR(&snmp_log[52][0]),  	NULL},	//53-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 54},  			MIB_STR(&snmp_log[53][0]),  	NULL},	//54-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 55},  			MIB_STR(&snmp_log[54][0]),  	NULL},	//55-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 56},  			MIB_STR(&snmp_log[55][0]),  	NULL},	//56-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 57},  			MIB_STR(&snmp_log[56][0]),  	NULL},	//57-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 58},  			MIB_STR(&snmp_log[57][0]),  	NULL},	//58-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 59},  			MIB_STR(&snmp_log[58][0]),  	NULL},	//59-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 60},  			MIB_STR(&snmp_log[59][0]),  	NULL},	//60-е событие из журнала

//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 61},  			MIB_STR(&snmp_log[60][0]),  	NULL},	//61-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 62},  			MIB_STR(&snmp_log[61][0]),  	NULL},	//62-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 63},  			MIB_STR(&snmp_log[62][0]),  	NULL},	//63-е событие из журнала
//	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_LOG, DISPLAY_LOG_ENTRY_EVENTS, 64},  			MIB_STR(&snmp_log[63][0]),  	NULL},	//64-е событие из журнала



	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_BOX_TEMPER, 0},  				MIB_INT(snmp_klimat_box_temper),  	NULL},	//Номер БПСа
	{ MIB_INTEGER, 		  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_BOX_ALARM_TEMPER, 0},  	MIB_INT(snmp_klimat_settings_box_alarm),  	snmp_klimat_settings_box_alarm_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_VENT_ON, 0},  			MIB_INT(snmp_klimat_settings_vent_on),  	snmp_klimat_settings_vent_on_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_VENT_OFF, 0},  			MIB_INT(snmp_klimat_settings_vent_off),  	snmp_klimat_settings_vent_off_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_WARM_ON, 0},  			MIB_INT(snmp_klimat_settings_warm_on),  	snmp_klimat_settings_warm_on_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_WARM_OFF, 0},  			MIB_INT(snmp_klimat_settings_warm_off),  	snmp_klimat_settings_warm_off_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_LOAD_ON, 0},  			MIB_INT(snmp_klimat_settings_load_on),  	snmp_klimat_settings_load_on_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_LOAD_OFF, 0},  			MIB_INT(snmp_klimat_settings_load_off),		snmp_klimat_settings_load_off_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_BATT_ON, 0},  			MIB_INT(snmp_klimat_settings_batt_on),  	snmp_klimat_settings_batt_on_write},	//Номер БПСа
	{ MIB_INTEGER,			  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_KLIMAT, DISPLAY_KLIMAT_SETTINGS_BATT_OFF, 0},  			MIB_INT(snmp_klimat_settings_batt_off),  	snmp_klimat_settings_batt_off_write},	//Номер БПСа

	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSSOUNDALARMEN, 0},				MIB_INT(snmp_zv_en),  			snmp_zv_on_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMSALARMAUTODISABLE, 0},				MIB_INT(snmp_alarm_auto_disable),	snmp_alarm_auto_disable_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_TEST_TIME, 0},				MIB_INT(snmp_bat_test_time),		snmp_bat_test_time_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MAX, 0},						MIB_INT(snmp_u_max),			snmp_u_max_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN, 0},						MIB_INT(snmp_u_min),			snmp_u_min_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_0_GRAD, 0},					MIB_INT(snmp_u_0_grad),			snmp_u_0_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_20_GRAD, 0},					MIB_INT(snmp_u_20_grad),			snmp_u_20_grad_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_SIGN, 0},					MIB_INT(snmp_u_sign),			snmp_u_sign_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_MIN_POWER, 0},				MIB_INT(snmp_u_min_power),		snmp_u_min_power_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_U_WITHOUT_BAT, 0},				MIB_INT(snmp_u_withouth_bat),		snmp_u_withouth_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IBK, 0},						MIB_INT(snmp_control_current),	snmp_control_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IZMAX, 0},						MIB_INT(snmp_max_charge_current),	snmp_max_charge_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMAX, 0},						MIB_INT(snmp_max_current),		snmp_max_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_IMIN, 0},						MIB_INT(snmp_min_current),		snmp_min_current_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_UVZ, 0},						MIB_INT(snmp_uvz),				snmp_uvz_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TZAS, 0},						MIB_INT(snmp_powerup_psu_timeout),	snmp_powerup_psu_timeout_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BAT, 0},					MIB_INT(snmp_tsign_bat),			snmp_tsign_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BAT, 0},					MIB_INT(snmp_tmax_bat),			snmp_tmax_bat_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TSIGN_BPS, 0},					MIB_INT(snmp_tsign_bps),			snmp_tsign_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_TMAX_BPS, 0},					MIB_INT(snmp_tmax_bps),			snmp_tmax_bps_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_BAT_PART_ALARM, 0},				MIB_INT(snmp_bat_part_alarm),		snmp_bat_part_alarm_write},		//номер первого бпса 
	{ MIB_INTEGER,  			12, {OID_ENTERPRISE, OID_DEVICE, SYSPARAMS, SYSPARAMS_POWER_CNT_ADRESS, 0},			MIB_INT(snmp_power_cnt_adress),	snmp_power_cnt_adress_write},		//номер первого бпса 

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 1},  			MIB_INT(snmp_sk_number[0]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 2},  			MIB_INT(snmp_sk_number[1]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 3},  			MIB_INT(snmp_sk_number[2]),  	NULL},	//Номер БПСа
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ENTRY_NUMBER, 4},  			MIB_INT(snmp_sk_number[3]),  	NULL},	//Номер БПСа

//  	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_NAME, 1},  					MIB_STR(&snmp_sk_name[0][0]),  	NULL},	//местоположение устройства
//  	{ MIB_OCTET_STR | MIB_ATR_RO, 13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_NAME, 2},  					MIB_STR(&snmp_sk_name[1][0]),  	NULL},	//местоположение устройства
//  	{ MIB_OCTET_STR | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_NAME, 3},  					MIB_STR(&snmp_sk_name[2][0]),  	NULL},	//местоположение устройства
 // 	{ MIB_OCTET_STR | MIB_ATR_RO,	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_NAME, 4},  					MIB_STR(&snmp_sk_name[3][0]),  	NULL},	//местоположение устройства

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 1},  				MIB_INT(snmp_sk_aktiv[0]),  	NULL},	//Напряжение БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 2},  				MIB_INT(snmp_sk_aktiv[1]),  	NULL},	//Напряжение БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 3},  				MIB_INT(snmp_sk_aktiv[2]),  	NULL},	//Напряжение БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_AKTIVITY, 4},  				MIB_INT(snmp_sk_aktiv[3]),  	NULL},	//Напряжение БПС3

//	{ MIB_INTEGER ,  		  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 1},  			MIB_INT(snmp_sk_alarm_aktiv[0]),  	snmp_alarm_aktiv_write1},	//Ток БПС1
//	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 2},  			MIB_INT(snmp_sk_alarm_aktiv[1]),  	snmp_alarm_aktiv_write2},	//Ток БПС2
//	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 3},  			MIB_INT(snmp_sk_alarm_aktiv[2]),  	snmp_alarm_aktiv_write3},	//Ток БПС3
//	{ MIB_INTEGER ,			13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM_AKTIVITY, 4},  			MIB_INT(snmp_sk_alarm_aktiv[3]),  	snmp_alarm_aktiv_write4},	//Ток БПС3

//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 1},  					MIB_INT(snmp_sk_alarm[0]),  NULL},	//Ток БПС1
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 2},  					MIB_INT(snmp_sk_alarm[1]),  NULL},	//Ток БПС2
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 3},  					MIB_INT(snmp_sk_alarm[2]),  NULL},	//Ток БПС3
//	{ MIB_INTEGER | MIB_ATR_RO,  	13, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_SK, DISPLAY_SK_ALARM, 4},  					MIB_INT(snmp_sk_alarm[3]),  NULL},	//Ток БПС3


	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_EXT, 0},  		MIB_INT(snmp_dt_ext),  	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_MSAN, 0},  		MIB_INT(snmp_dt_msan),  	NULL},	
	{ MIB_INTEGER | MIB_ATR_RO,  	12, {OID_ENTERPRISE, OID_DEVICE, DISPLAY_DT, DISPLAY_DT_EPU, 0},  		MIB_INT(snmp_dt_epu),  	NULL},	



//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 1},    MIB_INT(LPC_RTC->HOUR),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 1, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},				  
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 1},    MIB_INT(LPC_RTC->MIN),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 2, 2},    MIB_INT(LPC_RTC->YEAR),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 1},     MIB_INT(LPC_RTC->SEC),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 3, 2},    MIB_INT(LPC_RTC->MONTH),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 4, 2},    MIB_INT(LPC_RTC->HOUR),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 1},     MIB_INT(sysMainsVoltage),    NULL},	    //-----------------------------------------------
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 5, 2},    MIB_INT(sysServices),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 6, 2},    MIB_INT(sysServices),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 7, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 8, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 9, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 10, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 1},    MIB_INT(sysMainsVoltage),     NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 11, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 1},     MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 12, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 1},    MIB_INT(sysMainsVoltage),    NULL},
//	{ MIB_INTEGER, 	13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 4, 2, 1, 13, 2},    MIB_INT(TestForTableValues),    NULL},
//	{ MIB_OCTET_STR, 13, {OID0(1,3), 6, 1, 4, 1, 130, 131, 31, 1, 2, 7, 4, 0},  MIB_STR("Proverka sviazi. Проверка связи."),   NULL},

	};
#endif
const int snmp_mib_size = (sizeof(snmp_mib) / sizeof(MIB_ENTRY));

///*----------------------------------------------------------------------------
// *      MIB Callback Functions
// *---------------------------------------------------------------------------*/
//
//static void write_leds (int mode) {
//  /* No action on read access. */
//  if (mode == MIB_WRITE) {
//    LED_out (LedOut);
//  }
//}
//
//static void read_key (int mode) {
//  /* Read ARM Digital Input */
//  if (mode == MIB_READ) {
//    KeyIn = get_button();
//  }
//}
//
//static void upd_display (int mode) {
//  /* Update LCD Module display text. */
//  if (mode == MIB_WRITE) {
//    /* Write access. */
//    LCDupdate = __TRUE;
//  }
//}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
