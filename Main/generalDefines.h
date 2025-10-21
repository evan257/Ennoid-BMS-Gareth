// Define the hardware version here:
#ifndef ENNOID_SS_LITE
#define ENNOID_SS_LITE					0
#endif

#ifndef ENNOID_XLITE
#define ENNOID_XLITE					0
#endif

#ifndef ENNOID_SS
#define ENNOID_SS					0
#endif

#ifndef ENNOID_LV
#define ENNOID_LV					0
#endif

#ifndef ENNOID_HV
#define ENNOID_HV					1
#endif

#ifndef ENNOID_LV_2
#define ENNOID_LV_2					0
#endif

#ifndef ENNOID_SS_24
#define ENNOID_SS_24					0
#endif

// Firmware version
#define FW_VERSION_MAJOR				5
#define FW_VERSION_MINOR				2
#define FW_REAL_VERSION					"V5.2"
#define FW_TEST_VERSION_NUMBER				0
// UUID
#define STM32_UUID				((uint32_t*)0x1FFFF7AC)
#define STM32_UUID_8				((uint8_t*)0x1FFFF7AC)

// Hardware names and limits
#if ENNOID_LV
  #define HW_NAME "ENNOID-LV"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   18
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000
	#define HAS_ON_BOARD_NTC				1					
	#define ISL28022_MASTER_ADDRES				0x40
	#define HAS_DISCHARGE					1
	#define HAS_COMMON_CHARGE_DISCHARGE_OPTION		1
#endif

// Hardware names and limits
#if ENNOID_LV_2
  #define HW_NAME "ENNOID-LV-GEN2"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   18
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000
	#define ISL28022_MASTER_ADDRES				0x45
	#define HAS_DISCHARGE					1
	#define HAS_HUMIDITY					1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			1
	#define HAS_COMMON_CHARGE_DISCHARGE_OPTION		1
#endif

#if ENNOID_HV
  #define HW_NAME "ENNOID-HV"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   18
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000
	#define HAS_ON_BOARD_NTC				1					
	#define ISL28022_MASTER_ADDRES				0x40
	#define HAS_DISCHARGE					1
	#define HAS_COMMON_CHARGE_DISCHARGE_OPTION		1
#endif

#if ENNOID_SS
  #define HW_NAME "ENNOID-SS"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   1
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000

	#define HAS_PFET_OUTPUT					1
	#define ISL28022_MASTER_ADDRES				0x45
	#define HAS_DISCHARGE					1
	#define HAS_HUMIDITY					1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			1
#endif

#if ENNOID_SS_24
  #define HW_NAME "ENNOID-SS-24"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   2
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000

	#define HAS_PFET_OUTPUT					1
	#define ISL28022_MASTER_ADDRES				0x45
	#define HAS_DISCHARGE					1
	#define HAS_HUMIDITY					1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			1
#endif

#if ENNOID_SS_LITE
  #define HW_NAME "ENNOID-SS-LITE"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   1
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000

	#define HAS_PFET_OUTPUT					1
	#define ISL28022_MASTER_ADDRES				0x45
	#define HAS_HUMIDITY					1
	#define HAS_NO_DISCHARGE				1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			1
#endif

#if ENNOID_XLITE
  #define HW_NAME "ENNOID-XLITE"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE 24
	#define HW_LIM_CELL_MONITOR_IC_COUNT                   2
	#define HW_LIM_MIN_NOT_USED_DELAY                      5000
	#define HAS_PFET_OUTPUT					1
	#define ISL28022_MASTER_ADDRES				0x45
	#define HAS_NO_DISCHARGE				1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			1
#endif

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
