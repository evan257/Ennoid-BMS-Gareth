/*
	Copyright 2017 - 2018 Danny Bokma	danny@diebie.nl
	Copyright 2019 - 2020 Kevin Dionne	kevin.dionne@ennoid.me

	This file is part of the DieBieMS/ENNOID-BMS firmware.

	The DieBieMS/ENNOID-BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The DieBieMS/ENNOID-BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MODPOWERELECTRONICS_H
#define __MODPOWERELECTRONICS_H

#include "driverSWISL28022.h"
#include "driverHWADC.h"
#include "driverSWLTC6804.h"
#include "driverSWADC128D818.h"
#include "driverHWSwitches.h"
#include "driverSWEMC2305.h"
#include "modDelay.h"
#include "modConfig.h"
#include "modPowerState.h"
#include "stdbool.h"
#include "math.h"
#include "driverSWSHT21.h"
#include "driverSWHTC1080.h"


#define NoOfCellsPossibleOnBMS	      	180
#define NoOfCellMonitorsPossibleOnBMS 	10

#define NoOfAuxPossibleOnBMS	      		120

#define NoOfExpPossibleOnBMS	      		72
#define NoOfExpMonitorPossibleOnBMS	    9

#define NoOfTempSensors               	14
#define VinErrorThreshold             	10

typedef enum {
	TEMP_EXT_LTC_NTC0 = 0,									// EXT on master BMS on LTC
	TEMP_EXT_LTC_NTC1,											// EXT on master BMS on LTC
	TEMP_INT_LTC_CHIP,											// Int on master BMS inside LTC Chip
	TEMP_INT_STM_NTC,												// Int on master BMS outside STM Chip
	TEMP_EXT_ADC_NTC0,											// Ext on slave BMS OR water detect
	TEMP_EXT_ADC_NTC1,											// Ext on slave BMS OR water detect
	TEMP_EXT_ADC_NTC2,											// Ext on slave BMS OR water detect
	TEMP_EXT_ADC_NTC3,											// Ext on slave BMS OR water detect
	TEMP_EXT_ADC_NTC4,											// Ext on slave BMS OR water detect
	TEMP_EXT_ADC_NTC5,											// Ext on slave BMS OR water detect
	TEMP_INT_ADC_NTC6,											// Int on slave BMS
	TEMP_INT_ADC_NTC7,											// Int on slave BMS
	TEMP_INT_SHT,														// Int on slave BMS
	TEMP_INT_ADC_NTCAUX                     // Ext on the CAN connector
} modPowerElectronicsTemperatureSensorMapping;

typedef enum {
	PACK_STATE_ERROR_HARD_CELLVOLTAGE = 0,
	PACK_STATE_ERROR_SOFT_CELLVOLTAGE,
	PACK_STATE_ERROR_OVER_CURRENT,
	PACK_STATE_NORMAL,
	PACK_STATE_ERROR_TEMPERATURE,
} modPowerElectronicsPackOperationalCellStatesTypedef;

typedef struct {
	// Master BMS
	uint16_t throttleDutyGeneralTemperatureBMS;
	uint16_t throttleDutyChargeVoltage;
	uint16_t throttleDutyChargeTemperatureBattery;
	uint16_t throttleDutyCharge;
	uint16_t throttleDutyDischargeVoltage;
	uint16_t throttleDutyDischargeTemperatureBattery;
	uint16_t throttleDutyDischarge;
	float    SoC;
	float    SoCCapacityAh;
	OperationalStateTypedef operationalState;
	bms_fault_state faultState;
	float    packVoltage;
	float    packCurrent;
	float    packPower;
	float    loCurrentLoadCurrent;
	float    loCurrentLoadVoltage;
	float    chargerVoltage;
	float    cellVoltageHigh;
	float    cellVoltageLow;
	float    cellVoltageAverage;
	float    cellVoltageMisMatch;
	uint16_t cellBalanceResistorEnableMask;
	float    temperatures[NoOfTempSensors];
	float    tempBatteryHigh;
	float    tempBatteryLow;
	float    tempBatteryAverage;
	float    tempBMSHigh;
	float    tempBMSLow;
	float    tempBMSAverage;
	float 	 humidity;
	uint8_t  preChargeDesired;
	bool   stateError;
	uint8_t  disChargeDesired;
	uint8_t  disChargeLCAllowed;
	uint8_t  chargeDesired;
	uint8_t  chargePFETDesired;
	uint8_t  chargeAllowed;
	uint8_t  coolingDesired;
	uint8_t  coolingAllowed;
	uint8_t  safetyOverCANHCSafeNSafe;
	uint8_t  chargeCurrentDetected;
	uint8_t  chargeBalanceActive;
	uint8_t  balanceActive;
	uint8_t  powerButtonActuated;
	uint8_t  packInSOADischarge;
  uint8_t  packInSOACharge;
	uint8_t  powerDownDesired;
	uint8_t  powerOnLongButtonPress;
	uint8_t  buzzerOn;
	uint8_t  watchDogTime;
	cellMonitorCellsTypeDef cellVoltagesIndividual[NoOfCellsPossibleOnBMS];
	auxMonitorTypeDef auxVoltagesIndividual[NoOfAuxPossibleOnBMS];
	expMonitorTypeDef expVoltagesIndividual[NoOfExpPossibleOnBMS];
	
	modPowerElectronicsPackOperationalCellStatesTypedef packOperationalCellState;
	
	// Slave modules -> TODO move into struct.
	float    cellModuleVoltages[NoOfCellMonitorsPossibleOnBMS][18];
	float    auxModuleVoltages[NoOfCellMonitorsPossibleOnBMS][12];
	float    expModuleVoltages[NoOfExpMonitorPossibleOnBMS][8];
	uint32_t cellModuleBalanceResistorEnableMask[NoOfCellMonitorsPossibleOnBMS];
	uint32_t cellModuleBalanceResistorEnableMaskTest[NoOfCellMonitorsPossibleOnBMS];
	uint32_t dieTemperature[NoOfCellMonitorsPossibleOnBMS];
	
} modPowerElectronicsPackStateTypedef;

void  modPowerElectronicsInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfig);
bool  modPowerElectronicsTask(void);
void  modPowerElectronicsAllowForcedOn(bool allowedState);
void  modPowerElectronicsSetPreCharge(bool newState);
bool  modPowerElectronicsSetDisCharge(bool newState);
void  modPowerElectronicsSetCharge(bool newState);
void  modPowerElectronicsSetChargePFET(bool newState);
void  modPowerElectronicsSetCooling(bool newState);
void  modPowerElectronicsDisableAll(void);
void  modPowerElectronicsCalculateCellStats(void);
void  modPowerElectronicsSubTaskBalancing(void);
void  modPowerElectronicsCallMonitorsCalcBalanceResistorArray(void);
void  modPowerElectronicsSubTaskVoltageWatch(void);
void 	modPowerElectronicsSubTaskCurrentWatch(void);
void  modPowerElectronicsUpdateSwitches(void);
void  modPowerElectronicsSortCells(cellMonitorCellsTypeDef *cells, uint8_t cellCount);
void  modPowerElectronicsCalcTempStats(void);
void  modPowerElectronicsCalcThrottle(void);
int32_t modPowerElectronicsMapVariableInt(int32_t inputVariable, int32_t inputLowerLimit, int32_t inputUpperLimit, int32_t outputLowerLimit, int32_t outputUpperLimit);
float modPowerElectronicsMapVariableFloat(float inputVariable, float inputLowerLimit, float inputUpperLimit, float outputLowerLimit, float outputUpperLimit);
void  modPowerElectronicsInitISL(void);
void  modPowerElectronicsSubTaskBuzzer(void);
bool  modPowerElectronicsHCSafetyCANAndPowerButtonCheck(void);
void  modPowerElectronicsResetBalanceModeActiveTimeout(void);
void  modPowerElectronicsCellMonitorsInit(void);
void  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData(void);
void  modPowerElectronicsCellMonitorsStartCellConversion(void);
void  modPowerElectronicsCellMonitorsStartLoadedCellConversion(bool PUP);
void  modPowerElectronicsCellMonitorsStartTemperatureConversion(void);
void  modPowerElectronicsCellMonitorsEnableBalanceResistors(uint32_t);
void  modPowerElectronicsCellMonitorsEnableBalanceResistorsArrayTest(uint32_t);
void  modPowerElectronicsCellMonitorsEnableBalanceResistorsArray(void);
void  modPowerElectronicsCellMonitorsReadVoltageFlags(uint32_t *underVoltageFlags, uint32_t *overVoltageFlags);
void  modPowerElectronicsCellMonitorsCheckAndSolveInitState(void);
void  modPowerElectronicsCellMonitorsArrayTranslate(void);
void  modPowerElectronicsAuxMonitorsArrayTranslate(void);
void  modPowerElectronicsExpMonitorsArrayTranslate(void);
float modPowerElectronicsCalcPackCurrent(void);
void  modPowerElectronicsTerminalCellConnectionTest(int argc, const char **argv);
void  modPowerElectronicsCheckPackSOA(void);
void  modPowerElectronicsSamplePackAndLCData(void);
void  modPowerElectronicsSamplePackVoltage(float *voltagePointer);
void  modPowerElectronicsLCSenseSample(void);
void  modPowerElectronicsLCSenseInit(void);
uint16_t modPowerElectronicsLowestInThree(uint16_t num1,uint16_t num2,uint16_t num3);
void  modPowerElectronicsResetCurrentOffset(void);
float getIndividualCell(uint8_t cellNum);

#endif
