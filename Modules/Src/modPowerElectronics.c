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

#include "modPowerElectronics.h"
#include "modTerminal.h"

modPowerElectronicsPackStateTypedef *modPowerElectronicsPackStateHandle;
modConfigGeneralConfigStructTypedef *modPowerElectronicsGeneralConfigHandle;

uint32_t modPowerElectronicsCANCurrentVESCLastTick;

uint32_t modPowerElectronicsMeasureIntervalLastTick;

uint32_t modPowerElectronicsChargeRetryLastTick;
uint32_t modPowerElectronicsDisChargeLCRetryLastTick;
bool     modPowerElectronicsFirstDischarge;

uint32_t modPowerElectronicsCellBalanceUpdateLastTick;
uint32_t modPowerElectronicsTempMeasureDelayLastTick;
uint32_t modPowerElectronicsChargeCurrentDetectionLastTick;
uint32_t modPowerElectronicsBalanceModeActiveLastTick;

uint32_t modPowerElectronicsBuzzerUpdateIntervalLastTick;
uint32_t modPowerElectronicsThrottleChargeLastTick;
uint8_t  modPowerElectronicsUnderAndOverVoltageErrorCount;
uint8_t  modPowerElectronicsUnderAndOverTemperatureErrorCount;
bool     modPowerElectronicsAllowForcedOnState;
uint16_t modPowerElectronicsAuxVoltageArray[3];
uint16_t tempTemperature;
uint8_t  modPowerElectronicsVinErrorCount;
configCellMonitorICTypeEnum modPowerElectronicsCellMonitorsTypeActive;
float    modPowerElectronicsChargeDiodeBypassHysteresis;
bool     modPowerElectronicsVoltageSenseError;
bool 	balanceAllowed = 0;
bool    balanceAllowedECU = 0;

bool     modPowerElectronicsChargeDeratingActive;
uint32_t modPowerElectronicsChargeIncreaseLastTick;
uint32_t modPowerElectronicsSOAChargeChangeLastTick;
uint32_t modPowerElectronicsSOADisChargeChangeLastTick;
uint32_t chargeIncreaseIntervalTime;

uint16_t  calculatedChargeThrottle = 0;
float currentOffset = 0.0f;
//float currentOffsetTemp = 0.0f;
uint8_t currentOffsetCounter = 0;

//uint32_t hardUnderVoltageFlags, hardOverVoltageFlags;

void modPowerElectronicsInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer) {
	modPowerElectronicsGeneralConfigHandle						= generalConfigPointer;
	modPowerElectronicsPackStateHandle						= packState;
	modPowerElectronicsUnderAndOverVoltageErrorCount				= 0;
	modPowerElectronicsUnderAndOverTemperatureErrorCount				= 0;
	modPowerElectronicsAllowForcedOnState						= false;
	modPowerElectronicsVinErrorCount						= 0;
	modPowerElectronicsChargeDiodeBypassHysteresis					= 0.0f;
	modPowerElectronicsVoltageSenseError						= false;
	modPowerElectronicsChargeDeratingActive						= false;
	modPowerElectronicsFirstDischarge						= true;
	
	// Init pack status
	modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS		= 0;
	modPowerElectronicsPackStateHandle->throttleDutyChargeVoltage			= 0;
	modPowerElectronicsPackStateHandle->throttleDutyChargeTemperatureBattery	= 0;
	modPowerElectronicsPackStateHandle->throttleDutyCharge				= 0;
	modPowerElectronicsPackStateHandle->throttleDutyDischargeVoltage		= 0;
	modPowerElectronicsPackStateHandle->throttleDutyDischargeTemperatureBattery	= 0;
	modPowerElectronicsPackStateHandle->throttleDutyDischarge			= 0;
	modPowerElectronicsPackStateHandle->SoC						= 0.0f;
	modPowerElectronicsPackStateHandle->SoCCapacityAh				= 0.0f;
	modPowerElectronicsPackStateHandle->operationalState				= OP_STATE_INIT;
	modPowerElectronicsPackStateHandle->faultState					= FAULT_CODE_NONE;
	modPowerElectronicsPackStateHandle->packVoltage					= 0.0f;
	modPowerElectronicsPackStateHandle->packCurrent					= 0.0f;
	modPowerElectronicsPackStateHandle->packPower					= 0.0f;
	modPowerElectronicsPackStateHandle->loCurrentLoadCurrent			= 0.0f;
	modPowerElectronicsPackStateHandle->loCurrentLoadVoltage			= 0.0f;
	modPowerElectronicsPackStateHandle->chargerVoltage				= 0.0f;
	modPowerElectronicsPackStateHandle->cellVoltageHigh				= 0.0f;
	modPowerElectronicsPackStateHandle->cellVoltageLow				= 0.0f;
	modPowerElectronicsPackStateHandle->cellVoltageAverage				= 0.0;
	modPowerElectronicsPackStateHandle->disChargeDesired				= false;
	modPowerElectronicsPackStateHandle->disChargeLCAllowed				= true;
	modPowerElectronicsPackStateHandle->preChargeDesired				= false;
	modPowerElectronicsPackStateHandle->chargeDesired				= false;
	modPowerElectronicsPackStateHandle->chargePFETDesired				= false;
	modPowerElectronicsPackStateHandle->chargeAllowed				= true;
	modPowerElectronicsPackStateHandle->coolingDesired				= false;
	modPowerElectronicsPackStateHandle->coolingAllowed				= true;
	modPowerElectronicsPackStateHandle->safetyOverCANHCSafeNSafe			= false;
	modPowerElectronicsPackStateHandle->chargeBalanceActive				= false;
	modPowerElectronicsPackStateHandle->balanceActive				= false;
	modPowerElectronicsPackStateHandle->chargeCurrentDetected			= false;
	modPowerElectronicsPackStateHandle->powerButtonActuated				= false;
	modPowerElectronicsPackStateHandle->packInSOACharge				= true;
	modPowerElectronicsPackStateHandle->packInSOADischarge				= true;	
	modPowerElectronicsPackStateHandle->watchDogTime				= 255;
	modPowerElectronicsPackStateHandle->packOperationalCellState			= PACK_STATE_NORMAL;
	modPowerElectronicsPackStateHandle->temperatures[0]				= -50.0f;
	modPowerElectronicsPackStateHandle->temperatures[1]				= -50.0f;
	modPowerElectronicsPackStateHandle->temperatures[2]				= -50.0f;
	modPowerElectronicsPackStateHandle->temperatures[3]				= -50.0f;
	modPowerElectronicsPackStateHandle->tempBatteryHigh				= 0.0f;
	modPowerElectronicsPackStateHandle->tempBatteryLow				= 0.0f;
	modPowerElectronicsPackStateHandle->tempBatteryAverage				= 0.0f;
	modPowerElectronicsPackStateHandle->tempBMSHigh					= 0.0f;
	modPowerElectronicsPackStateHandle->tempBMSLow					= 0.0f;
	modPowerElectronicsPackStateHandle->tempBMSAverage				= 0.0f;
	modPowerElectronicsPackStateHandle->humidity					= 0.0f;
	modPowerElectronicsPackStateHandle->buzzerOn					= false;
	modPowerElectronicsPackStateHandle->powerDownDesired				= false;
	modPowerElectronicsPackStateHandle->powerOnLongButtonPress			= false;
	
	// init the cell module variables empty
	for( uint8_t modulePointer = 0; modulePointer < NoOfCellMonitorsPossibleOnBMS; modulePointer++) {
		for(uint8_t cellPointer = 0; cellPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule; cellPointer++)
			modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][cellPointer] = 0.0f;
		
		modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMask[modulePointer] = 0;
		modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMaskTest[modulePointer] = 0;
	}
	// init the aux module variables empty
	for( uint8_t modulePointer = 0; modulePointer < NoOfCellMonitorsPossibleOnBMS; modulePointer++) {
		for(uint8_t auxPointer = 0; auxPointer < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerModule; auxPointer++)
			modPowerElectronicsPackStateHandle->auxModuleVoltages[modulePointer][auxPointer] = 0.0f;
	}
	// init the exp module variables empty
	for( uint8_t modulePointer = 0; modulePointer < modPowerElectronicsGeneralConfigHandle->noOfExpansionBoard; modulePointer++) {
		for(uint8_t expPointer = 0; expPointer < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerExpansionBoard; expPointer++)
			modPowerElectronicsPackStateHandle->expModuleVoltages[modulePointer][expPointer] = 0.0f;
	}
	// init cell active table
	for(uint8_t cellPointer = 1; cellPointer <= modPowerElectronicsGeneralConfigHandle->noOfCellsSeries; cellPointer++){
		if(cellPointer == 6 || cellPointer == 17 || cellPointer == 22 || cellPointer == 28 || cellPointer == 39 || cellPointer == 44 || cellPointer == 50 || cellPointer == 61 || cellPointer == 66 || cellPointer == 72 || cellPointer == 83 || cellPointer == 88 || cellPointer == 94 || cellPointer == 105 || cellPointer == 110){
			modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer-1].isActive = true; // if you want to skip over cells change this back to false and enter the cells you want to skip over above
		} else {
			modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer-1].isActive = true;
		}
	}
	
	modPowerElectronicsCellMonitorsStartCellConversion();

	// Init the external bus monitor
  modPowerElectronicsInitISL();
	
	#if (HAS_HUMIDITY)
		if(modPowerElectronicsGeneralConfigHandle->humidityICType == si7020)
			driverSWSHT21Init();
		else
			driverSWHTC1080Init();
	#endif
	
	// Init internal ADC
	driverHWADCInit();
	driverHWSwitchesInit();
	
	// Init battery stack monitor
	modPowerElectronicsCellMonitorsInit();
	
	modPowerElectronicsChargeCurrentDetectionLastTick = HAL_GetTick();
	modPowerElectronicsBalanceModeActiveLastTick      = HAL_GetTick();
	modPowerElectronicsSOAChargeChangeLastTick        = HAL_GetTick();
	modPowerElectronicsSOADisChargeChangeLastTick     = HAL_GetTick();	
	
	// Sample the first pack voltage moment
	driverSWISL28022GetBusVoltage(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,&modPowerElectronicsPackStateHandle->packVoltage,modPowerElectronicsGeneralConfigHandle->voltageLCOffset, modPowerElectronicsGeneralConfigHandle->voltageLCFactor);

	
	// Register terminal commands
	modTerminalRegisterCommandCallBack("testbms","Test the cell connection between cell monitor and pack and pack vs cell measurement.","[error (V)] [bal drop (mV)]",modPowerElectronicsTerminalCellConnectionTest);

};

bool modPowerElectronicsTask(void) {
	bool returnValue = false;
	// Static variable for SHT measure timer

	
	if(modDelayTick1ms(&modPowerElectronicsMeasureIntervalLastTick,100)) {
		// reset tick for LTC Temp start conversion delay
		modPowerElectronicsTempMeasureDelayLastTick = HAL_GetTick();
		
		// Collect low current current path, pack data and check validity + recover if invalid.
		modPowerElectronicsSamplePackAndLCData();
		
		// Check whether packvoltage is whithin theoretical limits
		if(modPowerElectronicsPackStateHandle->packVoltage >= (modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->cellHardOverVoltage)) {
			modPowerElectronicsVoltageSenseError = true;
		}
		
		// Combine the currents based on config and calculate pack power.
		//modPowerElectronicsPackStateHandle->packCurrent = modPowerElectronicsCalcPackCurrent();
		modPowerElectronicsPackStateHandle->packPower   = modPowerElectronicsPackStateHandle->packCurrent * modPowerElectronicsPackStateHandle->packVoltage;
		driverHWADCGetPackCurrent(&modPowerElectronicsPackStateHandle->packCurrent); //This directly sets pack current using the hall effect sensor. If another method is desired this will need to be updated.
    // Read the battery cell voltages and temperatures with the cell monitor ICs
		modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();
		
		// get PCB mounted temperature sensor & humitity if applicable
		#if (HAS_HUMIDITY)
			if(modPowerElectronicsGeneralConfigHandle->humidityICType == si7020){
				static uint32_t measureSHTStartLastTick          = 0;
				static driverSWSHT21MeasureType lastMeasuredType = TEMP;
				if(driverSWSHT21PollMeasureReady()){
					modPowerElectronicsPackStateHandle->temperatures[0] = driverSWSHT21GetTemperature();
					modPowerElectronicsPackStateHandle->humidity        = driverSWSHT21GetHumidity();
				}
				if(modDelayTick1ms(&measureSHTStartLastTick,500)){
					driverSWSHT21StartMeasurement(lastMeasuredType);
		
				if(lastMeasuredType == TEMP)																																							// Toggle between SHT21 sensor modes
					lastMeasuredType = HUMIDITY;
				else
					lastMeasuredType = TEMP;
				}
			}else{
				static uint32_t measureHTC1080StartLastTick          = 0;
				if(driverSWHTC1080PollMeasureReady()){
					modPowerElectronicsPackStateHandle->temperatures[0] = driverSWHTC1080GetTemperature();
					modPowerElectronicsPackStateHandle->humidity        = driverSWHTC1080GetHumidity();
				}
				if(modDelayTick1ms(&measureHTC1080StartLastTick,500)){
					driverSWHTC1080StartMeasurement();
				}
			};
		#endif

		#if(HAS_ON_BOARD_NTC)
			driverHWADCGetNTCValue(&modPowerElectronicsPackStateHandle->temperatures[0],modPowerElectronicsGeneralConfigHandle->NTC25DegResistance[modConfigNTCGroupMasterPCB],modPowerElectronicsGeneralConfigHandle->NTCTopResistor[modConfigNTCGroupMasterPCB],modPowerElectronicsGeneralConfigHandle->NTCBetaFactor[modConfigNTCGroupMasterPCB],25.0f);
		#endif
		
		// When temperature and cellvoltages are known calculate charge and discharge throttle.
		modPowerElectronicsCalcThrottle();
		
		// Do the balancing task
		modPowerElectronicsSubTaskBalancing();
		
		// Measure cell voltages
		modPowerElectronicsCellMonitorsStartCellConversion();
		
		//Measure temperature voltages
		modPowerElectronicsCellMonitorsStartTemperatureConversion();
		
		// Calculate temperature statisticks
		modPowerElectronicsCalcTempStats();
		
		// Check and respond to the measured voltage values
		modPowerElectronicsSubTaskVoltageWatch();
		
		// Check and respond to the measured current values
		modPowerElectronicsSubTaskCurrentWatch();
		
		
		// Check and respond to the measured temperature values
		modPowerElectronicsCheckPackSOA();
		
		modPowerElectronicsPackStateHandle->powerButtonActuated = modPowerStateGetButtonPressedState();
		
		returnValue = true;
	}else
		returnValue = false;
	
	return returnValue;
};

void modPowerElectronicsAllowForcedOn(bool allowedState){
	modPowerElectronicsAllowForcedOnState = allowedState;
}

void modPowerElectronicsSetPreCharge(bool newState) {
	static bool preChargeLastState = false;
	
	if(preChargeLastState != newState) {
		preChargeLastState = newState;

		if(modPowerElectronicsGeneralConfigHandle->LCUsePrecharge>=1){
			modPowerElectronicsPackStateHandle->preChargeDesired = newState;
		}
			
		else
			modPowerElectronicsPackStateHandle->preChargeDesired = false;

		
		
		modPowerElectronicsUpdateSwitches();
		
	}
};


bool modPowerElectronicsSetDisCharge(bool newState) {
	static bool dischargeLastState = false;
	
	if(dischargeLastState != newState) {
		dischargeLastState = newState;	

		if(modPowerElectronicsGeneralConfigHandle->LCUseDischarge==1)
			modPowerElectronicsPackStateHandle->disChargeDesired = newState;
		else
			modPowerElectronicsPackStateHandle->disChargeDesired = false;
		
		
		modPowerElectronicsUpdateSwitches();
		
	}
	
	// Prevent turn on with too low output voltage
	if((modPowerElectronicsPackStateHandle->loCurrentLoadVoltage < modPowerElectronicsGeneralConfigHandle->minimalPrechargePercentage*(modPowerElectronicsPackStateHandle->packVoltage)) && modPowerElectronicsGeneralConfigHandle->LCUsePrecharge>=1){ 
		modPowerElectronicsDisChargeLCRetryLastTick = HAL_GetTick();

		if(modPowerElectronicsFirstDischarge == false){
			modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
		}
		return false;	// Load voltage too low (output not precharged enough) return whether or not precharge is needed.
	}else{
		modPowerElectronicsFirstDischarge = false;
		return true;
	}
};

void modPowerElectronicsSetCharge(bool newState) {
	static bool chargeLastState = false;
	
	if(chargeLastState != newState) {
		chargeLastState = newState;
		modPowerElectronicsPackStateHandle->chargeDesired = newState;
		modPowerElectronicsUpdateSwitches();
	}
};

void modPowerElectronicsSetChargePFET(bool newState) {
	static bool chargePFETLastState = false;
	
	if(chargePFETLastState != newState) {
		chargePFETLastState = newState;
		modPowerElectronicsPackStateHandle->chargePFETDesired = newState;
		modPowerElectronicsUpdateSwitches();
	}
};

void modPowerElectronicsSetCooling(bool newState) {
	static bool coolingLastState = false;
	
	if(coolingLastState != newState) {
		coolingLastState = newState;
		modPowerElectronicsPackStateHandle->coolingDesired = newState;
		modPowerElectronicsUpdateSwitches();
	}
};

void modPowerElectronicsDisableAll(void) {
	if(modPowerElectronicsPackStateHandle->disChargeDesired | modPowerElectronicsPackStateHandle->preChargeDesired | modPowerElectronicsPackStateHandle->chargeDesired) {
		modPowerElectronicsPackStateHandle->disChargeDesired = false;
		modPowerElectronicsPackStateHandle->preChargeDesired = false;
		modPowerElectronicsPackStateHandle->chargeDesired = false;
		modPowerElectronicsPackStateHandle->chargePFETDesired = false;
		modPowerElectronicsPackStateHandle->coolingDesired = false;
		driverHWSwitchesDisableAll();
	}
};

void modPowerElectronicsCalculateCellStats(void) {
	float cellVoltagesSummed = 0.0f;
	modPowerElectronicsPackStateHandle->cellVoltageHigh = 0.0f;
	modPowerElectronicsPackStateHandle->cellVoltageLow = 10.0f;
	
	for(uint8_t cellPointer = 0; cellPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->noOfParallelModules; cellPointer++) {
		cellVoltagesSummed += modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellVoltage * modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].isActive;
		
		if(modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellVoltage > modPowerElectronicsPackStateHandle->cellVoltageHigh && modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].isActive)
			modPowerElectronicsPackStateHandle->cellVoltageHigh = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellVoltage;
		
		if(modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellVoltage < modPowerElectronicsPackStateHandle->cellVoltageLow && modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].isActive)
			modPowerElectronicsPackStateHandle->cellVoltageLow = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellVoltage;		
	}
	
	modPowerElectronicsPackStateHandle->cellVoltageAverage = cellVoltagesSummed/(modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->noOfParallelModules);
	modPowerElectronicsPackStateHandle->cellVoltageMisMatch = modPowerElectronicsPackStateHandle->cellVoltageHigh - modPowerElectronicsPackStateHandle->cellVoltageLow;
};

void modPowerElectronicsSubTaskBalancing(void) { //Comp fix
	static uint32_t delayTimeHolder = 100;
	static bool     delaytoggle = false;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
		balanceAllowed = true;
	} else {
		balanceAllowed = false;
	}

	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
		balanceAllowedECU = true;
	} else {
		balanceAllowedECU = false;
	}
	
	
	if(modDelayTick1ms(&modPowerElectronicsCellBalanceUpdateLastTick,delayTimeHolder)) {
		delaytoggle ^= true;
		delayTimeHolder = delaytoggle ? modPowerElectronicsGeneralConfigHandle->cellBalanceUpdateInterval : 200;
		
		if(delaytoggle) {
			if(balanceAllowed == true && balanceAllowedECU == true) {	//comp fix
				for(uint8_t i = 0; i < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->noOfParallelModules; i++) {
					if(modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellVoltage >= (modPowerElectronicsPackStateHandle->cellVoltageLow + modPowerElectronicsGeneralConfigHandle->cellBalanceDifferenceThreshold) && modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].isActive) {
						if(modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellVoltage >= modPowerElectronicsGeneralConfigHandle->cellBalanceStart) {
							modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellBleedActive = true;
							modPowerElectronicsPackStateHandle->balanceActive = true;
						}else{
						  modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellBleedActive = false;
						}
					}else{
						modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellBleedActive = false;
					}
				}
			}
		}else{
		for(uint8_t i = 0; i < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->noOfParallelModules; i++) {
		modPowerElectronicsPackStateHandle->cellVoltagesIndividual[i].cellBleedActive = false;
			
		}
		modPowerElectronicsPackStateHandle->balanceActive = false;
	}
		
		modPowerElectronicsCallMonitorsCalcBalanceResistorArray();
		modPowerElectronicsCellMonitorsEnableBalanceResistorsArray();
	}
};

void modPowerElectronicsCallMonitorsCalcBalanceResistorArray(void) {
	uint8_t modulePointer = 0;
	uint8_t cellInMaskPointer = 0;
	uint8_t seriesCount = 0;
	uint8_t moduleCount = 0;
	
	// Clear array
	for(uint8_t moduleClearPointer = 0; moduleClearPointer < NoOfCellMonitorsPossibleOnBMS; moduleClearPointer++)
		modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMask[moduleClearPointer] = 0;
	
	
	for(uint8_t cellPointer = 0; cellPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->noOfParallelModules; cellPointer++) {
		seriesCount = cellPointer/modPowerElectronicsGeneralConfigHandle->noOfCellsSeries; 
		moduleCount = seriesCount*(modPowerElectronicsGeneralConfigHandle->cellMonitorICCount/modPowerElectronicsGeneralConfigHandle->noOfParallelModules);
		modulePointer = moduleCount + (cellPointer % modPowerElectronicsGeneralConfigHandle->noOfCellsSeries)/modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule;
		cellInMaskPointer = (cellPointer - (seriesCount*modPowerElectronicsGeneralConfigHandle->noOfCellsSeries)) % modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule;
		
		if(modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellPointer].cellBleedActive)
		  modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMask[modulePointer] |= (1 << cellInMaskPointer);
		else
			modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMask[modulePointer] &= ~(1 << cellInMaskPointer);
	}
}

void modPowerElectronicsSubTaskVoltageWatch(void) {
	static bool lastdisChargeLCAllowed = false;
	static bool lastChargeAllowed = false;
	
	//modPowerElectronicsCellMonitorsReadVoltageFlags(&hardUnderVoltageFlags,&hardOverVoltageFlags);
	modPowerElectronicsCalculateCellStats();
	
	if(modPowerElectronicsPackStateHandle->packOperationalCellState != PACK_STATE_ERROR_HARD_CELLVOLTAGE && modPowerElectronicsPackStateHandle->packOperationalCellState != PACK_STATE_ERROR_TEMPERATURE) {
		
		// Handle soft cell voltage limits & temperatures
		//Discharge disable
		if(modPowerElectronicsPackStateHandle->cellVoltageLow <= modPowerElectronicsGeneralConfigHandle->cellLCSoftUnderVoltage) {
			modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
			modPowerElectronicsDisChargeLCRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_CELL_SOFT_UNDER_VOLTAGE;
		}

		if(modPowerElectronicsPackStateHandle->tempBatteryHigh >= modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax){
			modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
			modPowerElectronicsDisChargeLCRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_DISCHARGE_OVER_TEMP_CELLS;
		}
		
		if(modPowerElectronicsPackStateHandle->tempBatteryLow <= modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMin){
			modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
			modPowerElectronicsDisChargeLCRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_DISCHARGE_UNDER_TEMP_CELLS;
		}
		//Charge disable
		if(modPowerElectronicsPackStateHandle->cellVoltageHigh >= modPowerElectronicsGeneralConfigHandle->cellSoftOverVoltage) {
			modPowerElectronicsPackStateHandle->chargeAllowed = false;
			modPowerElectronicsChargeRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_CELL_SOFT_OVER_VOLTAGE;
		}
		
		if( modPowerElectronicsPackStateHandle->tempBatteryHigh >= modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMax) {
			modPowerElectronicsPackStateHandle->chargeAllowed = false;
			modPowerElectronicsChargeRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_CHARGE_OVER_TEMP_CELLS;
		}
		
		if(modPowerElectronicsPackStateHandle->tempBatteryLow <= modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMin) {
			modPowerElectronicsPackStateHandle->chargeAllowed = false;
			modPowerElectronicsChargeRetryLastTick = HAL_GetTick();
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_CHARGE_UNDER_TEMP_CELLS;
		}
		
		//Enable discharge
		if(modPowerElectronicsPackStateHandle->cellVoltageLow >= (modPowerElectronicsGeneralConfigHandle->cellLCSoftUnderVoltage + modPowerElectronicsGeneralConfigHandle->hysteresisDischarge) && modPowerElectronicsPackStateHandle->tempBatteryHigh <= modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax && modPowerElectronicsPackStateHandle->tempBatteryLow >= modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMin) {
			if(modDelayTick1ms(&modPowerElectronicsDisChargeLCRetryLastTick,modPowerElectronicsGeneralConfigHandle->timeoutDischargeRetry)){
				modPowerElectronicsPackStateHandle->disChargeLCAllowed = true;
				modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_NONE;
			}	
		}
		//Enable charge
		if(modPowerElectronicsPackStateHandle->cellVoltageHigh <= (modPowerElectronicsGeneralConfigHandle->cellSoftOverVoltage - modPowerElectronicsGeneralConfigHandle->hysteresisCharge) && modPowerElectronicsPackStateHandle->tempBatteryHigh <= modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMax && modPowerElectronicsPackStateHandle->tempBatteryLow >= modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMin) {
			if(modDelayTick1ms(&modPowerElectronicsChargeRetryLastTick,modPowerElectronicsGeneralConfigHandle->timeoutChargeRetry)){
				modPowerElectronicsPackStateHandle->chargeAllowed = true;
				modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_NONE;
			}
		}
		
		//Handle cooling 
		if(modPowerElectronicsPackStateHandle->tempBatteryHigh <= modPowerElectronicsGeneralConfigHandle->allowedTempBattCoolingMax && modPowerElectronicsPackStateHandle->tempBatteryLow >= modPowerElectronicsGeneralConfigHandle->allowedTempBattCoolingMin){
			modPowerElectronicsPackStateHandle->coolingAllowed = false;
		}else{
			modPowerElectronicsPackStateHandle->coolingAllowed = true;
		};
		
	}
	
	// Handle hard cell voltage limits
	if(modPowerElectronicsVoltageSenseError || modPowerElectronicsPackStateHandle->cellVoltageHigh > modPowerElectronicsGeneralConfigHandle-> cellHardOverVoltage || modPowerElectronicsPackStateHandle->cellVoltageLow < modPowerElectronicsGeneralConfigHandle-> cellHardUnderVoltage || (modPowerElectronicsPackStateHandle->packVoltage > modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsGeneralConfigHandle->cellHardOverVoltage)) {
		if(modPowerElectronicsUnderAndOverVoltageErrorCount++ > modPowerElectronicsGeneralConfigHandle->maxUnderAndOverVoltageErrorCount){
			modPowerElectronicsPackStateHandle->packOperationalCellState = PACK_STATE_ERROR_HARD_CELLVOLTAGE;
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_MAX_UVP_OVP_ERRORS;
		}
		modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
		modPowerElectronicsPackStateHandle->chargeAllowed = false;
	}else
		modPowerElectronicsUnderAndOverVoltageErrorCount = 0;
	
		// Handle temperature limits
	if(modPowerElectronicsPackStateHandle->tempBatteryHigh > (modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax + 10.0f) || modPowerElectronicsPackStateHandle->tempBatteryLow < (modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMin - 10.0f)) {
		if(modPowerElectronicsUnderAndOverTemperatureErrorCount++ > modPowerElectronicsGeneralConfigHandle->maxUnderAndOverTemperatureErrorCount){
			modPowerElectronicsPackStateHandle->packOperationalCellState = PACK_STATE_ERROR_TEMPERATURE;
			modPowerElectronicsPackStateHandle->faultState = FAULT_CODE_MAX_UVT_OVT_ERRORS;
		}
		modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
		modPowerElectronicsPackStateHandle->chargeAllowed = false;
	}else
		modPowerElectronicsUnderAndOverTemperatureErrorCount = 0;
	
	// update outputs directly if needed
	if((lastChargeAllowed != modPowerElectronicsPackStateHandle->chargeAllowed) || (lastdisChargeLCAllowed != modPowerElectronicsPackStateHandle->disChargeLCAllowed)) {
		lastChargeAllowed = modPowerElectronicsPackStateHandle->chargeAllowed;
		lastdisChargeLCAllowed = modPowerElectronicsPackStateHandle->disChargeLCAllowed;
		modPowerElectronicsUpdateSwitches();
	}
};

void 	modPowerElectronicsSubTaskCurrentWatch(void){
		// Handle over current limits 
	if(modPowerElectronicsPackStateHandle->packCurrent > modPowerElectronicsGeneralConfigHandle->maxAllowedCurrent){
			modPowerElectronicsPackStateHandle->packOperationalCellState = PACK_STATE_ERROR_OVER_CURRENT;
			modPowerElectronicsPackStateHandle->disChargeLCAllowed = false;
			modPowerElectronicsPackStateHandle->chargeAllowed = false;
	}
};

// Update switch states, should be called after every desired/allowed switch state change
void modPowerElectronicsUpdateSwitches(void) {
	// Do the actual power switching in here
	
	//Handle precharge output
	if(modPowerElectronicsPackStateHandle->preChargeDesired && (modPowerElectronicsPackStateHandle->disChargeLCAllowed || modPowerElectronicsAllowForcedOnState)){
		driverHWSwitchesSetSwitchState(SWITCH_PRECHARGE,(driverHWSwitchesStateTypedef)SWITCH_SET);
		driverHWSwitchesSetSwitchState(SWITCH_DISCHARGEHV,(driverHWSwitchesStateTypedef)SWITCH_SET);
	}else{
		driverHWSwitchesSetSwitchState(SWITCH_PRECHARGE,(driverHWSwitchesStateTypedef)SWITCH_RESET);
		driverHWSwitchesSetSwitchState(SWITCH_DISCHARGEHV,(driverHWSwitchesStateTypedef)SWITCH_RESET);
	};
	//Handle discharge output
	if(modPowerElectronicsPackStateHandle->disChargeDesired && (modPowerElectronicsPackStateHandle->disChargeLCAllowed || modPowerElectronicsAllowForcedOnState)){
		driverHWSwitchesSetSwitchState(SWITCH_DISCHARGE,(driverHWSwitchesStateTypedef)SWITCH_SET);
		driverHWSwitchesSetSwitchState(SWITCH_DISCHARGEHV,(driverHWSwitchesStateTypedef)SWITCH_SET);
	}else{
		driverHWSwitchesSetSwitchState(SWITCH_DISCHARGE,(driverHWSwitchesStateTypedef)SWITCH_RESET);
	};
	//Handle charge input
	if(modPowerElectronicsPackStateHandle->chargeDesired && modPowerElectronicsPackStateHandle->chargeAllowed){
		driverHWSwitchesSetSwitchState(SWITCH_CHARGE,(driverHWSwitchesStateTypedef)SWITCH_SET);
	}else{
		driverHWSwitchesSetSwitchState(SWITCH_CHARGE,(driverHWSwitchesStateTypedef)SWITCH_RESET);
	};
	#if (HAS_PFET_OUTPUT)
	//Handle chargePFET input
	if(modPowerElectronicsPackStateHandle->chargePFETDesired && modPowerElectronicsPackStateHandle->chargeAllowed){
		driverHWSwitchesSetSwitchState(SWITCH_CHARGE_BYPASS,(driverHWSwitchesStateTypedef)SWITCH_SET);
	}else{
		driverHWSwitchesSetSwitchState(SWITCH_CHARGE_BYPASS,(driverHWSwitchesStateTypedef)SWITCH_RESET);
	};
	//Handle cooling output
	#else
	if(modPowerElectronicsPackStateHandle->coolingDesired && modPowerElectronicsPackStateHandle->coolingAllowed)
		driverHWSwitchesSetSwitchState(SWITCH_COOLING,(driverHWSwitchesStateTypedef)SWITCH_SET);
	else
		driverHWSwitchesSetSwitchState(SWITCH_COOLING,(driverHWSwitchesStateTypedef)SWITCH_RESET);
	#endif
};

void modPowerElectronicsSortCells(cellMonitorCellsTypeDef *cells, uint8_t cellCount) {
	int i,j;
	cellMonitorCellsTypeDef value;

	for(i=0 ; i<(cellCount-1) ; i++) {
		for(j=0 ; j<(cellCount-i-1) ; j++) {
				if(cells[j].cellVoltage < cells[j+1].cellVoltage) {
						value = cells[j+1];
						cells[j+1] = cells[j];
						cells[j] = value;
				}
		}
	}
};

void modPowerElectronicsCalcTempStats(void) {
	
	// Battery
	float   tempBatteryMax;
	float   tempBatteryMin;
	float   tempBatterySum = 0.0f;
	uint8_t tempBatterySumCount = 0;
	
	// BMS
	float   tempBMSMax;
	float   tempBMSMin;
	float   tempBMSSum = 0.0f;
	uint8_t tempBMSSumCount = 0;
	
	if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBattery || modPowerElectronicsGeneralConfigHandle->tempEnableMaskExpansion){
		tempBatteryMax = -100.0f;
		tempBatteryMin = 100.0f;
	}else{
		tempBatteryMax = 0.0f;
		tempBatteryMin = 0.0f;
	}
	
	if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBMS){
		tempBMSMax = -100.0f;
		tempBMSMin = 100.0f;
	}else{
		tempBMSMax = 0.0f;
		tempBMSMin = 0.0f;
	}
	
	// BMS temperatures
	
	for(uint8_t sensorPointer = 1; sensorPointer < 16; sensorPointer++){
		if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBMS & (1 << sensorPointer)){
			modPowerElectronicsPackStateHandle->temperatures[sensorPointer] = modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer-1].auxVoltage;
		}
	}
	
	for(uint8_t sensorPointer = 0; sensorPointer < 16; sensorPointer++){
	
		if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBMS & (1 << sensorPointer)){
			if(modPowerElectronicsPackStateHandle->temperatures[sensorPointer] > tempBMSMax)
				tempBMSMax = modPowerElectronicsPackStateHandle->temperatures[sensorPointer];
			
			if(modPowerElectronicsPackStateHandle->temperatures[sensorPointer] < tempBMSMin)
				tempBMSMin = modPowerElectronicsPackStateHandle->temperatures[sensorPointer];
			
			tempBMSSum += modPowerElectronicsPackStateHandle->temperatures[sensorPointer];
			tempBMSSumCount++;
		}
	}

	// Battery temperatures statistics for LTC aux channels without taking into account the first slave board temp measurement
	
	for(uint8_t sensorModulePointer = 0; sensorModulePointer < modPowerElectronicsGeneralConfigHandle->cellMonitorICCount; sensorModulePointer++) {
		for(uint8_t sensorPointer = 0; sensorPointer < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerModule; sensorPointer++) {
			if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBattery & (1 << sensorPointer)){
				if(modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer].auxVoltage > tempBatteryMax)
					tempBatteryMax = modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer].auxVoltage;
				
				if(modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer].auxVoltage < tempBatteryMin)
					tempBatteryMin = modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer].auxVoltage;
				
				tempBatterySum += modPowerElectronicsPackStateHandle->auxVoltagesIndividual[sensorPointer].auxVoltage;		
				tempBatterySumCount++;
			}
		}
	}
	
	// Battery temperatures statistics for expansion board
	for(uint8_t sensorModulePointer = 0; sensorModulePointer < modPowerElectronicsGeneralConfigHandle->noOfExpansionBoard; sensorModulePointer++) {
		for(uint8_t sensorPointer = 0; sensorPointer < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerExpansionBoard; sensorPointer++) {
			if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskExpansion & (1 << sensorPointer)){
				if(modPowerElectronicsPackStateHandle->expVoltagesIndividual[sensorPointer].expVoltage > tempBatteryMax)
					tempBatteryMax = modPowerElectronicsPackStateHandle->expVoltagesIndividual[sensorPointer].expVoltage;
			
				if(modPowerElectronicsPackStateHandle->expVoltagesIndividual[sensorPointer].expVoltage < tempBatteryMin)
					tempBatteryMin = modPowerElectronicsPackStateHandle->expVoltagesIndividual[sensorPointer].expVoltage;
		
			tempBatterySum += modPowerElectronicsPackStateHandle->expVoltagesIndividual[sensorPointer].expVoltage;
			tempBatterySumCount++;		
		}
	}
}
	

	
	// Battery temperatures
	modPowerElectronicsPackStateHandle->tempBatteryHigh    = tempBatteryMax;
	modPowerElectronicsPackStateHandle->tempBatteryLow     = tempBatteryMin;
	if(tempBatterySumCount)
		modPowerElectronicsPackStateHandle->tempBatteryAverage = tempBatterySum/tempBatterySumCount;
	else
		modPowerElectronicsPackStateHandle->tempBatteryAverage = 0.0f;
	
	// BMS temperatures
	modPowerElectronicsPackStateHandle->tempBMSHigh        = tempBMSMax;
	modPowerElectronicsPackStateHandle->tempBMSLow         = tempBMSMin;
	if(tempBMSSumCount)
		modPowerElectronicsPackStateHandle->tempBMSAverage = tempBMSSum/tempBMSSumCount;
	else
		modPowerElectronicsPackStateHandle->tempBMSAverage = 0.0f;
};

void modPowerElectronicsCalcThrottle(void) {
	static uint16_t filteredChargeThrottle = 0;
	static uint16_t filteredDisChargeThrottle = 0;
	
	// TODO make config to either do the throttling on the high or low current output
	// TODO Make lower percentages configurable
	
	//uint16_t  calculatedChargeThrottle = 0;
	uint16_t  calculatedDisChargeThrottle = 0;
	uint32_t  chargeIncreaseIntervalTime;
	uint16_t  chargeIncreaseRate;
	float     cellSoftUnderVoltage = modPowerElectronicsGeneralConfigHandle->cellLCSoftUnderVoltage;
	
	// Throttle charge
	float inputLowerLimitChargeVoltage               = modPowerElectronicsGeneralConfigHandle->cellSoftOverVoltage - modPowerElectronicsGeneralConfigHandle->cellThrottleUpperMargin - modPowerElectronicsGeneralConfigHandle->cellThrottleUpperStart;
	float inputUpperLimitChargeVoltage               = modPowerElectronicsGeneralConfigHandle->cellSoftOverVoltage - modPowerElectronicsGeneralConfigHandle->cellThrottleUpperMargin;
	float inputLowerLimitChargeTemperatureBattery    = modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMax - 3.0f;
	float inputUpperLimitChargeTemperatureBattery    = modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMax;	
	float outputLowerLimitCharge = 1000.0f;
	float outputUpperLimitCharge = 100.0f;
	
	// Throttle discharge
	float inputLowerLimitDisChargeVoltage            = cellSoftUnderVoltage + modPowerElectronicsGeneralConfigHandle->cellThrottleLowerMargin;
  float inputUpperLimitDisChargeVoltage            = cellSoftUnderVoltage + modPowerElectronicsGeneralConfigHandle->cellThrottleLowerMargin + modPowerElectronicsGeneralConfigHandle->cellThrottleLowerStart;
	float inputLowerLimitDisChargeTemperatureBattery = modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax - 3.0f;
	float inputUpperLimitDisChargeTemperatureBattery = modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax;
  float outputLowerLimitDisCharge = 50.0f;
	float outputUpperLimitDisCharge = 1000.0f;
	
	// Throttle general
	float inputLowerLimitTemperatureBMS = modPowerElectronicsGeneralConfigHandle->allowedTempBMSMax - 4.0f;
	float inputUpperLimitTemperatureBMS = modPowerElectronicsGeneralConfigHandle->allowedTempBMSMax;
	
	modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS       = (uint16_t)modPowerElectronicsMapVariableFloat(modPowerElectronicsPackStateHandle->tempBMSHigh,inputLowerLimitTemperatureBMS,inputUpperLimitTemperatureBMS,outputUpperLimitDisCharge,outputLowerLimitDisCharge);
	modPowerElectronicsPackStateHandle->throttleDutyChargeVoltage               = (uint16_t)modPowerElectronicsMapVariableFloat(modPowerElectronicsPackStateHandle->cellVoltageHigh,inputLowerLimitChargeVoltage,inputUpperLimitChargeVoltage,outputLowerLimitCharge,outputUpperLimitCharge);
	modPowerElectronicsPackStateHandle->throttleDutyChargeTemperatureBattery    = (uint16_t)modPowerElectronicsMapVariableFloat(modPowerElectronicsPackStateHandle->tempBatteryHigh,inputLowerLimitChargeTemperatureBattery,inputUpperLimitChargeTemperatureBattery,outputLowerLimitCharge,outputUpperLimitCharge);
	modPowerElectronicsPackStateHandle->throttleDutyDischargeVoltage            = (uint16_t)modPowerElectronicsMapVariableFloat(modPowerElectronicsPackStateHandle->cellVoltageLow,inputLowerLimitDisChargeVoltage,inputUpperLimitDisChargeVoltage,outputLowerLimitDisCharge,outputUpperLimitDisCharge);
	modPowerElectronicsPackStateHandle->throttleDutyDischargeTemperatureBattery = (uint16_t)modPowerElectronicsMapVariableFloat(modPowerElectronicsPackStateHandle->tempBatteryHigh,inputLowerLimitDisChargeTemperatureBattery,inputUpperLimitDisChargeTemperatureBattery,outputUpperLimitDisCharge,outputLowerLimitDisCharge);
		
	// Calculate (dis)charge throttle and pass it if in SOA
	if(modPowerElectronicsPackStateHandle->packInSOACharge){
		//modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS
		//modPowerElectronicsPackStateHandle->throttleDutyChargeVoltage
		//modPowerElectronicsPackStateHandle->throttleDutyChargeTemperatureBattery
		calculatedChargeThrottle = modPowerElectronicsLowestInThree(modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS,modPowerElectronicsPackStateHandle->throttleDutyChargeVoltage,modPowerElectronicsPackStateHandle->throttleDutyChargeTemperatureBattery);
	}else{
		calculatedChargeThrottle = 0;
	}
	
	if(modPowerElectronicsPackStateHandle->packInSOADischarge){
		//modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS
		//modPowerElectronicsPackStateHandle->throttleDutyDischargeVoltage
		//modPowerElectronicsPackStateHandle->throttleDutyDischargeTemperatureBattery
		
		calculatedDisChargeThrottle = modPowerElectronicsLowestInThree(modPowerElectronicsPackStateHandle->throttleDutyGeneralTemperatureBMS,modPowerElectronicsPackStateHandle->throttleDutyDischargeVoltage,modPowerElectronicsPackStateHandle->throttleDutyDischargeTemperatureBattery);
	}else{
		calculatedDisChargeThrottle = 0;
	}
	
	// Filter the calculated throttle charging
	if(calculatedChargeThrottle >= filteredChargeThrottle) {
		if(modPowerElectronicsChargeDeratingActive) {
			chargeIncreaseIntervalTime = 5000;
			chargeIncreaseRate         = 1;
    }else{
		  chargeIncreaseIntervalTime = 100;
			chargeIncreaseRate         = modPowerElectronicsGeneralConfigHandle->throttleChargeIncreaseRate;
		}
		
		if(modDelayTick1ms(&modPowerElectronicsChargeIncreaseLastTick,chargeIncreaseIntervalTime)){
			if(abs(calculatedChargeThrottle-filteredChargeThrottle) > chargeIncreaseRate) {
				filteredChargeThrottle += chargeIncreaseRate;		
			}else{
				filteredChargeThrottle = calculatedChargeThrottle;
			}
		}
	}else{
		modPowerElectronicsChargeDeratingActive = true;
		filteredChargeThrottle = calculatedChargeThrottle;
	}
	
	// Filter the calculated throttle discharging
	if(calculatedDisChargeThrottle >= filteredDisChargeThrottle){
		if((calculatedDisChargeThrottle-filteredDisChargeThrottle) > modPowerElectronicsGeneralConfigHandle->throttleDisChargeIncreaseRate) {
			filteredDisChargeThrottle += modPowerElectronicsGeneralConfigHandle->throttleDisChargeIncreaseRate;
		}else{
			filteredDisChargeThrottle = calculatedDisChargeThrottle;
		}
	}else{
		filteredDisChargeThrottle = calculatedDisChargeThrottle;
	}
	
  // Output the filtered output
	if(modPowerElectronicsPackStateHandle->chargeAllowed)
		modPowerElectronicsPackStateHandle->throttleDutyCharge = filteredChargeThrottle;
	else 
		modPowerElectronicsPackStateHandle->throttleDutyCharge = 0;
	
	// TODO have it configurable to either HC or LC
	if(modPowerElectronicsPackStateHandle->disChargeLCAllowed)
		modPowerElectronicsPackStateHandle->throttleDutyDischarge = filteredDisChargeThrottle;
	else 
		modPowerElectronicsPackStateHandle->throttleDutyDischarge = 0;
}

int32_t modPowerElectronicsMapVariableInt(int32_t inputVariable, int32_t inputLowerLimit, int32_t inputUpperLimit, int32_t outputLowerLimit, int32_t outputUpperLimit) {
	inputVariable = inputVariable < inputLowerLimit ? inputLowerLimit : inputVariable;
	inputVariable = inputVariable > inputUpperLimit ? inputUpperLimit : inputVariable;
	
	return (inputVariable - inputLowerLimit) * (outputUpperLimit - outputLowerLimit) / (inputUpperLimit - inputLowerLimit) + outputLowerLimit;
}

float modPowerElectronicsMapVariableFloat(float inputVariable, float inputLowerLimit, float inputUpperLimit, float outputLowerLimit, float outputUpperLimit) {
	inputVariable = inputVariable < inputLowerLimit ? inputLowerLimit : inputVariable;
	inputVariable = inputVariable > inputUpperLimit ? inputUpperLimit : inputVariable;
	
	return (inputVariable - inputLowerLimit) * (outputUpperLimit - outputLowerLimit) / (inputUpperLimit - inputLowerLimit) + outputLowerLimit;
}

void modPowerElectronicsInitISL(void) {
	// Init BUS monitor

	driverSWISL28022InitStruct ISLInitStruct;
	ISLInitStruct.ADCSetting = ADC_128_64010US;
	ISLInitStruct.busVoltageRange = BRNG_60V_1;
	ISLInitStruct.currentShuntGain = PGA_8_320MV;
	ISLInitStruct.Mode = MODE_SHUNTANDBUS_CONTINIOUS;
	driverSWISL28022Init(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,ISLInitStruct);
}

/*
void modPowerElectronicsInitINA(void) {

	// Init BUS monitor
	driverSWINA226InitStruct INAInitStruct;
	INAInitStruct.ADCSetting = ADC_128_64010US;
	INAInitStruct.busVoltageRange = BRNG_60V_1;
	INAInitStruct.currentShuntGain = PGA_8_320MV;
	ISNAInitStruct.Mode = MODE_SHUNTANDBUS_CONTINIOUS;

	driverSWINA226Init(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,INAInitStruct);

}
*/

void modPowerElectronicsCheckPackSOA(void) {
	static float hysteresysBMS       = -2.0f;
	static float hysteresysDischarge = -2.0f;
	static float hysteresysCharge    = -2.0f;
	
	static bool  lastPackInSOACharge    = true;
	static bool  lastPackInSOADisCharge = true;
	
	bool outsideLimitsBMS       = false;
	bool outsideLimitsDischarge = false;
	bool outsideLimitsCharge    = false;	
	
	outsideLimitsBMS |= (modPowerElectronicsVinErrorCount >= VinErrorThreshold) ? true : false;
	
	// Check BMS Limits
	if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBMS) {
		outsideLimitsBMS       |= (modPowerElectronicsPackStateHandle->tempBMSHigh     > (modPowerElectronicsGeneralConfigHandle->allowedTempBMSMax + hysteresysBMS) ) ? true : false;
		outsideLimitsBMS       |= (modPowerElectronicsPackStateHandle->tempBMSLow      < (modPowerElectronicsGeneralConfigHandle->allowedTempBMSMin - hysteresysBMS) ) ? true : false;
		
    if(outsideLimitsBMS)
			hysteresysBMS = -2.0f;
		else
			hysteresysBMS = 2.0f;
	}
	
	// Check Battery Limits discharge
	if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskBattery || modPowerElectronicsGeneralConfigHandle->tempEnableMaskExpansion) {
		outsideLimitsDischarge |= (modPowerElectronicsPackStateHandle->tempBatteryHigh > (modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMax + hysteresysDischarge) ) ? true : false;
		outsideLimitsDischarge |= (modPowerElectronicsPackStateHandle->tempBatteryLow  < (modPowerElectronicsGeneralConfigHandle->allowedTempBattDischargingMin - hysteresysDischarge) ) ? true : false;
		
    if(outsideLimitsDischarge)
			hysteresysDischarge = -2.0f;
		else
			hysteresysDischarge = 2.0f;
	}
	
	// Check Battery Limits charge
	if(modPowerElectronicsGeneralConfigHandle->tempEnableMaskExpansion) {
		outsideLimitsCharge    |= (modPowerElectronicsPackStateHandle->tempBatteryHigh > (modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMax + hysteresysCharge) ) ? true : false;
		outsideLimitsCharge    |= (modPowerElectronicsPackStateHandle->tempBatteryLow  < (modPowerElectronicsGeneralConfigHandle->allowedTempBattChargingMin - hysteresysCharge) ) ? true : false;

    if(outsideLimitsCharge)
			hysteresysCharge = -2.0f;
		else
			hysteresysCharge = 2.0f;
	}
	
	// DisCharge delayed response
	if(lastPackInSOADisCharge != !(outsideLimitsBMS || outsideLimitsDischarge)){
		if(modDelayTick1ms(&modPowerElectronicsSOADisChargeChangeLastTick,1000)) {
			lastPackInSOADisCharge = modPowerElectronicsPackStateHandle->packInSOADischarge = !(outsideLimitsBMS || outsideLimitsDischarge);
		}
	}else{
		modPowerElectronicsSOADisChargeChangeLastTick = HAL_GetTick();
	}	
	
	// Charge delayed response
	if(lastPackInSOACharge != !(outsideLimitsBMS || outsideLimitsCharge)){
		if(modDelayTick1ms(&modPowerElectronicsSOAChargeChangeLastTick,1000)) {
			lastPackInSOACharge = modPowerElectronicsPackStateHandle->packInSOACharge = !(outsideLimitsBMS || outsideLimitsCharge);
		}
	}else{
		modPowerElectronicsSOAChargeChangeLastTick = HAL_GetTick();
	}
}

bool modPowerElectronicsHCSafetyCANAndPowerButtonCheck(void) {
	if(modPowerElectronicsGeneralConfigHandle->useCANSafetyInput)
		return (modPowerElectronicsPackStateHandle->safetyOverCANHCSafeNSafe && (modPowerElectronicsPackStateHandle->powerButtonActuated | modPowerElectronicsGeneralConfigHandle->pulseToggleButton));
	else
		return true;
}

void modPowerElectronicsResetBalanceModeActiveTimeout(void) {
	modPowerElectronicsBalanceModeActiveLastTick = HAL_GetTick();
}

void modPowerElectronicsCellMonitorsInit(void){

			driverLTC6804ConfigStructTypedef configStruct;
			configStruct.GPIO1                    = true;																														// Do not pull down this pin (false = pull down)
			configStruct.GPIO2                    = true;																														// 
			configStruct.GPIO3                    = true;																														// 
			configStruct.GPIO4                    = true;																														// 
			configStruct.GPIO5                    = true;																														//
			configStruct.GPIO6                    = true;																														//
			configStruct.GPIO7                    = true;																														//
			configStruct.GPIO8                    = true;																														//
			configStruct.GPIO9                    = true;																														//
			configStruct.ReferenceON              = true;																														// Reference ON
			configStruct.ADCOption                = true;																											  		// ADC Option register for configuration of over sampling ratio
			configStruct.noOfCells                = modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule;			// Number of cells to monitor (that can cause interrupt)
			configStruct.DisChargeEnableMask      = 0x00000000;																									// Set enable state of discharge, 1=EnableDischarge, 0=DisableDischarge
			configStruct.DischargeTimout          = 0;																											// Discharge timout value / limit
			configStruct.CellUnderVoltageLimit    = modPowerElectronicsGeneralConfigHandle->cellHardUnderVoltage; // Undervoltage level, cell voltages under this limit will cause interrupt
			configStruct.CellOverVoltageLimit     = modPowerElectronicsGeneralConfigHandle->cellHardOverVoltage;  // Over voltage limit, cell voltages over this limit will cause interrupt
			driverSWLTC6804Init(configStruct, modPowerElectronicsGeneralConfigHandle->cellMonitorICCount, 18, 12, modPowerElectronicsGeneralConfigHandle->cellMonitorType);   
			
			// Safety signal is managed by the controller, it is configured as open drain and will be kept low by. watchdog will make the output to be released.
			//driverHWSwitchesSetSwitchState(SWITCH_SAFETY_OUTPUT,SWITCH_RESET);

	
	modPowerElectronicsCellMonitorsTypeActive = (configCellMonitorICTypeEnum)modPowerElectronicsGeneralConfigHandle->cellMonitorType;
}

void modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData(void){
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	
			// Check config valid and reinit
			// TODO: Implement
			
			// Read cell voltages
			driverSWLTC6804ReadCellVoltagesArray(modPowerElectronicsPackStateHandle->cellModuleVoltages);
			modPowerElectronicsCellMonitorsArrayTranslate();
			
				
			// Convert modules to full array
			
			// Read aux voltages
			driverSWLTC6804ReadAuxVoltagesArray(modPowerElectronicsPackStateHandle->auxModuleVoltages,modPowerElectronicsGeneralConfigHandle->NTC25DegResistance[modConfigNTCGroupLTCExt],modPowerElectronicsGeneralConfigHandle->NTCTopResistor[modConfigNTCGroupLTCExt],modPowerElectronicsGeneralConfigHandle->NTCBetaFactor[modConfigNTCGroupLTCExt],25.0f);
			modPowerElectronicsAuxMonitorsArrayTranslate();
			//driverSWLTC6804ReadAuxSensors(modPowerElectronicsAuxVoltageArray);
			//modPowerElectronicsPackStateHandle->temperatures[0] =	modPowerElectronicsPackStateHandle->temperatures[1] = driverSWLTC6804ConvertTemperatureExt(modPowerElectronicsAuxVoltageArray[1],modPowerElectronicsGeneralConfigHandle->NTC25DegResistance[modConfigNTCGroupLTCExt],modPowerElectronicsGeneralConfigHandle->NTCTopResistor[modConfigNTCGroupLTCExt],modPowerElectronicsGeneralConfigHandle->NTCBetaFactor[modConfigNTCGroupLTCExt],25.0f);
			
				//Read exp voltages
			driverSWADC128D818ReadExpVoltagesArray(modPowerElectronicsPackStateHandle->expModuleVoltages,modPowerElectronicsGeneralConfigHandle->NTC25DegResistance[modConfigNTCGroupExp],modPowerElectronicsGeneralConfigHandle->NTCTopResistor[modConfigNTCGroupExp],modPowerElectronicsGeneralConfigHandle->NTCBetaFactor[modConfigNTCGroupExp],25.0f);
			modPowerElectronicsExpMonitorsArrayTranslate();
}

void modPowerElectronicsCellMonitorsArrayTranslate(void) {
	uint8_t individualCellPointer = 0;
	
  for(uint8_t modulePointer = 0; modulePointer < modPowerElectronicsGeneralConfigHandle->cellMonitorICCount; modulePointer++) {
		if((modulePointer+1) % (modPowerElectronicsGeneralConfigHandle->cellMonitorICCount/modPowerElectronicsGeneralConfigHandle->noOfParallelModules)==0 && modulePointer != 0){ // DO NOT USE THIS FUNCTION
			for(uint8_t modulePointerCell = 0; modulePointerCell < modPowerElectronicsGeneralConfigHandle->lastICNoOfCells; modulePointerCell++) {
				if(individualCellPointer == 0 || individualCellPointer == 10 || individualCellPointer == 11 || individualCellPointer == 20 || individualCellPointer == 22 || individualCellPointer == 32 || individualCellPointer == 33 || individualCellPointer == 42 || individualCellPointer == 44 || individualCellPointer == 54 || individualCellPointer == 55 || individualCellPointer == 64 || individualCellPointer == 66 || individualCellPointer == 76 || individualCellPointer == 77 || individualCellPointer == 86 || individualCellPointer == 88 || individualCellPointer == 98 || individualCellPointer == 99 || individualCellPointer == 108){
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell] * 1.034f;
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
				}else{
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell];
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
				}
			}
		}else{ // use noOfCellsPerModule as usually
			for(uint8_t modulePointerCell = 0; modulePointerCell < modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule; modulePointerCell++) { //use this function - if you need to boost some cell values because of bad ADC readings put the multiplier in this function
				if(individualCellPointer == 0 || individualCellPointer == 10 || individualCellPointer == 11 || individualCellPointer == 20 || individualCellPointer == 22 || individualCellPointer == 32 || individualCellPointer == 33 || individualCellPointer == 42 || individualCellPointer == 44 || individualCellPointer == 54 || individualCellPointer == 55 || individualCellPointer == 64 || individualCellPointer == 66 || individualCellPointer == 76 ){
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell] * 1.034f;
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
				}else if(individualCellPointer == 77 || individualCellPointer == 86 || individualCellPointer == 88 || individualCellPointer == 98 || individualCellPointer == 99 || individualCellPointer == 108){
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell] * 1.034f;
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;	
				}
				else{
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellVoltage = modPowerElectronicsPackStateHandle->cellModuleVoltages[modulePointer][modulePointerCell];
					modPowerElectronicsPackStateHandle->cellVoltagesIndividual[individualCellPointer].cellNumber = individualCellPointer++;
				}
			}
		};
	}
}

void modPowerElectronicsAuxMonitorsArrayTranslate(void) {
	uint8_t individualAuxPointer = 0;
	
  for(uint8_t modulePointer = 0; modulePointer < modPowerElectronicsGeneralConfigHandle->cellMonitorICCount; modulePointer++) {
	  for(uint8_t modulePointerAux = 0; modulePointerAux < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerModule; modulePointerAux++) {
			if(modulePointerAux < 5){
				if (modPowerElectronicsPackStateHandle->auxModuleVoltages[modulePointer][modulePointerAux] == 100.0f){
					modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxVoltage = 10.0f;
					modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxNumber = individualAuxPointer++;
				}else{		
					modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxVoltage = modPowerElectronicsPackStateHandle->auxModuleVoltages[modulePointer][modulePointerAux];
					modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxNumber = individualAuxPointer++;
			}
			}else{ // when above 5, remove reference voltage measurement from Aux register group B : AVBR4 & AVBR5 for LTC6812 & LTC6813
				modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxVoltage = modPowerElectronicsPackStateHandle->auxModuleVoltages[modulePointer][modulePointerAux+1];
				modPowerElectronicsPackStateHandle->auxVoltagesIndividual[individualAuxPointer].auxNumber = individualAuxPointer++;
			}
		}
	}
}

void modPowerElectronicsExpMonitorsArrayTranslate(void) {
	uint8_t individualExpPointer = 0;
	
  for(uint8_t modulePointer = 0; modulePointer < modPowerElectronicsGeneralConfigHandle->noOfExpansionBoard; modulePointer++) {
	  for(uint8_t modulePointerExp = 0; modulePointerExp < modPowerElectronicsGeneralConfigHandle->noOfTempSensorPerExpansionBoard; modulePointerExp++) {	
			modPowerElectronicsPackStateHandle->expVoltagesIndividual[individualExpPointer].expVoltage = modPowerElectronicsPackStateHandle->expModuleVoltages[modulePointer][modulePointerExp];
			modPowerElectronicsPackStateHandle->expVoltagesIndividual[individualExpPointer].expNumber = individualExpPointer++;
		}
	}
}

void modPowerElectronicsCellMonitorsStartCellConversion(void) {
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804ResetCellVoltageRegisters();
	driverSWLTC6804StartCellVoltageConversion(MD_FILTERED,DCP_DISABLED,CELL_CH_ALL);
}

void modPowerElectronicsCellMonitorsStartLoadedCellConversion(bool PUP) {
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804ResetCellVoltageRegisters();
	driverSWLTC6804StartLoadedCellVoltageConversion(MD_FILTERED,DCP_ENABLED,CELL_CH_ALL,PUP);
}

void modPowerElectronicsCellMonitorsStartTemperatureConversion(void) {
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	
			// For other GPIOs voltages conversions, the below functions are used.
				driverSWLTC6804ResetAuxRegisters();
				driverSWLTC6804StartAuxVoltageConversion(MD_FILTERED, AUX_CH_ALL);
				
}

void modPowerElectronicsCellMonitorsEnableBalanceResistors(uint32_t balanceEnableMask){
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804EnableBalanceResistors(balanceEnableMask, modPowerElectronicsGeneralConfigHandle->cellMonitorType);
}

void modPowerElectronicsCellMonitorsEnableBalanceResistorsArrayTest(uint32_t balanceEnableMask){
	for(uint8_t moduleClearPointer = 0; moduleClearPointer < NoOfCellMonitorsPossibleOnBMS; moduleClearPointer++) 
		modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMaskTest[moduleClearPointer] = balanceEnableMask;
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804EnableBalanceResistorsArray(modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMaskTest, modPowerElectronicsGeneralConfigHandle->cellMonitorType);	
}


void modPowerElectronicsCellMonitorsEnableBalanceResistorsArray(){
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804EnableBalanceResistorsArray(modPowerElectronicsPackStateHandle->cellModuleBalanceResistorEnableMask, modPowerElectronicsGeneralConfigHandle->cellMonitorType);	
}

void modPowerElectronicsCellMonitorsReadVoltageFlags(uint32_t *underVoltageFlags, uint32_t *overVoltageFlags){
	modPowerElectronicsCellMonitorsCheckAndSolveInitState();
	driverSWLTC6804ReadVoltageFlags(underVoltageFlags,overVoltageFlags, modPowerElectronicsGeneralConfigHandle->lastICMask, modPowerElectronicsGeneralConfigHandle->noOfParallelModules, modPowerElectronicsPackStateHandle->dieTemperature);
}

void modPowerElectronicsCellMonitorsCheckAndSolveInitState(void){
  if(modPowerElectronicsCellMonitorsTypeActive != modPowerElectronicsGeneralConfigHandle->cellMonitorType){
		modPowerElectronicsCellMonitorsInit();
	}
}

void modPowerElectronicsTerminalCellConnectionTest(int argc, const char **argv) {
	(void)argc;
	(void)argv;
	uint32_t delayLastTick = HAL_GetTick();
	float conversionResults[5][modPowerElectronicsGeneralConfigHandle->noOfCellsSeries]; // unloaded, 100uAloaded, balance resistor load even and uneven.
	float difference;
	uint8_t cellIDPointer;
	bool passFail = true;
	bool overAllPassFail = true;
	
	float   argErrorVoltage = 1.0f;
	int32_t argBalanceDropMiliVoltage = 2;
	
	// Handle argument inputs
	modCommandsPrintf("---------     Test inputs:     ---------");
	if(argc == 3){
		// Two arguments given, taking this as balance and error threshold.
		sscanf(argv[1], "%f", &argErrorVoltage);
		sscanf(argv[2], "%d", &argBalanceDropMiliVoltage);
	}else{
	  modCommandsPrintf("No valid test arguments given. Using defaults:");
	}

	modCommandsPrintf("Error threshold:   %.1fV",argErrorVoltage);
	modCommandsPrintf("Balance threshold: %dmV",argBalanceDropMiliVoltage);
	// end of argument inputs
	
	
	// Start of general voltage test
	modCommandsPrintf("---  Starting voltage measure test  ---");	
	modCommandsPrintf("Pack voltage Direct   : %.2fV",modPowerElectronicsPackStateHandle->packVoltage);
	modCommandsPrintf("Pack voltage CVAverage: %.2fV",modPowerElectronicsPackStateHandle->cellVoltageAverage*modPowerElectronicsGeneralConfigHandle->noOfCellsSeries);
	modCommandsPrintf("Measure error         : %.2fV",fabs(modPowerElectronicsPackStateHandle->cellVoltageAverage*modPowerElectronicsGeneralConfigHandle->noOfCellsSeries-modPowerElectronicsPackStateHandle->packVoltage));
	
	if(fabs(modPowerElectronicsPackStateHandle->cellVoltageAverage*modPowerElectronicsGeneralConfigHandle->noOfCellsSeries-modPowerElectronicsPackStateHandle->packVoltage) > argErrorVoltage){
		passFail = overAllPassFail = false;
	}else{
		passFail = true;
	}
	modCommandsPrintf("Result                : %s",passFail ? "Pass" : "Fail");// Tell whether test passed / failed
	
	
	// Start of connection test
	while(!modDelayTick100ms(&delayLastTick,2)){};                     // Wait 
	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartCellConversion();              // Start ADC conversion
	while(!modDelayTick100ms(&delayLastTick,2)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
	  conversionResults[4][cellIDPointer] = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellIDPointer].cellVoltage;
	}
		
	
	modCommandsPrintf("------  Starting connectionTest  ------");
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait 
	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartLoadedCellConversion(false);   // Start ADC conversion
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait 
	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartLoadedCellConversion(false);   // Start ADC conversion
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
	  conversionResults[0][cellIDPointer] = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellIDPointer].cellVoltage;
	}

	
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartLoadedCellConversion(true);    // Start ADC conversion
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartLoadedCellConversion(true);    // Start ADC conversion
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
	modPowerElectronicsCellMonitorsStartLoadedCellConversion(true);    // Start ADC conversion
		
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
	  conversionResults[1][cellIDPointer] = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellIDPointer].cellVoltage;
	}
	
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
		difference = conversionResults[0][cellIDPointer] - conversionResults[1][cellIDPointer];                                          // Calculate difference
		
		if((conversionResults[0][cellIDPointer] >= modPowerElectronicsGeneralConfigHandle->cellHardOverVoltage) || (conversionResults[0][cellIDPointer] <= modPowerElectronicsGeneralConfigHandle->cellHardUnderVoltage)) {
			overAllPassFail = passFail = false;
		}else{
			if((cellIDPointer != 0) && (cellIDPointer != (modPowerElectronicsGeneralConfigHandle->noOfCellsSeries-1)) && ((cellIDPointer+1) % modPowerElectronicsGeneralConfigHandle->noOfCellsPerModule != 0) && (fabs(difference) >= 0.05f))
				overAllPassFail = passFail = false;
			else
			  passFail = true;
		}
		
	  modCommandsPrintf("[%2d] %.3fV - %.3fV = % .3fV -> %s",cellIDPointer+1,conversionResults[0][cellIDPointer],conversionResults[1][cellIDPointer],difference,passFail ? "Pass" : "Fail");  // Print the results
	}
	modCommandsPrintf("------    End connectionTest     ------");
	modCommandsPrintf("------    Start balance test     ------");
	
	uint32_t cellBalanceMaskEnableRegister = 0;
	for(int outputPointer = 0 ; outputPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries; outputPointer++) {
		cellBalanceMaskEnableRegister |= (1 << outputPointer);
	}

	// Even cells
	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();
	modPowerElectronicsCellMonitorsEnableBalanceResistorsArrayTest(cellBalanceMaskEnableRegister & 0x0002AAAA);
	
	for(int delay = 0; delay < 5; delay++){
		while(!modDelayTick100ms(&delayLastTick,3)){};                   // Wait
		modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	 // Read cell voltages from monitor
		modPowerElectronicsCellMonitorsStartLoadedCellConversion(false);            // Start ADC conversion
	}
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
		
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
	  conversionResults[2][cellIDPointer] = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellIDPointer].cellVoltage;
	}
	
	// Uneven cells
	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();
	modPowerElectronicsCellMonitorsEnableBalanceResistorsArrayTest(cellBalanceMaskEnableRegister & 0x00015555);
	
	
	for(int delay = 0; delay < 5; delay++){
		while(!modDelayTick100ms(&delayLastTick,3)){};                   // Wait
		modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	 // Read cell voltages from monitor
		modPowerElectronicsCellMonitorsStartLoadedCellConversion(false); // Start ADC conversion
	}
		
	while(!modDelayTick100ms(&delayLastTick,3)){};                     // Wait
  modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();	   // Read cell voltages from monitor
		
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
	  conversionResults[3][cellIDPointer] = modPowerElectronicsPackStateHandle->cellVoltagesIndividual[cellIDPointer].cellVoltage;
	}

	modPowerElectronicsCellMonitorsEnableBalanceResistorsArrayTest(0);
	
	for(cellIDPointer = 0; cellIDPointer < modPowerElectronicsGeneralConfigHandle->noOfCellsSeries ; cellIDPointer++){
		float difference = fabs(conversionResults[4][cellIDPointer] - conversionResults[2][cellIDPointer]) + fabs(conversionResults[4][cellIDPointer] - conversionResults[3][cellIDPointer]);                                          // Calculate difference
		
		if(difference >= (0.001f*argBalanceDropMiliVoltage))
			passFail = true;
		else
			overAllPassFail = passFail = false;
		
	  modCommandsPrintf("[%2d] %.3fV - %.3fV = % .3fV -> %s",cellIDPointer+1,conversionResults[2][cellIDPointer],conversionResults[3][cellIDPointer],difference,passFail ? "Pass" : "Fail");  // Print the results
	}
	
	modCommandsPrintf("------    End balance test     ------");
	modCommandsPrintf("------     Overall: %s       ------",overAllPassFail ? "Pass" : "Fail");// Tell whether test passed / failed
}

void modPowerElectronicsSamplePackAndLCData(void) {
	float tempPackVoltage;
	
	modPowerElectronicsSamplePackVoltage(&tempPackVoltage);
	modPowerElectronicsPackStateHandle->packVoltage = tempPackVoltage;
	modPowerElectronicsLCSenseSample();
	
	if(fabs(tempPackVoltage - modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsPackStateHandle->cellVoltageAverage) < 0.2f*(modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsPackStateHandle->cellVoltageAverage)) {    // If the error is different than 20% continue normal operation. 
		modPowerElectronicsVinErrorCount = 0;																								// Reset error count.
	}else{																																								// Error in voltage measurement.
		if(modPowerElectronicsVinErrorCount++ >= VinErrorThreshold){												// Increase error count
			modPowerElectronicsVinErrorCount = VinErrorThreshold;
			// Make BMS signal error state and power down.
			modPowerElectronicsVoltageSenseError = true;
		}else{
			modPowerElectronicsInitISL();																												// Reinit I2C and ISL
		}
	}
}

void modPowerElectronicsSamplePackVoltage(float *voltagePointer) {
	switch(modPowerElectronicsGeneralConfigHandle->packVoltageDataSource) {
		case sourcePackVoltageNone:
			break;
		case sourcePackVoltageISL28022:
				driverSWISL28022GetBusVoltage(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,voltagePointer,modPowerElectronicsGeneralConfigHandle->voltageLCOffset, modPowerElectronicsGeneralConfigHandle->voltageLCFactor);
			break;
		case sourcePackVoltageSumOfIndividualCellVoltages:
			*voltagePointer = modPowerElectronicsGeneralConfigHandle->noOfCellsSeries*modPowerElectronicsPackStateHandle->cellVoltageAverage;
			break;
		case sourcePackVoltageCANDieBieShunt:
			*voltagePointer = 0.0f;
			break;
		case sourcePackVoltageCANIsabellenhutte:
			*voltagePointer = 0.0f;
			break;
		case sourcePackVoltageINA226:
				//driverSWINA226GetBusVoltage(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,voltagePointer,modPowerElectronicsGeneralConfigHandle->voltageLCOffset, modPowerElectronicsGeneralConfigHandle->voltageLCFactor);
			break;		
		default:
			break;
	}
}

float modPowerElectronicsCalcPackCurrent(void){
	float returnCurrent = 0.0f;
	float returnCurrentTemp = 0.0f;

	switch(modPowerElectronicsGeneralConfigHandle->packCurrentDataSource){
		case sourcePackCurrentISL28022:
			returnCurrent = modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
			break;
		case sourcePackCurrentNone:
		case sourcePackCurrentCANDieBieShunt:
		case sourcePackCurrentCANIsaBellenHuette:
			returnCurrent = 0.0f;
			break;
		case sourcePackCurrentINA226:
			returnCurrent = modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
			break;
		case sourcePackCurrentCANVESC:
			if(modPowerStateChargerDetected()){
				returnCurrent = modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
			}else{
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg_4 *msg = comm_can_get_status_msg_4_index(i);

					if (msg->id >= 0 && modDelayTick1msNoRST(&modPowerElectronicsCANCurrentVESCLastTick,2000)) {
						returnCurrentTemp += msg->current_in;
					}
				}

				returnCurrent = returnCurrentTemp/CAN_STATUS_MSGS_TO_STORE;
			}
			break;
		default:
			break;
	}
	
	return returnCurrent;
}

void modPowerElectronicsLCSenseSample(void) {
		driverSWISL28022GetBusCurrent(ISL28022_MASTER_ADDRES,ISL28022_MASTER_BUS,&modPowerElectronicsPackStateHandle->loCurrentLoadCurrent,currentOffset, modPowerElectronicsGeneralConfigHandle->shuntLCFactor);
		driverHWADCGetLoadVoltage(&modPowerElectronicsPackStateHandle->loCurrentLoadVoltage, modPowerElectronicsGeneralConfigHandle->loadVoltageOffset, modPowerElectronicsGeneralConfigHandle->loadVoltageFactor);
		#if (HAS_NO_DISCHARGE)
			modPowerElectronicsPackStateHandle->loCurrentLoadVoltage = modPowerElectronicsPackStateHandle->packVoltage;
		#endif
	
	#if (HAS_CHARGER_VOLTAGE_MEASUREMENT)
		driverHWADCGetChargerVoltage(&modPowerElectronicsPackStateHandle->chargerVoltage, modPowerElectronicsGeneralConfigHandle->chargerVoltageOffset, modPowerElectronicsGeneralConfigHandle->chargerVoltageFactor);
	#endif
	
	//Calculate the zero current offset
	if(currentOffsetCounter < 2){
		//currentOffsetTemp += modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
		currentOffsetCounter++;
		if(currentOffsetCounter == 2){
			currentOffset = modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
		}
	}
}

uint16_t modPowerElectronicsLowestInThree(uint16_t num1,uint16_t num2,uint16_t num3) {
	if(num1 < num2 && num1 < num3) {
		return num1;
	}	else if(num2 < num3) {
		return num2;
	}	else{
		return num3;
	}
}

void  modPowerElectronicsResetCurrentOffset(void){
	currentOffset = modPowerElectronicsPackStateHandle->loCurrentLoadCurrent;
}


