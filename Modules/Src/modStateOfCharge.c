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
 
#include "modStateOfCharge.h"

modStateOfChargeStructTypeDef modStateOfChargeGeneralStateOfCharge;
modPowerElectronicsPackStateTypedef *modStateOfChargePackStatehandle;
modConfigGeneralConfigStructTypedef *modStateOfChargeGeneralConfigHandle;
uint32_t modStateOfChargeLargeCoulombTick;
uint32_t modStateOfChargeStoreSoCTick;

bool modStateOfChargePowerDownSavedFlag = false;

modStateOfChargeStructTypeDef* modStateOfChargeInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer){
	modStateOfChargePackStatehandle = packState;
	modStateOfChargeGeneralConfigHandle = generalConfigPointer;
	driverSWStorageManagerStateOfChargeStructSize = (sizeof(modStateOfChargeStructTypeDef)/sizeof(uint16_t)); // Calculate the space needed for the config struct in EEPROM
	
	modStateOfChargeLargeCoulombTick = HAL_GetTick();
	modStateOfChargeStoreSoCTick = HAL_GetTick();
	
	return &modStateOfChargeGeneralStateOfCharge;
};

void modStateOfChargeProcess(void){
	// Calculate accumulated energy
	uint32_t dt = HAL_GetTick() - modStateOfChargeLargeCoulombTick;
	modStateOfChargeStructTypeDef lastGeneralStateOfCharge;
	
	lastGeneralStateOfCharge = modStateOfChargeGeneralStateOfCharge;
	
	modStateOfChargeLargeCoulombTick = HAL_GetTick();
	modStateOfChargeGeneralStateOfCharge.remainingCapacityAh += dt*modStateOfChargePackStatehandle->packCurrent/(3600*1000);// (miliseconds * amps)/(3600*1000) accumulatedCharge in AmpHour.
	
	// Cap the max stored energy to the configured battery capacity.
	if(modStateOfChargeGeneralStateOfCharge.remainingCapacityAh > modStateOfChargeGeneralConfigHandle->batteryCapacity)
		modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = modStateOfChargeGeneralConfigHandle->batteryCapacity;
	
	if(modStateOfChargeGeneralStateOfCharge.remainingCapacityAh < 0.0f)
		modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = 0.0f;
	
	// Calculate state of charge
	modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh / modStateOfChargeGeneralConfigHandle->batteryCapacity * 100.0f;
	
	if(modStateOfChargeGeneralStateOfCharge.generalStateOfCharge >= 100.0f)
		modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = 100.0f;
	
	modStateOfChargePackStatehandle->SoC = modStateOfChargeGeneralStateOfCharge.generalStateOfCharge;
	modStateOfChargePackStatehandle->SoCCapacityAh = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh;
	
	// Store SoC every 'stateOfChargeStoreInterval'
	if(modDelayTick1ms(&modStateOfChargeStoreSoCTick,modStateOfChargeGeneralConfigHandle->stateOfChargeStoreInterval) && !modStateOfChargePowerDownSavedFlag && (lastGeneralStateOfCharge.remainingCapacityAh != modStateOfChargeGeneralStateOfCharge.remainingCapacityAh))
		modStateOfChargeStoreStateOfCharge();
};

bool modStateOfChargeStoreAndLoadDefaultStateOfCharge(void){
	bool returnVal = false;
	if(driverSWStorageManagerStateOfChargeEmpty){
		// TODO: SoC manager is empy -> Determin SoC from voltage when voltages are available.
		modStateOfChargeStructTypeDef defaultStateOfCharge;
		defaultStateOfCharge.generalStateOfCharge = 100.0f;
		defaultStateOfCharge.generalStateOfHealth = 100.0f;
		defaultStateOfCharge.remainingCapacityAh = modStateOfChargeGeneralConfigHandle->batteryCapacity;
		defaultStateOfCharge.remainingCapacityWh = 0.0f;
		
		driverSWStorageManagerStateOfChargeEmpty = false;
		driverSWStorageManagerStoreStruct(&defaultStateOfCharge,STORAGE_STATEOFCHARGE);
		// TODO_EEPROM
	}
	
	modStateOfChargeStructTypeDef tempStateOfCharge;
	driverSWStorageManagerGetStruct(&tempStateOfCharge,STORAGE_STATEOFCHARGE);
	
	modStateOfChargeLoadStateOfCharge();
	return returnVal;
};

bool modStateOfChargeStoreStateOfCharge(void){
	return driverSWStorageManagerStoreStruct(&modStateOfChargeGeneralStateOfCharge,STORAGE_STATEOFCHARGE);
};

bool modStateOfChargeLoadStateOfCharge(void){
	return driverSWStorageManagerGetStruct(&modStateOfChargeGeneralStateOfCharge,STORAGE_STATEOFCHARGE);
};

bool modStateOfChargePowerDownSave(void) {
	if(!modStateOfChargePowerDownSavedFlag) {
		modStateOfChargePowerDownSavedFlag = true;
		modStateOfChargeStoreStateOfCharge();
		// TODO_EEPROM
		return true;
	}else
		return false;
};

void modStateOfChargeVoltageEvent(modStateOfChargeVoltageEventTypeDef eventType) {
	switch(eventType) {
		case EVENT_EMPTY:
			break;
		case EVENT_FULL:
			modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = modStateOfChargeGeneralConfigHandle->batteryCapacity;
			break;
		default:
			break;
	}
};
