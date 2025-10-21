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
 
#include "modPowerState.h"

modConfigGeneralConfigStructTypedef *modPowerStateGeneralConfigHandle;

bool modPowerStatePulsePowerDownDesired;
bool modPowerStateForceOnDesired;
bool modPowerStateButtonPressedVar;
bool modPowerStateLastButtonPressedVar;
bool modPowerStateLastButtonFirstPress;
bool modPowerStateButtonActuated;
bool modPowerStateButtonPulsToggleMode;
bool modPowerStatePowerModeDirectHCDelay;
bool modPowerStateLongStartupButtonPress;
uint32_t modPowerStateButtonPressedDuration;
uint32_t modPowerStateButtonPressedTimeStamp;
uint32_t modPowerStateStartupDelay;
uint32_t modPowerStatePowerDownTimeout;

bool tempButtonPressed;

void modPowerStateInit(PowerStateStateTypedef desiredPowerState) {
	//uint32_t startupDelay = HAL_GetTick();
	modPowerStateStartupDelay = HAL_GetTick();
	modPowerStatePowerDownTimeout = HAL_GetTick();
	modPowerStatePulsePowerDownDesired = false;
	modPowerStateForceOnDesired  = false;
	modPowerStateButtonPressedVar = true;
	modPowerStateLongStartupButtonPress = false;
	modPowerStateButtonPulsToggleMode = true;
	modPowerStateButtonPressedDuration = 0;
	modPowerStateButtonPressedTimeStamp = 0;
	modPowerStateButtonActuated = false;
	
	driverHWPowerStateInit();
	
	while(!modDelayTick1ms(&modPowerStateStartupDelay,20));										// Needed for power button signal to reach uC
	modPowerStateLastButtonFirstPress = modPowerStateLastButtonPressedVar = driverHWPowerStateReadInput(P_STAT_BUTTON_INPUT);
	
	if(!modPowerStateLastButtonFirstPress) {
		while(!modDelayTick1ms(&modPowerStateStartupDelay,100));
	}

	modPowerStateSetState(desiredPowerState);
};



void modPowerStateSetConfigHandle(modConfigGeneralConfigStructTypedef *generalConfigPointer) {
	modPowerStateGeneralConfigHandle = generalConfigPointer;
	
	modPowerStateButtonPulsToggleMode   = modPowerStateGeneralConfigHandle->pulseToggleButton;
	modPowerStatePowerModeDirectHCDelay = modPowerStateGeneralConfigHandle->togglePowerModeDirectHCDelay;
}

void modPowerStateTask(void) {
	//bool tempButtonPressed = driverHWPowerStateReadInput(P_STAT_BUTTON_INPUT);
	tempButtonPressed = driverHWPowerStateReadInput(P_STAT_BUTTON_INPUT);
	
	if(modPowerStateLastButtonPressedVar != tempButtonPressed) {
		if(modPowerStateLastButtonPressedVar){ 																	// If is was high and now low (actuated)
			modPowerStateLastButtonFirstPress = false;
			modPowerStateButtonActuated = true;
		}else{ 																																	// If is was low and now high (non actuated)
			modPowerStateButtonPressedTimeStamp = HAL_GetTick();
			modPowerStateButtonActuated = false;
		}
		modPowerStateLastButtonPressedVar = tempButtonPressed;
	}
	
	if(tempButtonPressed) {
		modPowerStateButtonPressedDuration = HAL_GetTick() - modPowerStateButtonPressedTimeStamp;
		
		if((modPowerStateButtonPressedDuration > POWERBUTTON_DEBOUNCE_TIME) && (modPowerStateLastButtonFirstPress == false))
			modPowerStateButtonPressedVar = true;
	
		if((modPowerStateButtonPressedDuration >= POWERBUTTON_POWERDOWN_THRESHOLD_TIME) && (modPowerStateLastButtonFirstPress == false) && modPowerStateGeneralConfigHandle->pulseToggleButton) {
			modPowerStatePulsePowerDownDesired = true;
			modPowerStateButtonPressedDuration = 0;
		}
		
		if((modPowerStateButtonPressedDuration >= POWERBUTTON_FORCEON_THRESHOLD_TIME) && (modPowerStateLastButtonFirstPress == true) && modPowerStateGeneralConfigHandle->pulseToggleButton) {
			if(modPowerStateGeneralConfigHandle->allowForceOn) {
				modPowerStateForceOnDesired = true;
			}
			
			modPowerStateLongStartupButtonPress = true;
			modPowerStateButtonPressedDuration = 0;
		}
		
		modPowerStatePowerDownTimeout = HAL_GetTick();
	}else{
		if(modDelayTick1ms(&modPowerStatePowerDownTimeout,500) && modPowerStateButtonActuated)
			modPowerStateButtonPressedVar = false;
	}
};

bool modPowerStateGetButtonPressedState(void) {
	return modPowerStateButtonPressedVar;
};

bool modPowerStateChargerDetected(void) {
	static bool chargeDetect = false;
	chargeDetect = driverHWPowerStateReadInput(P_STAT_CHARGE_DETECT);
	return chargeDetect;
};

bool modPowerStatePowerdownRequest(void) {
	bool returnValue = false;
	
	if(modPowerStateGeneralConfigHandle->pulseToggleButton){
		returnValue = modPowerStatePulsePowerDownDesired;
	}else{
			returnValue = !modPowerStateButtonPressedVar;
		}
	return returnValue;
};

bool modPowerStateForceOnRequest(void) {
	static bool firstTrigger = true;
	
	if(modPowerStateForceOnDesired && firstTrigger){
		modPowerStateForceOnDesired = false;
		firstTrigger = false;
		return true;
	}else{
		return false;
	}
};

void modPowerStateSetState(PowerStateStateTypedef newState) {
	driverHWPowerStateSetOutput(P_STAT_POWER_ENABLE,newState);
};

bool modPowerStateButtonPressedOnTurnon(void) {
	return modPowerStateLastButtonFirstPress;
};

bool modPowerStateGetLongButtonPressState(void) {
  return modPowerStateLongStartupButtonPress;
}
