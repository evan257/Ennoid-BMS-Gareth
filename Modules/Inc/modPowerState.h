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
 
#include "modDelay.h"
#include "driverHWPowerState.h"
#include "modConfig.h"

#define POWERBUTTON_POWERDOWN_THRESHOLD_TIME 1000
#define POWERBUTTON_FORCEON_THRESHOLD_TIME 5000
#define POWERBUTTON_DEBOUNCE_TIME 600

void modPowerStateInit(PowerStateStateTypedef desiredPowerState);
void modPowerStateSetConfigHandle(modConfigGeneralConfigStructTypedef *generalConfigPointer);
void modPowerStateTask(void);
bool modPowerStateGetButtonPressedState(void);
bool modPowerStateChargerDetected(void);
bool modPowerStatePowerdownRequest(void);
bool modPowerStateForceOnRequest(void);
void modPowerStateSetState(PowerStateStateTypedef newState);
bool modPowerStateButtonPressedOnTurnon(void);
bool modPowerStateGetLongButtonPressState(void);
