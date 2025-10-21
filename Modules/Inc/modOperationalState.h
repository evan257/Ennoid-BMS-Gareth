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
 
#include "stdbool.h"
#include "mainDataTypes.h"
#include "modDelay.h"
#include "modPowerState.h"
#include "modEffect.h"
#include "modPowerElectronics.h"
#include "modDisplay.h"
#include "modConfig.h"
#include "modStateOfCharge.h"
#include "math.h"

#ifndef MODOPERATIONALSTATE_H_
#define MODOPERATIONALSTATE_H_

void modOperationalStateInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer,modStateOfChargeStructTypeDef *generalStateOfCharge);
void modOperationalStateTask(void);
void modOperationalStateUpdateStates(void);
void modOperationalStateSetAllStates(OperationalStateTypedef newState);
void modOperationalStateSetNewState(OperationalStateTypedef newState);

void modOperationalStateUpdateFaultStates(void);
void modOperationalStateSetAllFaultStates(bms_fault_state newFaultState);
void modOperationalStateSetNewFaultState(bms_fault_state newFaultState);

void modOperationalStateHandleChargerDisconnect(OperationalStateTypedef newState);
void modOperationalStateTerminateOperation(void);
bool modOperationalStateDelayedDisable(bool delayedPowerDownDesired);
bool modOperationalStatePowerDownDelayCheck(void);

#endif
