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
 
#include "stm32f3xx_hal.h"
#include "stdbool.h"
#include "generalDefines.h"

#define NoOfSwitches  6 //7

typedef struct {
	GPIO_TypeDef* Port;
	uint32_t ClkRegister;
	uint32_t Pin;
	uint32_t Mode;
	uint32_t Pull;
} driverHWSwitchesPortStruct;

extern const driverHWSwitchesPortStruct driverHWSwitchesPorts[NoOfSwitches];

typedef enum {
	#if (ENNOID_SS || ENNOID_SS_24 || ENNOID_XLITE)
	SWITCH_CHARGE_BYPASS = 0,
	SWITCH_CHARGE,
	SWITCH_PRECHARGE,
	SWITCH_DISCHARGE,
	SWITCH_DISCHARGEHV,
	SWITCH_SAFETY_OUTPUT,
	#elif (ENNOID_SS_LITE )
	SWITCH_DISCHARGE = 0,
	SWITCH_CHARGE,
	SWITCH_PRECHARGE,
	SWITCH_CHARGE_BYPASS,
	SWITCH_DISCHARGEHV,
	SWITCH_SAFETY_OUTPUT,
	#else
	SWITCH_COOLING = 0,
	SWITCH_CHARGE,
	SWITCH_PRECHARGE,
	SWITCH_DISCHARGE,
	SWITCH_DISCHARGEHV,
	SWITCH_SAFETY_OUTPUT,

	#endif
	
} driverHWSwitchesIDTypedef;

typedef enum {
	SWITCH_RESET = 0,
	SWITCH_SET,
} driverHWSwitchesStateTypedef;

void driverHWSwitchesInit(void);
void driverHWSwitchesSetSwitchState(driverHWSwitchesIDTypedef switchID, driverHWSwitchesStateTypedef newState);
void driverHWSwitchesDisableAll(void);
bool driverHWSwitchesGetSwitchState(driverHWSwitchesIDTypedef switchID);
