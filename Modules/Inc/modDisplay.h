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


#include "driverSWSSD1306.h"
#include "modDelay.h"
#include "libGraphics.h"
#include "libLogos.h"
#include "driverHWPowerState.h"

#define REFRESHTIMOUT	30																		// Interval between complete sync between desired and actual display content					
#define STARTUPDELAY	10																		// Startup delay, wait this amount of time before talking to the display

#ifndef MODDISPLAY_H_
#define MODDISPLAY_H_

typedef struct {
	float StateOfCharge;
	float Current;
	float PackVoltage;
	float LoadVoltage;
	float HighestTemp;
	float AverageTemp;
	float LowestTemp;
	float LowestCellVoltage;
	float HighestCellVoltage;
	float AverageCellVoltage;
	float Humidity;
	uint8_t DisplayStyle;
	uint8_t FaultCode;
	float CellMismatch;
	float ChargerVoltage;
} modDisplayDataTypedef;

typedef enum {
	DISP_MODE_OFF = 0,
	DISP_MODE_SPLASH,	
	DISP_MODE_LOAD,
	DISP_MODE_CHARGE,
	DISP_MODE_POWEROFF,
	DISP_MODE_ERROR,
	DISP_MODE_ERROR_PRECHARGE,
	DISP_MODE_EXTERNAL,
	DISP_MODE_BATTERY_DEAD,
	DISP_MODE_BALANCING,
	DISP_MODE_CHARGED,
	DISP_MODE_FORCED_ON
} modDisplayInfoType;

void modDisplayInit(void);
void modDisplayShowInfo(modDisplayInfoType newState,modDisplayDataTypedef modDisplayData);
void modDisplayTask(void);

void modDisplayWrite(float value, uint8_t decimals);
uint8_t modDisplay100ConvertValueToASCII(float value);
uint8_t modDisplay10ConvertValueToASCII(float value);
uint8_t modDisplay1ConvertValueToASCII(float value);
uint8_t modDisplay0_1ConvertValueToASCII(float value);
uint8_t modDisplay0_01ConvertValueToASCII(float value);
uint8_t modDisplay0_001ConvertValueToASCII(float value);
#endif
