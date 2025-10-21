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
 
#include "driverHWSwitches.h"

const driverHWSwitchesPortStruct driverHWSwitchesPorts[NoOfSwitches] =			// Hold all status configuration data
{
	{GPIOB,RCC_AHBENR_GPIOBEN,GPIO_PIN_2,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL},		// SWITCH_COOLING or SWITCH_CHARGE_BYPASS
	{GPIOB,RCC_AHBENR_GPIOBEN,GPIO_PIN_10,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL},		// SWITCH_CHARGE
	{GPIOB,RCC_AHBENR_GPIOBEN,GPIO_PIN_11,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL},		// SWITCH_PRECHARGE
	{GPIOB,RCC_AHBENR_GPIOBEN,GPIO_PIN_10,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL},		// SWITCH_DISCHARGE or (SWITCH_CHARGE_BYPASS for LITE version only)
	{GPIOC,RCC_AHBENR_GPIOCEN,GPIO_PIN_15,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL},		// SWITCH_DISCHARGEHV
	{GPIOC,RCC_AHBENR_GPIOCEN,GPIO_PIN_13,GPIO_MODE_OUTPUT_PP,GPIO_NOPULL}//,	// SWITCH_SAFETY_OUTPUT
};

void driverHWSwitchesInit(void) {
	GPIO_InitTypeDef switchPortHolder;
	uint8_t SwitchPointer;
	
	for(SwitchPointer = 0; SwitchPointer < NoOfSwitches; SwitchPointer++) {
		RCC->AHBENR |= driverHWSwitchesPorts[SwitchPointer].ClkRegister;				// Enable clock de desired port
		switchPortHolder.Mode = driverHWSwitchesPorts[SwitchPointer].Mode;			// Push pull output
		switchPortHolder.Pin = driverHWSwitchesPorts[SwitchPointer].Pin;				// Points to status pin
		switchPortHolder.Pull = driverHWSwitchesPorts[SwitchPointer].Pull;			// Pullup
		switchPortHolder.Speed = GPIO_SPEED_HIGH;																// GPIO clock speed
		HAL_GPIO_Init(driverHWSwitchesPorts[SwitchPointer].Port,&switchPortHolder);// Perform the IO init 
	};
	
	driverHWSwitchesSetSwitchState(SWITCH_SAFETY_OUTPUT,SWITCH_SET);
};

void driverHWSwitchesSetSwitchState(driverHWSwitchesIDTypedef switchID, driverHWSwitchesStateTypedef newState) {
	HAL_GPIO_WritePin(driverHWSwitchesPorts[switchID].Port,driverHWSwitchesPorts[switchID].Pin,(GPIO_PinState)newState); // Set desired pin to desired state 
};

void driverHWSwitchesDisableAll(void) {
	uint8_t SwitchPointer;
	for(SwitchPointer = 0; SwitchPointer < NoOfSwitches; SwitchPointer++) {
		HAL_GPIO_WritePin(driverHWSwitchesPorts[SwitchPointer].Port,driverHWSwitchesPorts[SwitchPointer].Pin,(GPIO_PinState)SWITCH_RESET); // Set desired pin to desired state 
	};
};


bool driverHWSwitchesGetSwitchState(driverHWSwitchesIDTypedef switchID) {
	return (bool) HAL_GPIO_ReadPin(driverHWSwitchesPorts[switchID].Port,driverHWSwitchesPorts[switchID].Pin); // Set desired pin to desired state 
};


