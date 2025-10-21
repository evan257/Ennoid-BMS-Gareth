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

#include "driverSWADC128D818.h"
#include "modTerminal.h"

uint8_t driverSWADC128D818TotalNoOfExpansionBoard = 0;
uint8_t driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard = 0;

uint32_t driverSWADC128StartupDelayLastTick;
static const uint8_t ADC128Address[9] ={0x1D,0x1E,0x1F,0x2D,0x2E,0x2F,0x35,0x36,0x37};

void driverSWADC128D818Init(uint8_t noOfExpansionBoard, uint8_t noOfTempSensorPerExpansionBoard){
	
	driverSWADC128D818TotalNoOfExpansionBoard = noOfExpansionBoard;
	driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard = noOfTempSensorPerExpansionBoard;
	
	uint32_t maxDelayIntervals = 100;
	bool notReady = true;
	driverSWADC128StartupDelayLastTick = HAL_GetTick();
	
	uint8_t registerPointer = 0x0C;
	uint8_t readData = 0x03;
	for(uint8_t i=0; i < driverSWADC128D818TotalNoOfExpansionBoard; i++){
		// Check busy register
		while(maxDelayIntervals && notReady) {
		
			if(modDelayTick1ms(&driverSWADC128StartupDelayLastTick,10) && maxDelayIntervals){
				maxDelayIntervals--;
			
				driverHWI2C1Write(ADC128Address[i],false,&registerPointer,1);
				driverHWI2C1Read(ADC128Address[i],&readData,1);
			
				if(readData == 0x00)
					notReady = false;
			}
		
		}
	
		uint8_t configBytes[] = {0x00,0x08};		//configuration register, value = 0000 1000								// Configuration register -> disable ADC conversion and reset registers
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));
		
		configBytes[0] = 0x0B;																																						// Advanced config register Mode 1 + ext ref
		configBytes[1] = 0x03;		//Advanced Configuration Register, value = 0000 0011	
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));

		
		configBytes[0] = 0x07;																																						// Configuration rate register -> continuous conversion
		configBytes[1] = 0x01; 	//Conversion Rate Register, value = 0000 0001
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));
	
		configBytes[0] = 0x08;																																						// Channel disable register 
		configBytes[1] = 0x00;	//Channel Disable Register
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));
	
		configBytes[0] = 0x03;																																						// Interrupt enable register -> disable all interrupts
		configBytes[1] = 0xFF;	//Interrupt Mask Register, value = 1111 1111
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));
	
		configBytes[0] = 0x00;																																						// Configuration register -> enable ADC conversion
		configBytes[1] = 0x01;	//configuration register, value = 0000 0001
		driverHWI2C1Write(ADC128Address[i],false,configBytes,sizeof(configBytes));
	
	}
	// Now ready to read samples
}
/*
uint16_t driverSWADC128D818GetChannel(uint8_t adcChannel, uint8_t address){	
	if(adcChannel > 7)
		return 0;
	
	uint8_t registerPointer = 0x20 + adcChannel;
	uint8_t readBytes[2];
	driverHWI2C1Write(ADC128Address[address],false,&registerPointer,1);
	driverHWI2C1Read(ADC128Address[address],readBytes,2);
	
	return (readBytes[0] << 4) | (readBytes[1] >> 4);
}
*/

void driverSWADC128D818ReadExpVoltages(uint8_t noOfExpansionBoard, uint16_t exp_codes[][driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard]) {
	
		static uint8_t registerPointer = 0x20;
		uint8_t expRegisterPointer = 0;
		uint8_t readBytes[2];
	
	for(uint8_t modulePointer = 0; modulePointer< noOfExpansionBoard; modulePointer++){
		for(uint8_t expPointer = 0; expPointer < driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard; expPointer++){
			expRegisterPointer = registerPointer+expPointer;
			driverHWI2C1Write(ADC128Address[modulePointer],false,&expRegisterPointer,1);
			driverHWI2C1Read(ADC128Address[modulePointer],readBytes,2);
			exp_codes[modulePointer][expPointer] = ((readBytes[0] << 4) | (readBytes[1])>>4);
		}
	}
}

float driverSWADC128D818ConvertTemperatureExt(uint16_t inputValue,uint32_t ntcNominal,uint32_t ntcSeriesResistance,uint16_t ntcBetaFactor, float ntcNominalTemp) {
	static float scalar;
	static float steinhart;
	
  scalar = 4095.0f / (float)inputValue - 1.0f;
  scalar = (float)ntcSeriesResistance / scalar;
  steinhart = scalar / (float)ntcNominal;               // (R/Ro)
  steinhart = log(steinhart);                           // ln(R/Ro)
  steinhart /= (float)ntcBetaFactor;                    // 1/B * ln(R/Ro)
  steinhart += 1.0f / ((float)ntcNominalTemp + 273.15f);       // + (1/To)
  steinhart = 1.0f / steinhart;                         // Invert
  steinhart -= 273.15f;                                 // convert to degree
	
	if(steinhart < -50.0f || (float)inputValue >= 4095.0f || (float)inputValue <= 0.0f)
		steinhart = 100.0f;

  return steinhart;
}

bool driverSWADC128D818ReadExpVoltagesArray(float expVoltagesArray[][driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard],uint32_t ntcNominal,uint32_t ntcSeriesResistance, uint16_t ntcBetaFactor, float ntcNominalTemp) {
	bool dataValid = true;
	
	uint16_t expVoltageArrayCodes[driverSWADC128D818TotalNoOfExpansionBoard][driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard]; 
	
	driverSWADC128D818ReadExpVoltages(driverSWADC128D818TotalNoOfExpansionBoard,expVoltageArrayCodes);
	
  for(uint8_t modulePointer = 0; modulePointer < driverSWADC128D818TotalNoOfExpansionBoard; modulePointer++) {
		for(uint8_t channelPointer = 0; channelPointer < driverSWADC128D818MaxNoOfTempSensorPerExpansionBoard; channelPointer++){
			if(expVoltageArrayCodes[modulePointer][channelPointer]*0.001f < 5.0f)
			  expVoltagesArray[modulePointer][channelPointer] = driverSWADC128D818ConvertTemperatureExt(expVoltageArrayCodes[modulePointer][channelPointer], ntcNominal, ntcSeriesResistance, ntcBetaFactor, ntcNominalTemp);
			else
				dataValid = false;
		}
  }

	return dataValid;
}
