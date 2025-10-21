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
 
#include "driverSWISL28022.h"

void driverSWISL28022Init(uint8_t i2cAddres, uint8_t i2cBus, driverSWISL28022InitStruct initStruct){
	
	uint8_t firstByte = initStruct.busVoltageRange << 6 | initStruct.currentShuntGain << 4 | (initStruct.ADCSetting & 0x06) >> 1;
	uint8_t secondByte = (initStruct.ADCSetting & 0x01) << 7 | initStruct.ADCSetting << 3 | initStruct.Mode ;
	uint8_t writeData[3] = {0x00,firstByte,secondByte};
	
	if(i2cBus == 1){
		driverHWI2C1Init();
		driverHWI2C1Write(i2cAddres,false,writeData,3);
	}else if(i2cBus == 2){
		driverHWI2C2Init();
		driverHWI2C2Write(i2cAddres,false,writeData,3);
	}
};

bool driverSWISL28022GetBusCurrent(uint8_t i2cAddres, uint8_t i2cBus, float *busCurrent, float initOffset, float scalar) {
	// ToDo make register to current conversion register dependent
	uint8_t writeDataC[1] = {REG_SHUNTVOLTAGE};
	uint8_t readDataC[2];
	uint8_t commSucces = HAL_OK;
	static int16_t busCurrentInt;

	if(i2cBus == 1){
		commSucces |= driverHWI2C1Write(i2cAddres,false,writeDataC,1);
		commSucces |= driverHWI2C1Read(i2cAddres,readDataC,2);
	}else if(i2cBus == 2){
		commSucces |= driverHWI2C2Write(i2cAddres,false,writeDataC,1);
		commSucces |= driverHWI2C2Read(i2cAddres,readDataC,2);
	}
	
	if(commSucces == HAL_OK) {
		busCurrentInt = (readDataC[0] << 9) | (readDataC[1] << 1);
		*busCurrent = (scalar*busCurrentInt)-initOffset;
	}else{
		*busCurrent = 0.0f;
	}
	
	return commSucces == HAL_OK;
};

bool driverSWISL28022GetBusVoltage(uint8_t i2cAddres, uint8_t i2cBus, float *busVoltage, int16_t offset, float scalar){
	uint8_t writeDataV[1] = {REG_BUSVOLTAGE};
	uint8_t readDataV[2];
	uint8_t commSucces = HAL_OK;
	uint16_t busVoltageInt;
	float step = 0.004f;
	
	if(i2cBus == 1){
	  commSucces |= driverHWI2C1Write(i2cAddres,false,writeDataV,1);
	  commSucces |= driverHWI2C1Read(i2cAddres,readDataV,2);
	}else if(i2cBus == 2){
	  commSucces |= driverHWI2C2Write(i2cAddres,false,writeDataV,1);
	  commSucces |= driverHWI2C2Read(i2cAddres,readDataV,2);
	}

	if(commSucces == HAL_OK) {
		busVoltageInt = (readDataV[0] << 6) | (readDataV[1] >> 2);
		*busVoltage = step*scalar*(busVoltageInt+offset);	
  }else{
		*busVoltage = 0.0f;
  }		
	
	return commSucces == HAL_OK;
};
