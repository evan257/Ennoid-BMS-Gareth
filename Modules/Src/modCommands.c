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

#include "modCommands.h"
#include "confxml.h"
#include "confparser.h"
#include "libMempools.h"
#include "conf_default.h"


// Private variables
static uint8_t modCommandsSendBuffer[PACKET_MAX_PL_LEN];
static void(*modCommandsSendFunction)(unsigned char *data, unsigned int len) = 0;
bool jumpBootloaderTrue;
modConfigGeneralConfigStructTypedef *modCommandsGeneralConfig;
modConfigGeneralConfigStructTypedef *modCommandsToBeSendConfig;
modConfigGeneralConfigStructTypedef  modCommandsConfigStorage;
modPowerElectronicsPackStateTypedef *modCommandsGeneralState;

void modCommandsInit(modPowerElectronicsPackStateTypedef   *generalState,modConfigGeneralConfigStructTypedef *configPointer) {
	modCommandsGeneralConfig = configPointer;
	modCommandsGeneralState  = generalState;
	jumpBootloaderTrue = false;
}

void modCommandsSetSendFunction(void(*func)(unsigned char *data, unsigned int len)) {
	modCommandsSendFunction = func;
}

void modCommandsSendPacket(unsigned char *data, unsigned int len) {
	if (modCommandsSendFunction) {
		modCommandsSendFunction(data, len);
	}
}

void modCommandsProcessPacket(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;
	int32_t ind = 0;
	uint16_t flash_res;
	uint32_t new_app_offset;
	uint32_t delayTick;
	uint8_t cellPointer;
	uint8_t auxPointer;
	uint8_t expPointer;
	uint8_t totalNoOfCells;
	uint8_t totalNoOfAux;

	packet_id = (COMM_PACKET_ID) data[0];
	data++;
	len--;

	switch (packet_id) {
		case COMM_FW_VERSION:
			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_FW_VERSION;
			modCommandsSendBuffer[ind++] = FW_VERSION_MAJOR;
			modCommandsSendBuffer[ind++] = FW_VERSION_MINOR;
			strcpy((char*)(modCommandsSendBuffer + ind), HW_NAME);
			ind += strlen(HW_NAME) + 1;
			memcpy(modCommandsSendBuffer + ind, STM32_UUID_8, 12);
			ind += 12;
			modCommandsSendBuffer[ind++] = 0;
			modCommandsSendBuffer[ind++] = FW_TEST_VERSION_NUMBER;

			modCommandsSendBuffer[ind++] = HW_TYPE_VESC_BMS;

			modCommandsSendBuffer[ind++] = 1; // One custom config

			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_JUMP_TO_BOOTLOADER:
			jumpBootloaderTrue = true;
			delayTick = HAL_GetTick();
			break;
		case COMM_ERASE_NEW_APP:
			ind = 0;
			flash_res = modFlashEraseNewAppData(libBufferGet_uint32(data, &ind));

			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_ERASE_NEW_APP;
			modCommandsSendBuffer[ind++] = flash_res == HAL_OK ? true : false;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_WRITE_NEW_APP_DATA:
			ind = 0;
			new_app_offset = libBufferGet_uint32(data, &ind);
			flash_res = modFlashWriteNewAppData(new_app_offset, data + ind, len - ind);

			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_WRITE_NEW_APP_DATA;
			modCommandsSendBuffer[ind++] = flash_res == HAL_OK ? 1 : 0;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_EBMS_GET_VALUES:
			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_EBMS_GET_VALUES;
		
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packVoltage,			1e3, 		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packCurrent,			1e3,		&ind);
		  	libBufferAppend_uint8(modCommandsSendBuffer, (uint8_t)round(modCommandsGeneralState->SoC), 				&ind);
		  	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->cellVoltageHigh,		1e3,		&ind);
		  	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->cellVoltageAverage,		1e3,		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->cellVoltageLow,			1e3,		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->cellVoltageMisMatch,		1e3,		&ind);
		  	libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->loCurrentLoadVoltage,		1e1,		&ind);
		  	libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->loCurrentLoadCurrent,		1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->chargerVoltage,			1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBatteryHigh,		1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBatteryAverage,		1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBatteryLow,			1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBMSHigh,			1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBMSAverage,			1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBMSLow,			1e1,		&ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->humidity,			1e1,		&ind);
			libBufferAppend_uint8(modCommandsSendBuffer, (uint8_t)modCommandsGeneralState->operationalState,			&ind);
			libBufferAppend_uint8(modCommandsSendBuffer, (uint8_t)modCommandsGeneralState->chargeBalanceActive,			&ind);
			libBufferAppend_uint8(modCommandsSendBuffer, (uint8_t)modCommandsGeneralState->faultState,				&ind);
		
			modCommandsSendBuffer[ind++] = modCommandsGeneralConfig->CANID;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
		
			break;
    		case COMM_EBMS_GET_CELLS:
			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_EBMS_GET_CELLS;
		
		  	libBufferAppend_uint8(modCommandsSendBuffer, modCommandsGeneralConfig->noOfCellsSeries*modCommandsGeneralConfig->noOfParallelModules, &ind);                
		  	for(cellPointer = 0; cellPointer < modCommandsGeneralConfig->noOfCellsSeries*modCommandsGeneralConfig->noOfParallelModules; cellPointer++){
				if(modCommandsGeneralState->cellVoltagesIndividual[cellPointer].cellBleedActive)
					libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->cellVoltagesIndividual[cellPointer].cellVoltage*-1.0f, 1e3, &ind);    
				else
					libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->cellVoltagesIndividual[cellPointer].cellVoltage, 1e3, &ind);          
			}
		
			modCommandsSendBuffer[ind++] = modCommandsGeneralConfig->CANID;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_EBMS_GET_AUX:
			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_EBMS_GET_AUX;
		
		  	libBufferAppend_uint8(modCommandsSendBuffer, modCommandsGeneralConfig->cellMonitorICCount*modCommandsGeneralConfig->noOfTempSensorPerModule, &ind);                
		  	for(auxPointer = 0; auxPointer < modCommandsGeneralConfig->cellMonitorICCount*modCommandsGeneralConfig->noOfTempSensorPerModule; auxPointer++){
					libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->auxVoltagesIndividual[auxPointer].auxVoltage, 1e1, &ind);          
			}
		
			modCommandsSendBuffer[ind++] = modCommandsGeneralConfig->CANID;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_EBMS_GET_EXP_TEMP:
			ind = 0;
			modCommandsSendBuffer[ind++] = COMM_EBMS_GET_EXP_TEMP;
		
		  	libBufferAppend_uint8(modCommandsSendBuffer, modCommandsGeneralConfig->noOfExpansionBoard*modCommandsGeneralConfig->noOfTempSensorPerExpansionBoard, &ind);
		  	for(expPointer = 0; expPointer < modCommandsGeneralConfig->noOfExpansionBoard*modCommandsGeneralConfig->noOfTempSensorPerExpansionBoard; expPointer++){
					libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->expVoltagesIndividual[expPointer].expVoltage, 1e1, &ind);
			}
		
			modCommandsSendBuffer[ind++] = modCommandsGeneralConfig->CANID;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_EBMS_SET_MCCONF:
			ind = 0;
			modCommandsGeneralConfig->noOfCellsSeries				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->noOfCellsParallel				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->noOfParallelModules				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->batteryCapacity				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellHardUnderVoltage				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellHardOverVoltage				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellLCSoftUnderVoltage			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellSoftOverVoltage				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellBalanceDifferenceThreshold		= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellBalanceStart				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellBalanceAllTime				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->cellThrottleUpperStart			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellThrottleLowerStart			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellThrottleUpperMargin			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->cellThrottleLowerMargin			= libBufferGet_float32_auto(data,&ind);		// 4
		  	modCommandsGeneralConfig->packVoltageDataSource				= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->packCurrentDataSource				= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->buzzerSignalSource				= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->buzzerSignalPersistant			= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->shuntLCFactor					= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->voltageLCFactor				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->voltageLCOffset				= libBufferGet_int16(data,&ind);		// 2
			modCommandsGeneralConfig->loadVoltageFactor				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->loadVoltageOffset				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->chargerVoltageFactor				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->chargerVoltageOffset				= libBufferGet_float32_auto(data,&ind);		// 4
		  	modCommandsGeneralConfig->throttleChargeIncreaseRate			= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->throttleDisChargeIncreaseRate			= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->cellBalanceUpdateInterval			= libBufferGet_uint32(data,&ind);		// 4
		  	modCommandsGeneralConfig->maxSimultaneousDischargingCells		= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->timeoutDischargeRetry				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->hysteresisDischarge				= libBufferGet_float32_auto(data,&ind);		// 4
		  	modCommandsGeneralConfig->timeoutChargeRetry				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->hysteresisCharge				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->timeoutChargeCompleted			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->timeoutChargingCompletedMinimalMismatch	= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->maxMismatchThreshold				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->chargerEnabledThreshold			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->timeoutChargerDisconnected			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->minimalPrechargePercentage			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->timeoutLCPreCharge				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->maxAllowedCurrent				= libBufferGet_float32_auto(data,&ind);		// 4
			//Max allowed charge current
			modCommandsGeneralConfig->allowedTempBattDischargingMax			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBattDischargingMin			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBattChargingMax			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBattChargingMin			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBattCoolingMax			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBattCoolingMin			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBMSMax				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->allowedTempBMSMin				= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->displayTimeoutBatteryDead			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->displayTimeoutBatteryError			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->displayTimeoutBatteryErrorPreCharge		= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->displayTimeoutSplashScreen			= libBufferGet_uint32(data,&ind);		// 4
		  	modCommandsGeneralConfig->displayStyle					= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->maxUnderAndOverVoltageErrorCount		= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->maxUnderAndOverTemperatureErrorCount		= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->notUsedCurrentThreshold			= libBufferGet_float32_auto(data,&ind);		// 4
			modCommandsGeneralConfig->notUsedTimeout				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->stateOfChargeStoreInterval			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->stateOfChargeMethod				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->CANID						= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->CANIDStyle					= libBufferGet_uint8(data,&ind);		// 1
      		modCommandsGeneralConfig->canBusSpeed					= libBufferGet_uint8(data,&ind);		// 1 //Might be different name?
			//modCommandsGeneralConfig->emitStatusOverCAN				= libBufferGet_uint8(data,&ind);		// 1 //Not in Config Manager
			modCommandsGeneralConfig->emitStatusProtocol				= libBufferGet_uint8(data,&ind);		// 1
			//emitStatusCharger Protocol
			modCommandsGeneralConfig->tempEnableMaskBMS				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->tempEnableMaskBattery				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->tempEnableMaskExpansion			= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->noOfTempSensorPerModule			= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->noOfExpansionBoard				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->noOfTempSensorPerExpansionBoard		= libBufferGet_uint8(data,&ind);		// 1
		  	modCommandsGeneralConfig->LCUseDischarge				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->LCUsePrecharge				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->allowChargingDuringDischarge			= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->allowForceOn					= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->pulseToggleButton				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->useCANSafetyInput				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->useCANDelayedPowerDown			= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->NTCTopResistor[modConfigNTCGroupLTCExt]	= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTC25DegResistance[modConfigNTCGroupLTCExt]	= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTCBetaFactor[modConfigNTCGroupLTCExt]	= libBufferGet_uint16(data,&ind);		// 2
			modCommandsGeneralConfig->NTCTopResistor[modConfigNTCGroupMasterPCB]	= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTC25DegResistance[modConfigNTCGroupMasterPCB]= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTCBetaFactor[modConfigNTCGroupMasterPCB]	= libBufferGet_uint16(data,&ind);		// 2
			modCommandsGeneralConfig->NTCTopResistor[modConfigNTCGroupExp]		= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTC25DegResistance[modConfigNTCGroupExp]	= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->NTCBetaFactor[modConfigNTCGroupExp]		= libBufferGet_uint16(data,&ind);		// 2
			modCommandsGeneralConfig->cellMonitorType				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->cellMonitorICCount				= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->externalEnableOperationalState		= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->chargeEnableOperationalState			= libBufferGet_uint8(data,&ind);		// 1
			modCommandsGeneralConfig->powerDownDelay				= libBufferGet_uint32(data,&ind);		// 4
			modCommandsGeneralConfig->humidityICType				= libBufferGet_uint8(data,&ind);		// 1
			
			ind = 0;
			modCommandsSendBuffer[ind++] = packet_id;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			
			modconfigHardwareLimitsApply(modCommandsGeneralConfig);
		
			break;
		case COMM_EBMS_GET_MCCONF:
		case COMM_EBMS_GET_MCCONF_DEFAULT:
      			if(packet_id == COMM_EBMS_GET_MCCONF_DEFAULT){
				modConfigLoadDefaultConfig(&modCommandsConfigStorage);
				modCommandsToBeSendConfig = &modCommandsConfigStorage;
			}else{
				modCommandsToBeSendConfig = modCommandsGeneralConfig;
			}
		
      			ind = 0;
		  	modCommandsSendBuffer[ind++] = packet_id;
		  
		  	libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfCellsSeries					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfCellsParallel				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfParallelModules				,&ind); // 1
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->batteryCapacity					,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellHardUnderVoltage				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellHardOverVoltage				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellLCSoftUnderVoltage				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellSoftOverVoltage				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellBalanceDifferenceThreshold			,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellBalanceStart					,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->cellBalanceAllTime				,&ind); // 1
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellThrottleUpperStart				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellThrottleLowerStart				,&ind); // 4
		  	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellThrottleUpperMargin				,&ind); // 4
		 	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->cellThrottleLowerMargin				,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->packVoltageDataSource				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->packCurrentDataSource				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->buzzerSignalSource				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->buzzerSignalPersistant				,&ind); // 1
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->shuntLCFactor					,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->voltageLCFactor					,&ind); // 4
			libBufferAppend_int16(        modCommandsSendBuffer,modCommandsToBeSendConfig->voltageLCOffset					,&ind); // 2
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->loadVoltageFactor				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->loadVoltageOffset				,&ind); // 4
		 	libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->chargerVoltageFactor				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->chargerVoltageOffset				,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->throttleChargeIncreaseRate			,&ind); // 1
		  	libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->throttleDisChargeIncreaseRate			,&ind); // 1
		  	libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->cellBalanceUpdateInterval			,&ind); // 4
		  	libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->maxSimultaneousDischargingCells			,&ind); // 1
		  	libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutDischargeRetry				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->hysteresisDischarge				,&ind); // 4
		  	libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutChargeRetry				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->hysteresisCharge					,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutChargeCompleted				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutChargingCompletedMinimalMismatch		,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->maxMismatchThreshold				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->chargerEnabledThreshold				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutChargerDisconnected			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->minimalPrechargePercentage			,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->timeoutLCPreCharge				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->maxAllowedCurrent				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattDischargingMax			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattDischargingMin			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattChargingMax			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattChargingMin			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattCoolingMax			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBattCoolingMin			,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBMSMax				,&ind); // 4
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->allowedTempBMSMin				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->displayTimeoutBatteryDead			,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->displayTimeoutBatteryError			,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->displayTimeoutBatteryErrorPreCharge		,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->displayTimeoutSplashScreen			,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->displayStyle					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->maxUnderAndOverVoltageErrorCount			,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->maxUnderAndOverTemperatureErrorCount		,&ind); // 1
			libBufferAppend_float32_auto( modCommandsSendBuffer,modCommandsToBeSendConfig->notUsedCurrentThreshold				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->notUsedTimeout					,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->stateOfChargeStoreInterval			,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->stateOfChargeMethod				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->CANID						,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->CANIDStyle					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->canBusSpeed					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->emitStatusOverCAN				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->emitStatusProtocol				,&ind); // 1
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->tempEnableMaskBMS				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->tempEnableMaskBattery				,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->tempEnableMaskExpansion				,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfTempSensorPerModule				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfExpansionBoard				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->noOfTempSensorPerExpansionBoard			,&ind); // 1
		  	libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->LCUseDischarge					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->LCUsePrecharge					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->allowChargingDuringDischarge			,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->allowForceOn					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->pulseToggleButton				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->useCANSafetyInput				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->useCANDelayedPowerDown				,&ind); // 1
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCTopResistor[modConfigNTCGroupLTCExt]		,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTC25DegResistance[modConfigNTCGroupLTCExt]	,&ind); // 4
			libBufferAppend_uint16(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCBetaFactor[modConfigNTCGroupLTCExt]		,&ind); // 2
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCTopResistor[modConfigNTCGroupMasterPCB]	,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTC25DegResistance[modConfigNTCGroupMasterPCB]	,&ind); // 4
			libBufferAppend_uint16(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCBetaFactor[modConfigNTCGroupMasterPCB]	,&ind); // 2
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCTopResistor[modConfigNTCGroupExp]		,&ind); // 4
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTC25DegResistance[modConfigNTCGroupExp]		,&ind); // 4
			libBufferAppend_uint16(       modCommandsSendBuffer,modCommandsToBeSendConfig->NTCBetaFactor[modConfigNTCGroupExp]		,&ind); // 2
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->cellMonitorType					,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->cellMonitorICCount				,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->externalEnableOperationalState			,&ind); // 1
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->chargeEnableOperationalState			,&ind); // 1	
			libBufferAppend_uint32(       modCommandsSendBuffer,modCommandsToBeSendConfig->powerDownDelay					,&ind); // 4
			libBufferAppend_uint8(        modCommandsSendBuffer,modCommandsToBeSendConfig->humidityICType					,&ind); // 1
			
		  	modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		case COMM_TERMINAL_CMD:
		  	data[len] = '\0';
		  	modTerminalProcessString((char*)data);
			break;
		case COMM_REBOOT:
			modCommandsJumpToMainApplication();
			break;
		case COMM_ALIVE:
			break;
		case COMM_FORWARD_CAN:
			modCANSendBuffer(data[0], data + 1, len - 1, false);
			break;
		case COMM_EBMS_STORE_CONF:
			modConfigStoreConfig();
		
			ind = 0;
			modCommandsSendBuffer[ind++] = packet_id;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;
		//Specific to VESC tool app
		case COMM_BMS_GET_VALUES: 
			ind = 0;

			modCommandsSendBuffer[ind++] = COMM_BMS_GET_VALUES;

		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packVoltage,			1e6, 		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->chargerVoltage,			1e6, 		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packCurrent,			1e6, 		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packCurrent,			1e6, 		&ind);
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packCurrent,			1e3, 		&ind); //TO DO: define AhCounter instead of packCurrent
		 	libBufferAppend_float32(modCommandsSendBuffer, modCommandsGeneralState->packVoltage,			1e3, 		&ind); //TO DO: define WhCounter instead of packCurrent

			// Cell voltages
			totalNoOfCells = modCommandsGeneralConfig->noOfCellsSeries*modCommandsGeneralConfig->noOfParallelModules;
			modCommandsSendBuffer[ind++] = totalNoOfCells;
			for (cellPointer = 0; cellPointer < totalNoOfCells; cellPointer++) {
				libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->cellVoltagesIndividual[cellPointer].cellVoltage, 1e3, &ind);
			}

			// Balancing state
			for (int i = 0; i < totalNoOfCells; i++) {
				modCommandsSendBuffer[ind++] = modCommandsGeneralState->cellVoltagesIndividual[i].cellBleedActive;
			}

			// Temperatures
			totalNoOfAux = modCommandsGeneralConfig->cellMonitorICCount*modCommandsGeneralConfig->noOfTempSensorPerModule;
			modCommandsSendBuffer[ind++] = totalNoOfAux;
			for (auxPointer = 0; auxPointer < totalNoOfAux; auxPointer++) {
				libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->auxVoltagesIndividual[auxPointer].auxVoltage, 1e2, &ind); 
			}
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->temperatures[0], 1e2, &ind); 

			// Humidity
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->humidity, 1e2, &ind);
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->temperatures[1], 1e2, &ind);

			// Highest cell temperature
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->tempBatteryHigh, 1e2, &ind);

			// State of charge and state of health
			libBufferAppend_float16(modCommandsSendBuffer, modCommandsGeneralState->SoC/100, 1e3, &ind);
			libBufferAppend_float16(modCommandsSendBuffer, 0.0, 1e3, &ind);

		  	modCommandsSendPacket(modCommandsSendBuffer, ind);
			break;

		case COMM_BMS_SET_CHARGE_ALLOWED:
			modCommandsGeneralState->chargeAllowed = true;
			break;

		case COMM_BMS_SET_BALANCE_OVERRIDE:
			//modCommandsGeneralConfig->cellBalanceAllTime = true;
			break;

		case COMM_BMS_RESET_COUNTERS:
			if (data[0]) {
			//To do
			}

			if (data[1]) {
			//To do
			}
			break;

		case COMM_BMS_FORCE_BALANCE:
			modCommandsGeneralConfig->cellBalanceAllTime = true;
			break;
		case COMM_BMS_ZERO_CURRENT_OFFSET:
			modPowerElectronicsResetCurrentOffset();
		break;
		case COMM_GET_CUSTOM_CONFIG:
		case COMM_GET_CUSTOM_CONFIG_DEFAULT: {
			main_config_t *conf = libMempools_alloc_conf();

			int conf_ind = data[0];

			if (conf_ind != 0) {
				break;
			}

			if (packet_id == COMM_GET_CUSTOM_CONFIG) {
				modCommandsEBMSToVESC(conf);
			} else {
				confparser_set_defaults_main_config_t(conf);
			}

			ind = 0;
			modCommandsSendBuffer[ind++] = packet_id;
			modCommandsSendBuffer[ind++] = conf_ind;
			int32_t len = confparser_serialize_main_config_t(modCommandsSendBuffer + ind, conf);
			modCommandsSendPacket(modCommandsSendBuffer, len + ind);
			libMempools_free_conf(conf);
		} break;

		case COMM_SET_CUSTOM_CONFIG: {
			main_config_t *conf = libMempools_alloc_conf();

			int conf_ind = data[0];

			if (conf_ind == 0 && confparser_deserialize_main_config_t(data + 1, conf)) {
				modCommandsVESCToEBMS(conf);

				ind = 0;
				//modCommandsSendBuffer[50];
				modCommandsSendBuffer[ind++] = packet_id;
				modCommandsSendPacket(modCommandsSendBuffer, ind);
			} else {
				modCommandsPrintf("Warning: Could not set configuration");
			}

			libMempools_free_conf(conf);
		} break;

		case COMM_GET_CUSTOM_CONFIG_XML: {
			ind = 0;

			int conf_ind = data[ind++];

			if (conf_ind != 0) {
				break;
			}

			int32_t len_conf = libBufferGet_int32(data, &ind);
			int32_t ofs_conf = libBufferGet_int32(data, &ind);

			if ((len_conf + ofs_conf) > DATA_MAIN_CONFIG_T__SIZE || len_conf > (PACKET_MAX_PL_LEN - 10)) {
			break;
			}

			ind = 0;
			modCommandsSendBuffer[ind++] = packet_id;
			modCommandsSendBuffer[ind++] = conf_ind;
			libBufferAppend_int32(modCommandsSendBuffer, DATA_MAIN_CONFIG_T__SIZE, &ind);
			libBufferAppend_int32(modCommandsSendBuffer, ofs_conf, &ind);
			memcpy(modCommandsSendBuffer + ind, data_main_config_t_ + ofs_conf, len_conf);
			ind += len_conf;
			modCommandsSendPacket(modCommandsSendBuffer, ind);
			

		} break;
		default:
			break;
	}
	
	if(modDelayTick1ms(&delayTick,1000) && jumpBootloaderTrue)
		modFlashJumpToBootloader();
}

void modCommandsPrintf(const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer+1, 254, format, arg);
	va_end (arg);

	if(len > 0) {
		modCommandsSendPacket((unsigned char*)print_buffer, (len<254)? len+1: 255);
	}
}

void modCommandsVESCToEBMS(main_config_t *conf) {
	
	modCommandsGeneralConfig->CANID	= conf->controller_id;
	modCommandsGeneralConfig->canBusSpeed = conf->can_baud_rate;
	modCommandsGeneralConfig->noOfCellsSeries = conf->cell_num;
	modCommandsGeneralConfig->noOfTempSensorPerModule = conf->temp_num;
	modCommandsGeneralConfig->cellBalanceStart = conf->balance_start_voltage;
	modCommandsGeneralConfig->maxMismatchThreshold = conf->balance_difference_threshold;
	modCommandsGeneralConfig->cellSoftOverVoltage = conf->soft_overvoltage;
	modCommandsGeneralConfig->cellLCSoftUnderVoltage = conf->soft_undervoltage;
	modCommandsGeneralConfig->cellHardOverVoltage = conf->hard_overvoltage;
	modCommandsGeneralConfig->cellHardUnderVoltage = conf->hard_undervoltage;
	modCommandsGeneralConfig->allowedTempBattChargingMax = conf->t_charge_max;
	modCommandsGeneralConfig->allowedTempBattDischargingMax = conf->t_discharge_max;
	modCommandsGeneralConfig->notUsedCurrentThreshold = conf->not_used_current_threshold;
	modCommandsGeneralConfig->notUsedTimeout = conf->not_used_timeout;

}

void modCommandsEBMSToVESC(main_config_t *conf) {
	conf->controller_id = modCommandsGeneralConfig->CANID;
	conf->can_baud_rate = modCommandsGeneralConfig->canBusSpeed;
	conf->cell_num = modCommandsGeneralConfig->noOfCellsSeries;
	conf->temp_num = modCommandsGeneralConfig->noOfTempSensorPerModule;
	conf->balance_start_voltage = modCommandsGeneralConfig->cellBalanceStart;
	conf->balance_difference_threshold = modCommandsGeneralConfig->maxMismatchThreshold;
	conf->soft_overvoltage = modCommandsGeneralConfig->cellSoftOverVoltage;
	conf->soft_undervoltage = modCommandsGeneralConfig->cellLCSoftUnderVoltage;
	conf->hard_overvoltage = modCommandsGeneralConfig->cellHardOverVoltage;
	conf->hard_undervoltage = modCommandsGeneralConfig->cellHardUnderVoltage;
	conf->t_charge_max = modCommandsGeneralConfig->allowedTempBattChargingMax;
	conf->t_discharge_max = modCommandsGeneralConfig->allowedTempBattDischargingMax;
	conf->not_used_current_threshold = modCommandsGeneralConfig->notUsedCurrentThreshold;
	conf->not_used_timeout = modCommandsGeneralConfig->notUsedTimeout;
}

void modCommandsJumpToMainApplication(void) {
	NVIC_SystemReset();
}
