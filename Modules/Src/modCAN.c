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

#include "modCAN.h"

// Variables
CAN_HandleTypeDef      modCANHandle;
uint32_t               modCANErrorLastTick;
uint32_t               modCANSendStatusFastLastTisk;
uint32_t               modCANSendStatusSlowLastTisk;
uint32_t               modCANSendStatusVESCLastTisk;
uint32_t               modCANSafetyCANMessageTimeout;
uint32_t               modCANLastRXID;
uint32_t               modCANLastRXDifferLastTick;
static uint8_t         modCANRxBuffer[RX_CAN_BUFFER_SIZE];
static uint8_t         modCANRxBufferLastID;
static CanRxMsgTypeDef modCANRxFrames[RX_CAN_FRAMES_SIZE];
static uint8_t         modCANRxFrameRead;
static uint8_t         modCANRxFrameWrite;

uint32_t               modCANLastChargerHeartBeatTick;
uint32_t               modCANChargerTaskIntervalLastTick;
bool                   modCANChargerPresentOnBus;
bool				   highVoltageFault = false;
uint8_t                modCANChargerCANOpenState;
uint8_t                modCANChargerChargingState;

ChargerStateTypedef chargerOpState = opInit;
ChargerStateTypedef chargerOpStateNew = opInit;

modPowerElectronicsPackStateTypedef *modCANPackStateHandle;
modConfigGeneralConfigStructTypedef *modCANGeneralConfigHandle;

// Private variables
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_2 stat_msgs_2[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_3 stat_msgs_3[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_4 stat_msgs_4[CAN_STATUS_MSGS_TO_STORE];
static can_status_msg_5 stat_msgs_5[CAN_STATUS_MSGS_TO_STORE];

/*
bool modCANPing(uint8_t controller_id, HW_TYPE *hw_type) {
	ping_tp = chThdGetSelfX();
	chEvtGetAndClearEvents(ALL_EVENTS);

	uint8_t buffer[1];
	buffer[0] = backup.config.controller_id;
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_PING << 8), buffer, 1);

	int ret = chEvtWaitAnyTimeout(1 << 29, TIME_MS2I(10));
	ping_tp = 0;

	if (ret != 0) {
		if (hw_type) {
			*hw_type = ping_hw_last;
		}
	}

	return ret != 0;
}

*/

void modCANInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer){
  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;
	
	modCANPackStateHandle = packState;
	modCANGeneralConfigHandle = generalConfigPointer;
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
  modCANHandle.Instance = CAN;
  modCANHandle.pTxMsg = &TxMessage;
  modCANHandle.pRxMsg = &RxMessage;
	
	switch(modCANGeneralConfigHandle->canBusSpeed) {
		case canSpeedBaud125k:
			modCANHandle.Init.Prescaler = 36;
			modCANHandle.Init.Mode = CAN_MODE_NORMAL;
			modCANHandle.Init.SJW = CAN_SJW_1TQ;
			modCANHandle.Init.BS1 = CAN_BS1_5TQ;
			modCANHandle.Init.BS2 = CAN_BS2_2TQ;
			break;
		case canSpeedBaud250k:
			modCANHandle.Init.Prescaler = 18;
			modCANHandle.Init.Mode = CAN_MODE_NORMAL;
			modCANHandle.Init.SJW = CAN_SJW_1TQ;
			modCANHandle.Init.BS1 = CAN_BS1_5TQ;
			modCANHandle.Init.BS2 = CAN_BS2_2TQ;
			break;
		case canSpeedBaud500k:
		default:
			modCANHandle.Init.Prescaler = 9;
			modCANHandle.Init.Mode = CAN_MODE_NORMAL;
			modCANHandle.Init.SJW = CAN_SJW_1TQ;
			modCANHandle.Init.BS1 = CAN_BS1_5TQ;
			modCANHandle.Init.BS2 = CAN_BS2_2TQ;
			break;
	}
	
	modCANHandle.Init.TTCM = DISABLE;
	modCANHandle.Init.ABOM = ENABLE; // Enable this for automatic recovery?
	modCANHandle.Init.AWUM = DISABLE;
	modCANHandle.Init.NART = DISABLE;
	modCANHandle.Init.RFLM = DISABLE;
	modCANHandle.Init.TXFP = DISABLE;
	

	//No init error recovery this will hang - Gareth Siegel 7/25/2024	
  if(HAL_CAN_Init(&modCANHandle) != HAL_OK)
    while(true){};
			
  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 0;
  HAL_CAN_ConfigFilter(&modCANHandle, &canFilterConfig);

  if(HAL_CAN_Receive_IT(&modCANHandle, CAN_FIFO0) != HAL_OK)
    while(true){};

	modCANRxFrameRead = 0;
	modCANRxFrameWrite = 0;
			
	modCANSendStatusFastLastTisk = HAL_GetTick();
	modCANSendStatusSlowLastTisk = HAL_GetTick();
	modCANSendStatusVESCLastTisk = HAL_GetTick();
	modCANSafetyCANMessageTimeout = HAL_GetTick();
	modCANErrorLastTick = HAL_GetTick();
}

void modCANTask(void){		
	// Manage HAL CAN driver's active state
	if((modCANHandle.State != HAL_CAN_STATE_BUSY_RX)) {
				//		if(modDelayTick1ms(&modCANErrorLastTick,1000))
	  HAL_CAN_Receive_IT(&modCANHandle, CAN_FIFO0);
	}else{
		modCANErrorLastTick = HAL_GetTick();
	}
	
	if(modCANGeneralConfigHandle->emitStatusOverCAN) {
		if(modCANGeneralConfigHandle->emitStatusProtocol == canEmitProtocolDieBieEngineering) {
			// Send status messages with interval
			if(modDelayTick1ms(&modCANSendStatusFastLastTisk,200))                        // 5 Hz
				modCANSendStatusFast();
		
			// Send status messages with interval
			if(modDelayTick1ms(&modCANSendStatusSlowLastTisk,500))                        // 2 Hz
				modCANSendStatusSlow();
		}else if(modCANGeneralConfigHandle->emitStatusProtocol == canEmitProtocolVESC){
			if(modDelayTick1ms(&modCANSendStatusVESCLastTisk,1000)) 
				modCANSendStatusVESC();
		}
	}
	
	if(modDelayTick1ms(&modCANSafetyCANMessageTimeout,5000))
		modCANPackStateHandle->safetyOverCANHCSafeNSafe = false;
		
	// Handle received CAN bus data
	modCANSubTaskHandleCommunication();
	modCANRXWatchDog();

	/*C5 Fix 2 - Charging CAN Message
	  
	  Per the rules if the safety circuit is
	  tripped the charger must shut off and 
	  the only way to do that is to stop sending
	  the CAN message so this code reads the same
	  safety signal coming from the HV PCB that stops
	  balancing and decides to stop sending the CAN message
	
    */
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
		highVoltageFault = true;
	} else {
		highVoltageFault = false;
	}


	// Control the charger
	if(modCANPackStateHandle->faultState == false && highVoltageFault == false) {
	modCANHandleSubTaskCharger();
	}
}

uint32_t modCANGetDestinationID(CanRxMsgTypeDef canMsg) {
	uint32_t destinationID;
	
	switch(modCANGeneralConfigHandle->CANIDStyle) {
		default:																																					// Default to VESC style ID
	  case CANIDStyleVESC:
			destinationID = canMsg.ExtId & 0xFF;
			break;
		case CANIDStyleFoiler:
			destinationID = (canMsg.ExtId >> 8) & 0xFF;
			break;
	}
	
	return destinationID;
}

CAN_PACKET_ID modCANGetPacketID(CanRxMsgTypeDef canMsg) {
	CAN_PACKET_ID packetID;

	switch(modCANGeneralConfigHandle->CANIDStyle) {
		default:																																					// Default to VESC style ID
	  case CANIDStyleVESC:
			packetID = (CAN_PACKET_ID)((canMsg.ExtId >> 8) & 0xFF);
			break;
		case CANIDStyleFoiler:
			packetID = (CAN_PACKET_ID)((canMsg.ExtId) & 0xFF);
			break;
	}
	
	return packetID;
}

uint32_t modCANGetCANID(uint32_t destinationID, CAN_PACKET_ID packetID) {
	uint32_t returnCANID;
	
	switch(modCANGeneralConfigHandle->CANIDStyle) {
		default:																																					// Default to VESC style ID
	  case CANIDStyleVESC:
			returnCANID = ((uint32_t) destinationID) | ((uint32_t)packetID << 8);
			break;
		case CANIDStyleFoiler:
			returnCANID = ((uint32_t) destinationID << 8) | ((uint32_t)packetID);
			break;
	}
	
  return returnCANID;
}

void modCANSendStatusFast(void) {
	int32_t sendIndex;
	uint8_t buffer[8];
	uint8_t flagHolder = 0;
	uint8_t disChargeDesiredMask;
	
	if(modCANGeneralConfigHandle->togglePowerModeDirectHCDelay || modCANGeneralConfigHandle->pulseToggleButton){
		disChargeDesiredMask = modCANPackStateHandle->disChargeDesired && modPowerElectronicsHCSafetyCANAndPowerButtonCheck();
	}else{
		disChargeDesiredMask = modCANPackStateHandle->disChargeDesired && modCANPackStateHandle->powerButtonActuated && modPowerElectronicsHCSafetyCANAndPowerButtonCheck();
	}
	
	flagHolder |= (modCANPackStateHandle->chargeAllowed		<< 0);
	flagHolder |= (modCANPackStateHandle->chargeDesired		<< 1);
	flagHolder |= (modCANPackStateHandle->disChargeLCAllowed	<< 2);
	flagHolder |= (disChargeDesiredMask				<< 3);
	flagHolder |= (modCANPackStateHandle->balanceActive		<< 4);
	flagHolder |= (modCANPackStateHandle->packInSOADischarge	<< 5);
	flagHolder |= (modCANPackStateHandle->chargePFETDesired		<< 6);
	flagHolder |= (modCANPackStateHandle->stateError	<< 7); //comp fix
	
	// Send (dis)charge throttle and booleans.
	sendIndex = 0;
	libBufferAppend_float16(buffer, modCANPackStateHandle->tempBatteryHigh,1e2,&sendIndex);
	libBufferAppend_float16(buffer, modCANPackStateHandle->tempBatteryLow,1e2,&sendIndex);
	libBufferAppend_uint8(buffer, (uint8_t)modCANPackStateHandle->SoC,&sendIndex);
	libBufferAppend_uint8(buffer, modCANPackStateHandle->throttleDutyCharge/10,&sendIndex);
	libBufferAppend_uint8(buffer, modCANPackStateHandle->throttleDutyDischarge/10,&sendIndex);
	libBufferAppend_uint8(buffer,flagHolder,&sendIndex);
	modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_EBMS_STATUS_THROTTLE_CH_DISCH_BOOL), buffer, sendIndex);
}

void modCANSendStatusSlow(void) {
	int32_t sendIndex;
	uint8_t buffer[8];

	// Send voltage and current
	sendIndex = 0;
	libBufferAppend_float32(buffer, modCANPackStateHandle->packVoltage,1e5,&sendIndex);
	libBufferAppend_float32(buffer, modCANPackStateHandle->packCurrent,1e5,&sendIndex);
	modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_EBMS_STATUS_MAIN_IV), buffer, sendIndex);
	
	// Send highest and lowest cell voltage
	sendIndex = 0;
	libBufferAppend_float32(buffer, modCANPackStateHandle->cellVoltageLow,1e5,&sendIndex);
	libBufferAppend_float32(buffer, modCANPackStateHandle->cellVoltageHigh,1e5,&sendIndex);
	modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_EBMS_STATUS_CELLVOLTAGE), buffer, sendIndex);
}

void modCANSendStatusVESC(void){

		int32_t send_index = 0;
		uint8_t buffer[8];
		
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->packVoltage, &send_index);
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->chargerVoltage, &send_index);
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_V_TOT), buffer, send_index);

		send_index = 0;
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->packCurrent, &send_index);
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->packCurrent, &send_index);
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_I), buffer, send_index);

		send_index = 0;
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->packCurrent, &send_index); //To do : define AhCounter
		libBufferAppend_float32_auto(buffer, modCANPackStateHandle->packVoltage, &send_index); //To do : define WhCounter
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_AH_WH), buffer, send_index);
		
		uint8_t cellPointer = 0;
		uint8_t totalNoOfCells = modCANGeneralConfigHandle->noOfCellsSeries*modCANGeneralConfigHandle->noOfParallelModules;
		while(cellPointer < totalNoOfCells){
			send_index = 0;
			buffer[send_index++] = cellPointer;
			buffer[send_index++] = totalNoOfCells;

			if (cellPointer < totalNoOfCells) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->cellVoltagesIndividual[cellPointer++].cellVoltage, 1e3, &send_index);
			}
			if (cellPointer < totalNoOfCells) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->cellVoltagesIndividual[cellPointer++].cellVoltage, 1e3, &send_index);
			}
			if (cellPointer < totalNoOfCells) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->cellVoltagesIndividual[cellPointer++].cellVoltage, 1e3, &send_index);
			}
			modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_V_CELL), buffer, send_index);
		}

		send_index = 0;
		buffer[send_index++] = totalNoOfCells;
		uint64_t bal_state = 0;
		for (int i = 0; i < totalNoOfCells; i++) {
			bal_state |= (uint64_t)modCANPackStateHandle->cellVoltagesIndividual[i].cellBleedActive << i;
		}
		buffer[send_index++] = (bal_state >> 48) & 0xFF;
		buffer[send_index++] = (bal_state >> 40) & 0xFF;
		buffer[send_index++] = (bal_state >> 32) & 0xFF;
		buffer[send_index++] = (bal_state >> 24) & 0xFF;
		buffer[send_index++] = (bal_state >> 16) & 0xFF;
		buffer[send_index++] = (bal_state >> 8) & 0xFF;
		buffer[send_index++] = (bal_state >> 8) & 0xFF; //Maybe change?
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_BAL), buffer, send_index);

		uint8_t auxPointer = 0;
		uint8_t totalNoOfAux =modCANGeneralConfigHandle->cellMonitorICCount*modCANGeneralConfigHandle->noOfTempSensorPerModule;
		while (auxPointer < totalNoOfAux ) {
			send_index = 0;
			buffer[send_index++] = auxPointer;
			buffer[send_index++] = totalNoOfAux;
			if (auxPointer < totalNoOfAux) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->auxVoltagesIndividual[auxPointer++].auxVoltage, 1e2, &send_index);
			}
			if (auxPointer < totalNoOfAux) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->auxVoltagesIndividual[auxPointer++].auxVoltage, 1e2, &send_index);
			}
			if (auxPointer < totalNoOfAux) {
				libBufferAppend_float16(buffer, modCANPackStateHandle->auxVoltagesIndividual[auxPointer++].auxVoltage, 1e2, &send_index);
			}
			modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_TEMPS), buffer, send_index);
		}

		send_index = 0;
		libBufferAppend_float16(buffer, modCANPackStateHandle->temperatures[0], 1e2, &send_index);
		libBufferAppend_float16(buffer, modCANPackStateHandle->humidity, 1e2, &send_index);
		libBufferAppend_float16(buffer, modCANPackStateHandle->temperatures[1], 1e2, &send_index);
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_HUM), buffer, send_index);

		
		 //* CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
		 //*
		 //* b[0] - b[1]: V_CELL_MIN (mV)
		 //* b[2] - b[3]: V_CELL_MAX (mV)
		 //* b[4]: SoC (0 - 255)
		 //* b[5]: SoH (0 - 255)
		 //* b[6]: T_CELL_MAX (-128 to +127 degC)
		 //* b[7]: State bitfield:
		 //* [B7      B6      B5      B4      B3      B2      B1      B0      ]
		 //* [RSV     RSV     RSV     RSV     RSV     CHG_OK  IS_BAL  IS_CHG  ]
		 
		send_index = 0;
		libBufferAppend_float16(buffer, modCANPackStateHandle->cellVoltageLow, 1e3, &send_index);
		libBufferAppend_float16(buffer, modCANPackStateHandle->cellVoltageHigh, 1e3, &send_index);
		buffer[send_index++] = (uint8_t) ((modCANPackStateHandle->SoC/100) * 255.0);
		buffer[send_index++] = (uint8_t) (1 * 255.0);
		buffer[send_index++] = (int8_t) modCANPackStateHandle->tempBatteryHigh;
		buffer[send_index++] =
				(modCANPackStateHandle->chargeDesired << 0) | //To do: Define isCharging bool instead of chargeDesired...
				(modCANPackStateHandle->balanceActive << 1) |
				(modCANPackStateHandle->chargeAllowed << 2);
		modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_BMS_SOC_SOH_TEMP_STAT), buffer, send_index);


}

void CAN_RX0_IRQHandler(void) {
  HAL_CAN_IRQHandler(&modCANHandle);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *CanHandle) {
	// Handle CAN message	
	if((*CanHandle->pRxMsg).IDE == CAN_ID_STD) {         // Standard ID
		modCANHandleCANOpenMessage(*CanHandle->pRxMsg);
	}else{                                               // Extended ID
		if((*CanHandle->pRxMsg).ExtId == 0x0A23){
			modCANHandleKeepAliveSafetyMessage(*CanHandle->pRxMsg);
		}else{
			uint8_t destinationID = modCANGetDestinationID(*CanHandle->pRxMsg);
			if(destinationID == modCANGeneralConfigHandle->CANID){
				modCANRxFrames[modCANRxFrameWrite++] = *CanHandle->pRxMsg;
				if(modCANRxFrameWrite >= RX_CAN_FRAMES_SIZE) {
					modCANRxFrameWrite = 0;
				}
			}
		}
	}
	
  HAL_CAN_Receive_IT(&modCANHandle, CAN_FIFO0);
}

void modCANSubTaskHandleCommunication(void) {
	static int32_t ind = 0;
	static unsigned int rxbuf_len;
	static unsigned int rxbuf_ind;
	static uint8_t crc_low;
	static uint8_t crc_high;
	static bool commands_send;

	while(modCANRxFrameRead != modCANRxFrameWrite) {
		CanRxMsgTypeDef rxmsg = modCANRxFrames[modCANRxFrameRead++];

		if(rxmsg.IDE == CAN_ID_EXT) {
			uint8_t destinationID = modCANGetDestinationID(rxmsg);
			CAN_PACKET_ID cmd = modCANGetPacketID(rxmsg);

			if(destinationID == 255 || destinationID == modCANGeneralConfigHandle->CANID) {
				switch(cmd) {
					case CAN_PACKET_FILL_RX_BUFFER:
  					memcpy(modCANRxBuffer + rxmsg.Data[0], rxmsg.Data + 1, rxmsg.DLC - 1);
						break;

					case CAN_PACKET_FILL_RX_BUFFER_LONG:
						rxbuf_ind = (unsigned int)rxmsg.Data[0] << 8;
						rxbuf_ind |= rxmsg.Data[1];
						if(rxbuf_ind < RX_CAN_BUFFER_SIZE) {
							memcpy(modCANRxBuffer + rxbuf_ind, rxmsg.Data + 2, rxmsg.DLC - 2);
						}
						break;

					case CAN_PACKET_PROCESS_RX_BUFFER:
						ind = 0;
						modCANRxBufferLastID = rxmsg.Data[ind++];
						commands_send = rxmsg.Data[ind++];
						rxbuf_len = (unsigned int)rxmsg.Data[ind++] << 8;
						rxbuf_len |= (unsigned int)rxmsg.Data[ind++];

						if(rxbuf_len > RX_CAN_BUFFER_SIZE) {
							break;
						}

						crc_high = rxmsg.Data[ind++];
						crc_low = rxmsg.Data[ind++];

						if(libCRCCalcCRC16(modCANRxBuffer, rxbuf_len) == ((unsigned short) crc_high << 8 | (unsigned short) crc_low)) {
							
							if(commands_send) {
								modCommandsSendPacket(modCANRxBuffer, rxbuf_len);
							}else{
								modCommandsSetSendFunction(modCANSendPacketWrapper);
								modCommandsProcessPacket(modCANRxBuffer, rxbuf_len);
							}
						}
						break;

					case CAN_PACKET_PROCESS_SHORT_BUFFER:
						ind = 0;
						modCANRxBufferLastID = rxmsg.Data[ind++];
						commands_send = rxmsg.Data[ind++];
						
						if(commands_send) {
							modCommandsSendPacket(rxmsg.Data + ind, rxmsg.DLC - ind);
						}else{
							modCommandsSetSendFunction(modCANSendPacketWrapper);
							modCommandsProcessPacket(rxmsg.Data + ind, rxmsg.DLC - ind);
						}

					case CAN_PACKET_PING: {
						uint8_t buffer[2];
						buffer[0] = modCANGeneralConfigHandle->CANID;
						buffer[1] = HW_TYPE_VESC_BMS;
						modCANTransmitExtID(modCANGetCANID(modCANGeneralConfigHandle->CANID,CAN_PACKET_PONG), buffer, 2);
						} 
						break;/*

					case CAN_PACKET_PONG:
						// data8[0]; // Sender ID
						if (ping_tp) {
							if (len >= 2) {
								ping_hw_last = data8[1];
							} else {
								ping_hw_last = HW_TYPE_VESC_BMS;
							}
							chEvtSignal(ping_tp, 1 << 29);
						}
						break;*/

					case CAN_PACKET_SHUTDOWN: {
						// TODO: Implement when hw has power switch
						} break;
					default:
						break;
					}
				}

				switch (cmd) {
				case CAN_PACKET_PING:
					//sleep_reset();
					break;

				case CAN_PACKET_STATUS:
					//sleep_reset();

					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *stat_tmp = &stat_msgs[i];
						if (stat_tmp->id == destinationID || stat_tmp->id == -1) {
							ind = 0;
							stat_tmp->id = destinationID;
							stat_tmp->rx_time = HAL_GetTick();
							stat_tmp->rpm = (float)libBufferGet_int32(rxmsg.Data, &ind);
							stat_tmp->current = (float)libBufferGet_int16(rxmsg.Data, &ind) / 10.0;
							stat_tmp->duty = (float)libBufferGet_int16(rxmsg.Data, &ind) / 1000.0;
							break;
						}
					}
					break;

				case CAN_PACKET_STATUS_2:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg_2 *stat_tmp_2 = &stat_msgs_2[i];
						if (stat_tmp_2->id == destinationID || stat_tmp_2->id == -1) {
							ind = 0;
							stat_tmp_2->id = destinationID;
							stat_tmp_2->rx_time = HAL_GetTick();
							stat_tmp_2->amp_hours = (float)libBufferGet_int32(rxmsg.Data, &ind) / 1e4;
							stat_tmp_2->amp_hours_charged = (float)libBufferGet_int32(rxmsg.Data, &ind) / 1e4;
							break;
						}
					}
					break;

				case CAN_PACKET_STATUS_3:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg_3 *stat_tmp_3 = &stat_msgs_3[i];
						if (stat_tmp_3->id == destinationID || stat_tmp_3->id == -1) {
							ind = 0;
							stat_tmp_3->id = destinationID;
							stat_tmp_3->rx_time = HAL_GetTick();
							stat_tmp_3->watt_hours = (float)libBufferGet_int32(rxmsg.Data, &ind) / 1e4;
							stat_tmp_3->watt_hours_charged = (float)libBufferGet_int32(rxmsg.Data, &ind) / 1e4;
							break;
						}
					}
					break;

				case CAN_PACKET_STATUS_4:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg_4 *stat_tmp_4 = &stat_msgs_4[i];
						if (stat_tmp_4->id == destinationID || stat_tmp_4->id == -1) {
							ind = 0;
							stat_tmp_4->id = destinationID;
							stat_tmp_4->rx_time = HAL_GetTick();
							stat_tmp_4->temp_fet = (float)libBufferGet_int16(rxmsg.Data, &ind) / 10.0;
							stat_tmp_4->temp_motor = (float)libBufferGet_int16(rxmsg.Data, &ind) / 10.0;
							stat_tmp_4->current_in = (float)libBufferGet_int16(rxmsg.Data, &ind) / 10.0;
							stat_tmp_4->pid_pos_now = (float)libBufferGet_int16(rxmsg.Data, &ind) / 50.0;
							break;
						}
					}
					break;

				case CAN_PACKET_STATUS_5:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg_5 *stat_tmp_5 = &stat_msgs_5[i];
						if (stat_tmp_5->id == destinationID || stat_tmp_5->id == -1) {
							ind = 0;
							stat_tmp_5->id = destinationID;
							stat_tmp_5->rx_time = HAL_GetTick();
							stat_tmp_5->tacho_value = libBufferGet_int32(rxmsg.Data, &ind);
							stat_tmp_5->v_in = (float)libBufferGet_int16(rxmsg.Data, &ind) / 1e1;
							break;
						}
					}
					break;
/*
				case CAN_PACKET_BMS_SOC_SOH_TEMP_STAT: {
					int32_t ind = 0;
					bms_soc_soh_temp_stat msg;
					msg.id = id;
					msg.rx_time = chVTGetSystemTime();
					msg.v_cell_min = buffer_get_float16(data8, 1e3, &ind);
					msg.v_cell_max = buffer_get_float16(data8, 1e3, &ind);
					msg.soc = ((float)((uint8_t)data8[ind++])) / 255.0;
					msg.soh = ((float)((uint8_t)data8[ind++])) / 255.0;
					msg.t_cell_max = (float)((int8_t)data8[ind++]);
					uint8_t stat = data8[ind++];
					msg.is_charging = (stat >> 0) & 1;
					msg.is_balancing = (stat >> 1) & 1;
					msg.is_charge_allowed = (stat >> 2) & 1;

					// Do not go to sleep when some other pack is charging or balancing.
					if (msg.is_charging || msg.is_balancing) {
						sleep_reset();
					}

					// Find BMS with lowest cell voltage
					if (bms_stat_v_cell_min.id < 0 ||
							UTILS_AGE_S(bms_stat_v_cell_min.rx_time) > 10.0 ||
							bms_stat_v_cell_min.v_cell_min > msg.v_cell_min) {
						bms_stat_v_cell_min = msg;
					} else if (bms_stat_v_cell_min.id == msg.id) {
						bms_stat_v_cell_min = msg;
					}

					for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
						bms_soc_soh_temp_stat *msg_buf = &bms_stat_msgs[i];

						// Reset ID after 10 minutes of silence
						if (msg_buf->id != -1 && UTILS_AGE_S(msg_buf->rx_time) > 60 * 10) {
							msg_buf->id = -1;
						}

						if (msg_buf->id == id || msg_buf->id == -1) {
							*msg_buf = msg;
							break;
						}
					}
					} break;

				case CAN_PACKET_PSW_STAT: {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						psw_status *msg = &psw_stat[i];
						if (msg->id == id || msg->id == -1) {
							ind = 0;
							msg->id = id;
							msg->rx_time = chVTGetSystemTime();

							msg->v_in = buffer_get_float16(data8, 10.0, &ind);
							msg->v_out = buffer_get_float16(data8, 10.0, &ind);
							msg->temp = buffer_get_float16(data8, 10.0, &ind);
							msg->is_out_on = (data8[ind] >> 0) & 1;
							msg->is_pch_on = (data8[ind] >> 1) & 1;
							msg->is_dsc_on = (data8[ind] >> 2) & 1;
							ind++;
							break;
						}
					}
				} break;
				*/
				default:
					break;
				}

			
		}

		if(modCANRxFrameRead >= RX_CAN_FRAMES_SIZE)
			modCANRxFrameRead = 0;
	}
}

void modCANTransmitExtID(uint32_t id, uint8_t *data, uint8_t len) {
	CanTxMsgTypeDef txmsg;
	txmsg.IDE = CAN_ID_EXT;
	txmsg.ExtId = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.Data, data, len);
	
	modCANHandle.pTxMsg = &txmsg;
	HAL_CAN_Transmit(&modCANHandle,1);
}

void modCANTransmitStandardID(uint32_t id, uint8_t *data, uint8_t len) {
	CanTxMsgTypeDef txmsg;
	txmsg.IDE = CAN_ID_STD;
	txmsg.StdId = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.Data, data, len);
	
	modCANHandle.pTxMsg = &txmsg;
	HAL_CAN_Transmit(&modCANHandle,1);
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * If true, this packet will be passed to the send function of commands.
 * Otherwise, it will be passed to the process function.
 */
void modCANSendBuffer(uint8_t controllerID, uint8_t *data, unsigned int len, bool send) {
	uint8_t send_buffer[8];

	if(len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = modCANGeneralConfigHandle->CANID;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_PROCESS_SHORT_BUFFER), send_buffer, ind);
	}else{
		unsigned int end_a = 0;
		for(unsigned int i = 0;i < len;i += 7) {
			if(i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			}else{
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_FILL_RX_BUFFER), send_buffer, send_len + 1);
		}

		for(unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			}else{
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_FILL_RX_BUFFER_LONG), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = modCANGeneralConfigHandle->CANID;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = libCRCCalcCRC16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);
    
		// Old ID method
		//modCANTransmitExtID(controllerID | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
		modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_PROCESS_RX_BUFFER), send_buffer, ind++);
	}
}

void modCANSetESCDuty(uint8_t controllerID, float duty) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_int32(buffer, (int32_t)(duty * 100000.0f), &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_DUTY), buffer, sendIndex);
}

void modCANSetESCCurrent(uint8_t controllerID, float current) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_int32(buffer, (int32_t)(current * 1000.0f), &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_CURRENT), buffer, sendIndex);
}

void modCANSetESCBrakeCurrent(uint8_t controllerID, float current) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_int32(buffer, (int32_t)(current * 1000.0f), &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_CURRENT_BRAKE), buffer, sendIndex);
}

void modCANSetESCRPM(uint8_t controllerID, float rpm) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_int32(buffer, (int32_t)rpm, &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_RPM), buffer, sendIndex);
}

void modCANSetESCPosition(uint8_t controllerID, float pos) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_int32(buffer, (int32_t)(pos * 1000000.0f), &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_POS), buffer, sendIndex);
}

void modCANSetESCCurrentRelative(uint8_t controllerID, float currentRel) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_float32(buffer, currentRel, 1e5, &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_CURRENT_REL), buffer, sendIndex);
}

void modCANSetESCBrakeCurrentRelative(uint8_t controllerID, float currentRel) {
	int32_t sendIndex = 0;
	uint8_t buffer[4];
	libBufferAppend_float32(buffer, currentRel, 1e5, &sendIndex);
	modCANTransmitExtID(modCANGetCANID(controllerID,CAN_PACKET_SET_CURRENT_BRAKE_REL), buffer, sendIndex);
}

static void modCANSendPacketWrapper(unsigned char *data, unsigned int length) {
	modCANSendBuffer(modCANRxBufferLastID, data, length, true);
}

void modCANHandleKeepAliveSafetyMessage(CanRxMsgTypeDef canMsg) {
	if(canMsg.DLC >= 1){
		if(canMsg.Data[0] & 0x01){
			modCANSafetyCANMessageTimeout = HAL_GetTick();
			modCANPackStateHandle->safetyOverCANHCSafeNSafe = (canMsg.Data[0] & 0x02) ? true : false;
		}
		
		if(canMsg.Data[0] & 0x04){
				modCANPackStateHandle->watchDogTime = (canMsg.Data[0] & 0x08) ? 255 : 0;
		}
	}
	
	if(canMsg.DLC >= 2){
		if(canMsg.Data[1] & 0x10){
			modCANPackStateHandle->chargeBalanceActive = modCANGeneralConfigHandle->allowChargingDuringDischarge;
			modPowerElectronicsResetBalanceModeActiveTimeout();
		}
	}
}

void modCANHandleCANOpenMessage(CanRxMsgTypeDef canMsg) {
  if(canMsg.StdId == 0x070A){
		modCANLastChargerHeartBeatTick = HAL_GetTick();
		modCANChargerCANOpenState = canMsg.Data[0];
	}else if(canMsg.StdId == 0x048A){
	  modCANChargerChargingState = canMsg.Data[5];
	}
}

void modCANHandleSubTaskCharger(void) {
  //static uint8_t chargerOpState = opInit;
  //static uint8_t chargerOpStateNew = opInit;
	
  if(modDelayTick1ms(&modCANChargerTaskIntervalLastTick, 500)) {
		// Check charger present
		modCANOpenChargerCheckPresent();
		
		if(modCANChargerPresentOnBus) {
		  // Send HeartBeat from bms
			modCANOpenBMSSendHeartBeat();
		
			// Manage operational state and start network
			if(modCANChargerCANOpenState != 0x05)
				modCANOpenChargerStartNode();
			
			if(modCANChargerCANOpenState == 0x05) {
				switch(chargerOpState) {
					case opInit:
						if(modCANPackStateHandle->powerDownDesired) {
						  modCANOpenChargerSetCurrentVoltageReady(0.0f,0.0f,false);
						}else{
						  chargerOpStateNew = opChargerReset;
						}
						break;
					case opChargerReset:
						modCANOpenChargerSetCurrentVoltageReady(0.0f,0.0f,false);
					  chargerOpStateNew = opChargerSet;
						break;
					case opChargerSet:
						modCANOpenChargerSetCurrentVoltageReady(0.0f,0.0f,true);
					  chargerOpStateNew = opCharging;
						break;
					case opCharging:
						modCANOpenChargerSetCurrentVoltageReady(30.0f*modCANPackStateHandle->throttleDutyCharge/1000,modCANGeneralConfigHandle->noOfCellsSeries*modCANGeneralConfigHandle->cellSoftOverVoltage+0.6f,true);
					
					  if(modCANPackStateHandle->powerDownDesired)
					    chargerOpStateNew = opInit;

						break;
					default:
						chargerOpStateNew = opInit;
				}
				
				chargerOpState = chargerOpStateNew;
				
			  modCANPackStateHandle->chargeBalanceActive = modCANGeneralConfigHandle->allowChargingDuringDischarge;
			  modPowerElectronicsResetBalanceModeActiveTimeout();
		  }
			
	  }else{
		  chargerOpState = opInit;
		}
	}
}

void modCANRXWatchDog(void){
  if(modCANHandle.pRxMsg->ExtId != modCANLastRXID){
	  modCANLastRXID = modCANHandle.pRxMsg->ExtId;
		modCANLastRXDifferLastTick = HAL_GetTick();
	}
	
	if(modDelayTick1ms(&modCANLastRXDifferLastTick,1000)){
		modCANInit(modCANPackStateHandle,modCANGeneralConfigHandle);
	}
}

void modCANOpenChargerCheckPresent(void) {
	if((HAL_GetTick() - modCANLastChargerHeartBeatTick) < 2000)
		modCANChargerPresentOnBus = true;
	else
		modCANChargerPresentOnBus = true; //Change this back to false when charger is not present
}

void modCANOpenBMSSendHeartBeat(void) {
  // Send the canopen heartbeat from the BMS
	int32_t sendIndex = 0;
	//uint8_t operationalState = 5;
	uint8_t buffer[8];
	//libBufferAppend_uint8(buffer, operationalState, &sendIndex);
	//modCANTransmitStandardID(0x0701, buffer, sendIndex);
	libBufferAppend_uint16(buffer, modCANGeneralConfigHandle->MaxChargingVoltage*10, &sendIndex);
	libBufferAppend_uint16(buffer, modCANGeneralConfigHandle->MaxChargingCurrent*10, &sendIndex);
	libBufferAppend_uint8(buffer, modCANGeneralConfigHandle->ChargingDisallowed, &sendIndex);
	libBufferAppend_uint8(buffer,0x00 , &sendIndex);
	libBufferAppend_uint16(buffer, 0x0000, &sendIndex);
	modCANTransmitExtID(0x1806E5F4, buffer, sendIndex);

}

void modCANOpenChargerStartNode(void) {
  // Send the canopen heartbeat from the BMS
	int32_t sendIndex = 0;
	uint8_t buffer[2];
	libBufferAppend_uint8(buffer, 0x01, &sendIndex);
	libBufferAppend_uint8(buffer, 0x0A, &sendIndex);	
	modCANTransmitStandardID(0x0000, buffer, sendIndex);
}

void modCANOpenChargerSetCurrentVoltageReady(float current,float voltage,bool ready) {
	uint32_t modCANChargerRequestVoltageInt = voltage * 1024;
	uint16_t modCANChargerRequestCurrentInt = current * 16;
	
	int32_t sendIndex = 0;
	uint8_t buffer[8];
	libBufferAppend_uint16_LSBFirst(buffer, modCANChargerRequestCurrentInt, &sendIndex);
	libBufferAppend_uint8(buffer, ready, &sendIndex);	
	libBufferAppend_uint32_LSBFirst(buffer, modCANChargerRequestVoltageInt, &sendIndex);		
	modCANTransmitStandardID(0x040A, buffer, sendIndex);
}

uint16_t modCANGetVESCCurrent(void){
	//return stat_tmp->current;
}



can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

can_status_msg_2 *comm_can_get_status_msg_2_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_2[index];
	} else {
		return 0;
	}
}

can_status_msg_2 *comm_can_get_status_msg_2_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_2[i].id == id) {
			return &stat_msgs_2[i];
		}
	}

	return 0;
}

can_status_msg_3 *comm_can_get_status_msg_3_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_3[index];
	} else {
		return 0;
	}
}

can_status_msg_3 *comm_can_get_status_msg_3_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_3[i].id == id) {
			return &stat_msgs_3[i];
		}
	}

	return 0;
}

can_status_msg_4 *comm_can_get_status_msg_4_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_4[index];
	} else {
		return 0;
	}
}

can_status_msg_4 *comm_can_get_status_msg_4_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_4[i].id == id) {
			return &stat_msgs_4[i];
		}
	}

	return 0;
}

can_status_msg_5 *comm_can_get_status_msg_5_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs_5[index];
	} else {
		return 0;
	}
}

can_status_msg_5 *comm_can_get_status_msg_5_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs_5[i].id == id) {
			return &stat_msgs_5[i];
		}
	}

	return 0;
}
/*
bms_soc_soh_temp_stat *comm_can_get_bms_soc_soh_temp_stat_index(int index) {
	if (index < CAN_BMS_STATUS_MSGS_TO_STORE) {
		return &bms_stat_msgs[index];
	} else {
		return 0;
	}
}

bms_soc_soh_temp_stat *comm_can_get_bms_soc_soh_temp_stat_id(int id) {
	for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
		if (bms_stat_msgs[i].id == id) {
			return &bms_stat_msgs[i];
		}
	}

	return 0;
}

bms_soc_soh_temp_stat *comm_can_get_bms_stat_v_cell_min(void) {
	return &bms_stat_v_cell_min;
}

psw_status *comm_can_get_psw_status_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &psw_stat[index];
	} else {
		return 0;
	}
}

psw_status *comm_can_get_psw_status_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (psw_stat[i].id == id) {
			return &psw_stat[i];
		}
	}

	return 0;
}

void comm_can_psw_switch(int id, bool is_on, bool plot) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	buffer[send_index++] = is_on ? 1 : 0;
	buffer[send_index++] = plot ? 1 : 0;

	comm_can_transmit_eid(id | ((uint32_t)CAN_PACKET_PSW_SWITCH << 8),
			buffer, send_index);
}
*/


