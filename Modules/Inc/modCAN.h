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


#ifndef MODCAN_H_
#define MODCAN_H_

#include "generalDefines.h"
#include "mainDataTypes.h"
#include "stm32f3xx_hal.h"
#include "stdbool.h"
#include "modDelay.h"
#include "modCommands.h"
#include "libCRC.h"
#include "libPacket.h"
#include "libBuffer.h"
#include "modPowerElectronics.h"
#include "modConfig.h"
#include <string.h>
#include <math.h>

// Settings
#define CAN_STATUS_MSG_INT_MS			1
#define RX_CAN_FRAMES_SIZE	    		255       // max 255
#define RX_CAN_BUFFER_SIZE	    		PACKET_MAX_PL_LEN

// Settings
#define CAN_STATUS_MSGS_TO_STORE		10
#define CAN_BMS_STATUS_MSGS_TO_STORE		185

void		modCANInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer);
void		modCANTask(void);
uint32_t	modCANGetDestinationID(CanRxMsgTypeDef canMsg);
CAN_PACKET_ID	modCANGetPacketID(CanRxMsgTypeDef canMsg);
uint32_t	modCANGetCANID(uint32_t destinationID, CAN_PACKET_ID packetID);
void		modCANSendStatusFast(void);
void		modCANSendStatusSlow(void);
void		modCANSendStatusVESC(void);
void		modCANSubTaskHandleCommunication(void);
void		modCANTransmitExtID(uint32_t id, uint8_t *data, uint8_t len);
void		modCANTransmitStandardID(uint32_t id, uint8_t *data, uint8_t len);
void		modCANSendBuffer(uint8_t controllerID, uint8_t *data, unsigned int len, bool send);
void		modCANSetESCDuty(uint8_t controllerID, float duty);
void		modCANSetESCCurrent(uint8_t controllerID, float current);
void		modCANSetESCBrakeCurrent(uint8_t controllerID, float current);
void		modCANSetESCRPM(uint8_t controllerID, float rpm);
void		modCANSetESCPosition(uint8_t controllerID, float pos);
void		modCANSetESCCurrentRelative(uint8_t controllerID, float currentRel);
void		modCANSetESCBrakeCurrentRelative(uint8_t controllerID, float currentRel);
static void	modCANSendPacketWrapper(unsigned char *data, unsigned int len);
void		modCANHandleKeepAliveSafetyMessage(CanRxMsgTypeDef canMsg);
void		modCANHandleCANOpenMessage(CanRxMsgTypeDef canMsg);
void		modCANHandleSubTaskCharger(void);
void		modCANRXWatchDog(void);
void		modCANOpenChargerCheckPresent(void);
void		modCANOpenBMSSendHeartBeat(void);
void		modCANOpenChargerStartNode(void);
void		modCANOpenChargerSetCurrentVoltageReady(float current,float voltage,bool ready);

uint16_t 	modCANGetVESCCurrent(void);

can_status_msg *comm_can_get_status_msg_index(int index);
can_status_msg *comm_can_get_status_msg_id(int id);
can_status_msg_2 *comm_can_get_status_msg_2_index(int index);
can_status_msg_2 *comm_can_get_status_msg_2_id(int id);
can_status_msg_3 *comm_can_get_status_msg_3_index(int index);
can_status_msg_3 *comm_can_get_status_msg_3_id(int id);
can_status_msg_4 *comm_can_get_status_msg_4_index(int index);
can_status_msg_4 *comm_can_get_status_msg_4_id(int id);
can_status_msg_5 *comm_can_get_status_msg_5_index(int index);
can_status_msg_5 *comm_can_get_status_msg_5_id(int id);



#endif /* MODCAN_H_ */
