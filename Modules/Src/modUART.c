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
 
#include "modUART.h"

uint32_t consoleStatusLastTick;

void modUARTInit(void) {	
	libPacketInit(modUARTSendPacket, modUARTProcessPacket, PACKET_HANDLER_UART);
	
	driverSWUART2Init(115200);																								// Configure the UART driver
};

void modUARTTask(void) {
	char inputChar;
	
	while(driverSWUART2Task()){};
		
	if(driverHWUART2GetChar(&inputChar))
		libPacketProcessByte(inputChar,PACKET_HANDLER_UART);
};

void modUARTSendPacketWrapper(unsigned char *data, unsigned int len) {
	libPacketSendPacket(data, len, PACKET_HANDLER_UART);
}

void modUARTProcessPacket(unsigned char *data, unsigned int len) {
	modCommandsSetSendFunction(modUARTSendPacketWrapper);
	modCommandsProcessPacket(data, len);
}

void modUARTSendPacket(unsigned char *data, unsigned int len) {
	uint32_t outputCharPointer;
	
	for(outputCharPointer = 0; outputCharPointer < len; outputCharPointer++) {
	  driverHWUART2SendChar(data[outputCharPointer]);
	}
}
