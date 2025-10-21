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


#ifndef __MODMESSAGE_H
#define __MODMESSAGE_H

#include "stdint.h"
#include "stdbool.h"
#include "driverSWUART2.h"
#include "modDelay.h"
#include "modConfig.h"
#include "modPowerElectronics.h"

#include "libPacket.h"
#include "modCommands.h"

#define PACKET_HANDLER_UART		1

typedef enum {
	MESSAGE_DEBUG = 0,
	MESSAGE_ERROR,
	MESSAGE_NORMAL,
} modMessageMessageTypeDef;

void modUARTInit(void);
void modUARTTask(void);
void modUARTSendPacketWrapper(unsigned char *data, unsigned int len);
void modUARTProcessPacket(unsigned char *data, unsigned int len);
void modUARTSendPacket(unsigned char *data, unsigned int len);

#endif
