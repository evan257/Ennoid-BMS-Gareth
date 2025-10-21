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


#include "modDelay.h"
#include "driverHWStatus.h"

// Public functions:
void modEffectInit(void);
void modEffectTask(void);
void modEffectChangeState(STATIDTypedef id, STATStateTypedef status);
void modEffectChangeStateError(STATIDTypedef id, STATStateTypedef status, uint8_t errorCode);

// Private functions:
STATStateTypedef modEffectTaskFlash(void);
STATStateTypedef modEffectTaskFlashFast(void);
STATStateTypedef modEffectTaskBlinkShort(uint32_t LEDPointer, uint32_t blinkTime);
STATStateTypedef modEffectTaskBlinkLong(uint32_t LEDPointer, uint32_t blinkTime);
STATStateTypedef modEffectTaskBlinkShortLong(uint32_t blinkTimeShort, uint32_t blinkRatio);
STATStateTypedef modEffectTaskError(uint32_t blinkTime, uint32_t blinkRatio, uint32_t LEDPointer);
