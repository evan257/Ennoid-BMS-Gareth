#ifndef __DRIVERSWINA226_H
#define __DRIVERSWINA226_H

#include "driverHWI2C2.h"
#include "driverHWI2C1.h"

#define INA226_MASTER_BUS         2

typedef enum {
	REG_CONFIG = 0,
	REG_VSHUNT,
	REG_VBUS,
	REG_POWER,
	REG_CURRENT,
	REG_CAL,
	REG_MASK,
	REG_LIM,
	REG_MAN_ID = 254,
	REG_DIE_ID
} driverSWINA226Registers;

void driverSWINA226Init(uint8_t i2cAddres, uint8_t i2cBus, driverSWINA226InitStruct initStruct);
bool driverSWINA226GetBusCurrent(uint8_t i2cAddres, uint8_t i2cBus, float *busCurrent, int16_t offset, float scalar);
bool driverSWINA226GetBusVoltage(uint8_t i2cAddres, uint8_t i2cBus, float *busVoltage, int16_t offset, float scalar);

#endif
