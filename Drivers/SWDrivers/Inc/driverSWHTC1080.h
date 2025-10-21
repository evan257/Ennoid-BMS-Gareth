#ifndef __DRIVERSWHTC1080_H
#define __DRIVERSWHTC1080_H

#include "stdbool.h"
#include "modDelay.h"
#include "driverHWI2C2.h"


#define DRIVERHTC1080_ADDR   0x40
#define DRIVERHTC1080_TRIGGER_T_MEASUREMENT 0X00   	// temperature
#define DRIVERHTC1080_TRIGGER_RH_MEASUREMENT 0X01  	// humidity
#define DRIVERHTC1080_CONFIGURATION 0X03  			// configuration register 


void driverSWHTC1080Init(void);
void driverSWHTC1080StartMeasurement(void);
bool driverSWHTC1080PollMeasureReady(void);
float driverSWHTC1080CalculateTemperature(uint16_t rawTemperature);
float driverSWHTC1080GetTemperature(void);
float driverSWHTC1080CalculateHumidity(uint16_t rawHumidity);
float driverSWHTC1080GetHumidity(void);

#endif
