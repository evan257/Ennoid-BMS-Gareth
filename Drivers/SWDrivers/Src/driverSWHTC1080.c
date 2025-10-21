#include "driverSWHTC1080.h"

uint32_t driverSWHTC1080LastMeasureStart;
float driverSWHTC1080LastTemperature;
float driverSWHTC1080LastHumidity;

void driverSWHTC1080Init(void) {
	
	// Wait for HTC1080 init
	uint32_t driverSWHTC1080StartupDelayLastTick = HAL_GetTick();
	while(!modDelayTick1ms(&driverSWHTC1080StartupDelayLastTick,20)) {}; // wait
		
	uint16_t conf = 0;
	conf |= (1 << 12); // Read temperature and humidity 14bits
	uint8_t configBytes[] = {DRIVERHTC1080_CONFIGURATION,conf};
	
	// Init measure
	driverSWHTC1080LastMeasureStart = HAL_GetTick();
	
	driverSWHTC1080LastTemperature = 0.0f;
	driverSWHTC1080LastHumidity = 0.0f;
	driverHWI2C2Write(DRIVERHTC1080_ADDR,false, configBytes,sizeof(configBytes));
	
}

void driverSWHTC1080StartMeasurement(void) {
  driverSWHTC1080LastMeasureStart = HAL_GetTick();
	
	uint8_t commandBytes[] = {DRIVERHTC1080_TRIGGER_T_MEASUREMENT};									// make command package, init with temp but that will be changed later		 
	driverHWI2C2Write(DRIVERHTC1080_ADDR,false,commandBytes,sizeof(commandBytes));
	
}

bool driverSWHTC1080PollMeasureReady(void) {
	bool returnValue = false;
	uint16_t returnedSensorValueHum;
	uint16_t returnedSensorValueTemp;
	uint8_t returnedData[4];
	
	if(modDelayTick1ms(&driverSWHTC1080LastMeasureStart,50)){		
		if(driverHWI2C2Read(DRIVERHTC1080_ADDR,returnedData,4) == HAL_OK){								// Try to read data from sensor
				returnedSensorValueHum = (returnedData[2] << 8) | returnedData[3];							// Compose ADC value
				returnedSensorValueTemp = (returnedData[0] << 8) | returnedData[1];							// Compose ADC value
				driverSWHTC1080LastTemperature = driverSWHTC1080CalculateTemperature(returnedSensorValueTemp); // Calculate temperature
				driverSWHTC1080LastHumidity = driverSWHTC1080CalculateHumidity(returnedSensorValueHum); // Calculate humidity				
				returnValue = true;
		}
	}
	
	return returnValue;
}

float driverSWHTC1080CalculateTemperature(uint16_t rawTemperature) {
  float temperatureC;                                                  // variable for result
  rawTemperature &= ~0x0003;                                           // clear bits [1..0] (status bits)
  temperatureC= -40.0 + 165.0/65536 *(float)rawTemperature;          //T= -40 + 165 * ST/2^16
  return temperatureC;
}

float driverSWHTC1080GetTemperature(void) {
	return driverSWHTC1080LastTemperature;
}

float driverSWHTC1080CalculateHumidity(uint16_t rawHumidity) {
  float humidityRH;                                                    // variable for result
  rawHumidity &= ~0x0003;                                              // clear bits [1..0] (status bits)
  humidityRH = 100.0/65536 * (float)rawHumidity;                // RH= 100 * SRH/2^16
  return humidityRH;
}

float driverSWHTC1080GetHumidity(void) {
	return driverSWHTC1080LastHumidity;
}
