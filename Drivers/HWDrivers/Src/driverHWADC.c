#include "driverHWADC.h"

// Example: http://visualgdb.com/tutorials/arm/stm32/adc/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3; //Added by Gareth for current sensor ADC

const driverHWADCPortStruct driverHWADCPorts[NoOfADCPorts] = 								// Hold all I2C pin configuration data
{
	{GPIOA,RCC_AHBENR_GPIOAEN,GPIO_PIN_1,GPIO_MODE_ANALOG,GPIO_NOPULL,0x00},	// LoadVoltageSense analog pin
	{GPIOA,RCC_AHBENR_GPIOAEN,GPIO_PIN_0,GPIO_MODE_ANALOG,GPIO_PULLUP,0x00},		// ChargeVoltage analog pin
	{GPIOB,RCC_AHBENR_GPIOBEN,GPIO_PIN_0,GPIO_MODE_ANALOG,GPIO_NOPULL,0x00}	// Current Sensor Input

};

void driverHWADCInit(void) {
	ADC_MultiModeTypeDef multimode = {0};
	GPIO_InitTypeDef PortInitHolder;
	uint8_t PortPointer;
	
	for(PortPointer = 0; PortPointer < NoOfADCPorts; PortPointer++) {
		RCC->AHBENR |= driverHWADCPorts[PortPointer].ClkRegister;								// Enable clock de desired port
		PortInitHolder.Mode = driverHWADCPorts[PortPointer].Mode;								// Push pull output
		PortInitHolder.Pin = driverHWADCPorts[PortPointer].Pin;									// Points to status pin
		PortInitHolder.Pull = driverHWADCPorts[PortPointer].Pull;								// No pullup
		PortInitHolder.Speed = GPIO_SPEED_HIGH;																	// GPIO clock speed
		PortInitHolder.Alternate = driverHWADCPorts[PortPointer].Alternate;			// Alternate function
		HAL_GPIO_Init(driverHWADCPorts[PortPointer].Port,&PortInitHolder);			// Perform the IO init 
	};
	
	__ADC1_CLK_ENABLE();															// Enable clock to ADC1
	__HAL_RCC_ADC34_CLK_ENABLE();													// Enable clock for ADC3-4															
	//__ADC3_CLK_ENABLE();	
	// ADC config
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    while(true) {}; 																												// Error situation 
  }

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    while(true) {};
  }
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    while(true) {}; 
  }
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

	driverHWADCSetInputChannel(&hadc1,ADC_CHANNEL_2);
};

void driverHWADCSetInputChannel(ADC_HandleTypeDef* hadc, uint32_t inputChannel) {
  ADC_ChannelConfTypeDef sConfig;
	
  sConfig.Channel = inputChannel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    while(true) {}; 																												// Error situation 
  }
}

bool driverHWADCGetLoadVoltage(float *loCurrentLoadVoltage, float offset, float scalar) {
	uint32_t driverHWADCAverageSum = 0;
	uint8_t	driverHWADCAverageCount = 0;
	
	driverHWADCSetInputChannel(&hadc1,ADC_CHANNEL_2);

	driverHWADCAverageSum = 0;
	for(driverHWADCAverageCount = 0; driverHWADCAverageCount < NoOfAverages; driverHWADCAverageCount++) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			driverHWADCAverageSum += HAL_ADC_GetValue(&hadc1);
		};
	};
	
	uint16_t temp = driverHWADCAverageSum/NoOfAverages;
	*loCurrentLoadVoltage = temp*(3.3f/4096*scalar)+offset;

	return false;
};

bool driverHWADCGetChargerVoltage(float *chargerVoltage, float offset, float scalar) {
	uint32_t driverHWADCAverageSum = 0;
	uint8_t	driverHWADCAverageCount = 0;
	
	driverHWADCSetInputChannel(&hadc1,ADC_CHANNEL_1);

	driverHWADCAverageSum = 0;
	for(driverHWADCAverageCount = 0; driverHWADCAverageCount < NoOfAverages; driverHWADCAverageCount++) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			driverHWADCAverageSum += HAL_ADC_GetValue(&hadc1);
		};
	};
	
	uint16_t temp = driverHWADCAverageSum/NoOfAverages;
	*chargerVoltage = temp*(3.3f/4096*scalar)+offset;

	return false;
};

bool driverHWADCGetNTCValue(float *ntcValue, uint32_t ntcNominal, uint32_t ntcSeriesResistance, uint16_t ntcBetaFactor, float ntcNominalTemp) {
	uint32_t driverHWADCAverageSum;
	uint8_t	 driverHWADCAverageCount;
	uint16_t driverHWADCAverage;
	
	driverHWADCSetInputChannel(&hadc1,ADC_CHANNEL_1);

	driverHWADCAverageSum = 0;
	for(driverHWADCAverageCount = 0; driverHWADCAverageCount < 5; driverHWADCAverageCount++) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			driverHWADCAverageSum += HAL_ADC_GetValue(&hadc1);
		};
	};
	
	driverHWADCAverage = driverHWADCAverageSum/5;
	
	static float scalar;
	static float steinhart;
	
  scalar = 4095.0f / (float)driverHWADCAverage - 1.0f;
  scalar = (float)ntcSeriesResistance / scalar;
  steinhart = scalar / (float)ntcNominal;               // (R/Ro)
  steinhart = log(steinhart);                           // ln(R/Ro)
  steinhart /= (float)ntcBetaFactor;                    // 1/B * ln(R/Ro)
  steinhart += 1.0f / (ntcNominalTemp + 273.15f);       // + (1/To)
  steinhart = 1.0f / steinhart;                         // Invert
  *ntcValue = steinhart - 273.15f;                      // convert to degree


	return false;
};

void driverHWADCGetPackCurrent(float *packCurrent) {
	uint32_t driverHWADCAverageSum = 0;
	uint8_t	driverHWADCAverageCount = 0;
	
	driverHWADCSetInputChannel(&hadc3,ADC_CHANNEL_12);

	driverHWADCAverageSum = 0;
	for(driverHWADCAverageCount = 0; driverHWADCAverageCount < NoOfAverages; driverHWADCAverageCount++) {
		HAL_ADC_Start(&hadc3);
		if (HAL_ADC_PollForConversion(&hadc3, 1000) == HAL_OK) {
			driverHWADCAverageSum += HAL_ADC_GetValue(&hadc3);
		};
	};

	uint16_t temp = driverHWADCAverageSum/NoOfAverages;
	volatile float PackCurrent = (((3.3f*((float)temp/4096.0f))/(2.0f/3.0f))-2.5f)/0.005f;

	*packCurrent = PackCurrent;

	return;

};
