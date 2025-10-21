#include "report_status.h"

#include "stm32f3xx_hal.h"

#include "safety_check.h"
#include "modCAN.h"
#include "sre_dbc.h"

static modPowerElectronicsPackStateTypedef *pack_state;

int32_t data_length = 0; 
uint8_t data_buffer[8]; 
static uint32_t timestamp_1hz;
static uint32_t timestamp_10hz;
static uint8_t can_data[8]; 
static dbc_message_header_t header;

void report_status_init(modPowerElectronicsPackStateTypedef *copy_pack_state)
{
    pack_state = copy_pack_state;
    timestamp_1hz = HAL_GetTick();
    timestamp_10hz = HAL_GetTick();
}

void report_status_task(void)
{
    // 1hz, 10ms
    if ((HAL_GetTick() - timestamp_1hz) >= 10) {
        timestamp_1hz = HAL_GetTick(); 
        report_safety_status(); 
    }

    // 10hz, 100ms
    if ((HAL_GetTick() - timestamp_10hz) >= 100) {
        timestamp_10hz = HAL_GetTick();
        report_battery_pack_voltage_and_current(); 
        report_load_voltage_and_current(); 
        reoprt_cell_voltage();
        report_soc(); 
        report_cell_and_bms_average_temperature_celsius(); 
        report_cell_temperature_celsius();
        report_bms_board_temperature_celsius();
        report_load_power_w();    
    }
}

void report_safety_status(void)
{
    safety_status_s safety_status = safety_check_get_status(); 

    dbc_BMS_Status_s bms_status; 
    modCANTransmitStandardID((0x620 & 0xFFFFFFFF), safety_status.bms_fault_data, 6);
}

void report_battery_pack_voltage_and_current(void) 
{
    static dbc_BMS_Pack_Info_s pack_info; 
    pack_info.BMS_Pack_Voltage = pack_state->packVoltage; 
    pack_info.BMS_Pack_Current = pack_state->packCurrent; 

    header = dbc_encode_BMS_Pack_Info(can_data, &pack_info);
	modCANTransmitStandardID(0x623, can_data, 8);
}

void report_load_voltage_and_current(void)
{
    static dbc_BMS_Load_Info_s load_info; 
    load_info.BMS_Load_Voltage = pack_state->loCurrentLoadVoltage; 
    load_info.BMS_Load_Current = pack_state->loCurrentLoadCurrent; 

    header = dbc_encode_BMS_Load_Info(can_data, &load_info);
    modCANTransmitStandardID(0x624, can_data, 8);
}

void reoprt_cell_voltage(void) 
{
    static dbc_BMS_Cell_Voltage_High_Low_s cell_voltage_high_low; 
    cell_voltage_high_low.BMS_Cell_Voltage_High = pack_state->cellVoltageHigh; 
    cell_voltage_high_low.BMS_Cell_Voltage_Low = pack_state->cellVoltageLow; 

    header = dbc_encode_BMS_Cell_Voltage_High_Low(can_data, &cell_voltage_high_low);
    modCANTransmitStandardID((0x625 & 0xFFFFFFFF), can_data, 8);

    static dbc_BMS_Cell_Voltage_Average_s cell_voltage_average; 
    cell_voltage_average.BMS_Cell_Voltage_Average = pack_state->cellVoltageAverage; 
    cell_voltage_average.BMS_Cell_Voltage_Mismatch = pack_state->cellVoltageMisMatch; 
    header = dbc_encode_BMS_Cell_Voltage_Average(can_data, &cell_voltage_average);

	modCANTransmitStandardID((0x626 & 0xFFFFFFFF), can_data, 8);
}

void report_soc(void)
{
    static dbc_BMS_Status_Soc_s soc_info; 
    soc_info.BMS_SOC = pack_state->SoC; 
    soc_info.BMS_Capacity = pack_state->SoCCapacityAh; 

    header = dbc_encode_BMS_Status_Soc(can_data, &soc_info);
    modCANTransmitStandardID(0x627, can_data, 8);
}

void report_cell_and_bms_average_temperature_celsius(void)
{
    static dbc_BMS_Temperature_Average_s temp_info; 
    temp_info.BMS_Cell_Temperature_Average = pack_state->tempBatteryAverage; 
    temp_info.BMS_Board_Temperature_Average = pack_state->tempBMSAverage; 

    header = dbc_encode_BMS_Temperature_Average(can_data, &temp_info);
    modCANTransmitStandardID(0x628, can_data, 8);
}

void report_cell_temperature_celsius(void)
{
    static dbc_BMS_Cell_Temperature_s cell_temp_info; 
    cell_temp_info.BMS_Cell_Temperature_High = pack_state->tempBatteryHigh; 
    cell_temp_info.BMS_Cell_Temperature_Low = pack_state->tempBatteryLow; 

    header = dbc_encode_BMS_Cell_Temperature(can_data, &cell_temp_info);
    modCANTransmitStandardID(0x629, can_data, 8);
}

void report_bms_board_temperature_celsius(void)
{
    static dbc_BMS_Board_Temperature_s bms_temp_info; 
    bms_temp_info.BMS_Board_Temperature_High = pack_state->tempBMSHigh; 
    bms_temp_info.BMS_Board_Temperature_Low = pack_state->tempBMSLow; 

    header = dbc_encode_BMS_Board_Temperature(can_data, &bms_temp_info);
    modCANTransmitStandardID(0x630, can_data, 8);
}

void report_load_power_w(void)
{
    static float load_power;
    load_power = pack_state->loCurrentLoadVoltage * pack_state->loCurrentLoadCurrent;

    static dbc_BMS_Load_Power_s load_power_info; 
    load_power_info.BMS_Load_Power = load_power; 

    header = dbc_encode_BMS_Load_Power(can_data, &load_power_info);
    modCANTransmitStandardID(0x631, can_data, 4);
}
