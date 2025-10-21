#include "driverHWStatus.h"
#include "generalDefines.h"
#include "modEffect.h"
#include "driverHWStatus.h"
#include "modConfig.h"

#include "safety_check.h"

#include <stdio.h>

static modPowerElectronicsPackStateTypedef *pack_state;
static modConfigGeneralConfigStructTypedef *pack_configuration; 

static safety_status_s safety_status; 

static bool safty_enable; 

static uint32_t last_status_ok_timestamp; 
static uint32_t last_fault_timestamp;
static uint32_t fault_time_ms;

void safety_check_init(modPowerElectronicsPackStateTypedef *copy_pack_state, modConfigGeneralConfigStructTypedef *copy_pack_configuration) 
{
    pack_state = copy_pack_state;
    pack_configuration = copy_pack_configuration;
    safty_enable = true; 
    modEffectChangeState(STAT_LED_POWER, STAT_RESET);
    modEffectChangeState(STAT_BUZZER, STAT_RESET);

    pack_configuration->buzzerSignalSource = 1;
    pack_configuration->noOfCellsSeries = 72;
    pack_configuration->cellMonitorICCount = 6;
    pack_configuration->noOfCellsParallel = 9; 
    pack_configuration->batteryCapacity = 27; 
    pack_configuration->cellHardOverVoltage = 4.2;
    pack_configuration->cellHardUnderVoltage = 2.0;
    pack_configuration->cellLCSoftUnderVoltage = 2.5;
    pack_configuration->noOfTempSensorPerModule = 5;
    pack_configuration->noOfTempSensorPerExpansionBoard = 6;
    pack_configuration->noOfExpansionBoard = 6;
    pack_configuration->NTCBetaFactor[modConfigNoOfNTCTypes] = 4540;
    pack_configuration->allowedTempBattChargingMax = 60;
    pack_configuration->allowedTempBattDischargingMax = 60;
    pack_configuration->allowedTempBMSMax = 60;
    pack_configuration->maxAllowedCurrent = 300; 
    pack_configuration->shuntLCFactor = -0.042; 

    pack_state->SoC = 95; 

    last_status_ok_timestamp = HAL_GetTick(); 

    fault_time_ms = HAL_GetTick(); 

}

void safety_check_task(void) 
{
    safety_status.status_is_ok = (safety_check_cell_over_voltage() & safety_check_cell_under_voltage() 
                                & safety_check_cell_over_temperature() & safety_check_bms_board_over_temperature()
                                & safety_check_power_exceed_80kw()); 

    if (pack_configuration->buzzerSignalSource) {
        safty_enable = true;
    } else {
        safty_enable = false; 
        modEffectChangeState(STAT_LED_POWER, STAT_RESET);
    }

    #if 0
    if (!safety_status.status_is_ok) {
        last_fault_timestamp = HAL_GetTick(); 

        fault_time_ms = last_fault_timestamp - fault_time_ms;

        if (safty_enable && fault_time_ms >= 5 * 1000) {
            modEffectChangeState(STAT_LED_POWER, STAT_SET);
            safety_status.bms_fault_data[0] = 0xFF; 
        }
    } else {
        safety_status.bms_fault_data[0] = 0 & 0xFF;
        modEffectChangeState(STAT_LED_POWER, STAT_RESET);
        fault_time_ms = HAL_GetTick(); 
    }
    #endif

    #if 1
    if (safety_status.status_is_ok) {
        safety_status.bms_fault_data[0] = 0 & 0xFF;

        if (safty_enable) {
            modEffectChangeState(STAT_LED_POWER, STAT_RESET);
        }
        last_status_ok_timestamp = HAL_GetTick(); 

    } else {
        
        safety_status.bms_fault_data[0] = 0xFF; 
        static uint32_t fault_time_ms;

        fault_time_ms = HAL_GetTick() - last_status_ok_timestamp;
        
        // Check continous fault time here, currently set to five seconds and will trigger HVIL open
        // It's a temp solution, eventually we need dig in actual data processing for temperature and voltage
        if (safty_enable && fault_time_ms >= 5 * 1000) {
            modEffectChangeState(STAT_LED_POWER, STAT_SET);
        }

    }
    #endif
}

bool safety_check_cell_over_voltage(void)
{
    bool cell_is_ok = false; 

    if (pack_state->cellVoltageHigh >= pack_configuration->cellHardOverVoltage) {
        safety_status.bms_fault_data[cell_over_voltage] = 0xFF; 
        cell_is_ok = false; 
    } else {
        safety_status.bms_fault_data[cell_over_voltage] = 0 & 0xFF;
        cell_is_ok = true; 
    }

    return cell_is_ok;
}

bool safety_check_cell_under_voltage(void)
{
    bool cell_is_ok = false; 

    if (pack_state->cellVoltageLow <= pack_configuration->cellHardUnderVoltage) {
        safety_status.bms_fault_data[cell_under_voltage] = 0xFF; 
        cell_is_ok =  false; 
    } else {
        safety_status.bms_fault_data[cell_under_voltage] = 0 & 0xFF;
        cell_is_ok = true; 
    }

    return cell_is_ok;
}

bool safety_check_cell_over_temperature(void)
{
    bool temperature_is_ok = false; 

    if (pack_state->tempBatteryHigh >= pack_configuration->allowedTempBattDischargingMax) {
        safety_status.bms_fault_data[cell_over_temperature] = 0xFF; 
        temperature_is_ok =  false;
    } else {
        safety_status.bms_fault_data[cell_over_temperature] = 0 & 0xFF; 
        temperature_is_ok = true; 
    }

    return temperature_is_ok; 
}

bool safety_check_bms_board_over_temperature(void)
{
    bool bms_temperature_is_ok = false; 

    // It can be bms master board or bms slave boards
    if (pack_state->tempBMSHigh >= pack_configuration->allowedTempBMSMax) {
        safety_status.bms_fault_data[bms_board_over_temperature] = 0xFF; 
        bms_temperature_is_ok = false; 
    } else {
        safety_status.bms_fault_data[bms_board_over_temperature] = 0 & 0xFF; 
        bms_temperature_is_ok = true; 
    }

    return bms_temperature_is_ok; 
}

bool safety_check_power_exceed_80kw(void)
{
    bool power_is_ok = false; 

    float load_power;
    float maximum_allowed_power = 79 * 1000; 
    load_power = pack_state->packVoltage * pack_state->loCurrentLoadCurrent;
    if (load_power >= maximum_allowed_power) {
        safety_status.bms_fault_data[power_exceed_80kw] = 0xFF; 
        power_is_ok =  false; 
    } else {
        safety_status.bms_fault_data[power_exceed_80kw] = 0 & 0xFF; 
        power_is_ok = true; 
    }
    
    return power_is_ok; 
}

safety_status_s safety_check_get_status(void)
{
    return safety_status; 
}
