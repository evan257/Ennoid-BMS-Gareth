#pragma once

#include <stdbool.h>

#include "modPowerElectronics.h"
#include "modConfig.h"

typedef struct 
{
    bool status_is_ok;
    enum {
        cell_over_voltage = 1,
        cell_under_voltage,
        cell_over_temperature,
        bms_board_over_temperature,
        power_exceed_80kw
    } bms_fault_code_e; 

    uint8_t bms_fault_data[8]; 

} safety_status_s; 

void safety_check_init(modPowerElectronicsPackStateTypedef *copy_pack_state, modConfigGeneralConfigStructTypedef *copy_pack_configuration);
void safety_check_task(void); 
bool safety_check_cell_over_voltage(void);
bool safety_check_cell_under_voltage(void);
bool safety_check_cell_over_temperature(void); 
bool safety_check_bms_board_over_temperature(void); 
bool safety_check_power_exceed_80kw(void); 
safety_status_s safety_check_get_status(void); 




