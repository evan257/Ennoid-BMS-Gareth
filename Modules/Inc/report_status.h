#pragma once

#include "modPowerElectronics.h"

void report_status_init(modPowerElectronicsPackStateTypedef *copy_pack_state); 
void report_status_task(void);
void report_safety_status(void);
void reoprt_cell_voltage(void); 
void report_cell_temperature_celsius(void);
void report_bms_board_temperature_celsius(void); 
void report_battery_pack_voltage_and_current(void); 
void report_load_voltage_and_current(void);
void report_load_power_w(void);
void report_soc(void); 
void report_cell_and_bms_average_temperature_celsius(void); 
