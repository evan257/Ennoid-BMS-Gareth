# Other parts of this project

This is the firmware repository containing all firmware source files. There are three more repositories for this project:

[ENNOID-BMS Hardware](https://github.com/EnnoidMe/ENNOID-BMS) The hardware sourcefiles.

[ENNOID-BMS Bootloader](https://github.com/EnnoidMe/DieBieMS-Bootloader) Same as DieBieMS bootloader. Can be flashed with the BMS Tool in the firmware tab or with an STlink-V2. 

[ENNOID-BMS Configuration tool](https://github.com/EnnoidMe/ENNOID-BMS-Tool) the tool to configure the ENNOID-BMS and upload the bootloader / update the main firmware.


# Building with GCC

- Install arm-none-eabi-gcc
- Configure `generalDefines.h` for LV, HV, SS, or SS_LITE versions
- Makefile has rules for flashing with STLink or JLink
- Could also load using bootloader with binary

When flashing the application the start address should be: <b>0x08000000</b>
When flashing the bootloader the start address should be: <b>0x08032000</b>

The flash is formatted as follows (summary):

((uint32_t)0x08000000) /* Base @ of Page 0, 2 Kbytes */  // Startup Code - Main application<br>
((uint32_t)0x08000800) /* Base @ of Page 1, 2 Kbytes */  // Page0 - EEPROM emulation<br>
((uint32_t)0x08001000) /* Base @ of Page 2, 2 Kbytes */  // Page1 - EEPROM emulation<br>
((uint32_t)0x08001800) /* Base @ of Page 3, 2 Kbytes */  // Remainder of the main application firmware stars from here.<br>
((uint32_t)0x08019000) /* Base @ of Page 50, 2 Kbytes */  // New app firmware base addres<br>
((uint32_t)0x08032000) /* Base @ of Page 100, 2 Kbytes */  // Bootloader base<br>

See "modFlash.h" and "modFlash.c" for more info.
