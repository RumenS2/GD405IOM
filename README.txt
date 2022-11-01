This project is made for STM32CubeIDE. But STlink GDB check processor and refuse attempt to program.
STM32CubeProgrammer (ver 2.9) does not have this check, and may be used but Reset mode -> Hardware reset (because this program reinit SWCLK and SWDIO pins)
Port from STM32F405RGT6 to GD32F405RGT6
STM32CubeProgrammer may be used, but for THIS program, in STM32CubeProgrammer must be select Reset mode -> Hardware reset (cheap chinese Stlinkv2 clones does not drives hw Reset!!!)
st-tools from https://github.com/stlink-org/stlink may be used to flash THIS program, but only one time. Seems that st-flash does not drive  RESET pin??
But STM32CubeProgrammer work, and Windows STM32 ST-LINK Utility v4.5.0 also may be used (Reset mode ->Hardware reset)
