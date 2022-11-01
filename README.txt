This project is made for STM32CubeIDE. But STlink GDB check processor and refuse attempt to program.
STM32CubeProgrammer (ver 2.9) does not have this check, and may be used but Reset mode -> Hardware reset (because this program reinit SWCLK and SWDIO pins)
Иомка електрическа(хидравлична) с всички екстри (за дисплея има версии с проц 429 и 767)
Порт от STM32F405RGT6 към GD32F405RGT6
Може да се изполва STM32CubeProgrammer, но за ТАЗИ програма в STM32CubeProgrammer трябва да се избере Reset mode -> Hardware reset (cheap chinese Stlinkv2 clones doses not drives hw Reset!!!)
