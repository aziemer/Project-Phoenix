# Project Phoenix
**Alternative firmware for the Voltcraft VC-7055BT = OWON XDM-2041 = PeakTech P 4094 DMMs**

This project was born out of pure necessity, as my VC-7055BT did not work (stopping on the boot progress bar).  

After a lot of unsuccessful repair attempts, I searched the internet and found [Petteri Aimonen](http://essentialscrap.com), who did a modification on his OWON (which the Voltcraft is a rebrand of) and he had a lot of technical information about the device, from the hardware as well as the software side.

He provided me with some software, that showed that my device's hardware was not broken - its firmware obviously "just" failed.

As my DMM now was already open, modified (I needed to reflash the MCU via SWD - btw. firmware uploads via RS232 were not even possible with the original firmware), and one of Petteri's statements on his site was, that he would like to write a replacement firmware for the OWON, I thought: why not start it myself.

So here it goes...

I started with a new bootloader (in another project), based on the STM32Duino bootloader. Now I can upload new firmware via USB, which involved adding the non-populated USB Socket on the back and changing some resistors.

What works:

* TFT routines
* keypad scanning
* Measurement of U / I / R / Diode / Continuity (no buzzer yet). Displayed values are __UNCALIBRATED__.
* Frequency

What does not (yet) work:

* Calibration
* the menu functions
* SCPI module (with calibration commands)
* add all the other functions ;-)
* Capacitance measurement (need to figure out how the values are processed)
* Temperature measurement


To install the firmware on the device, the BOOT0 pin has to pulled up (i.e. by shorting Pin 4 and 6 of the connector CO2 on the front PCB), the upload via STM-Cube programmer (or the programmer of your coice) over RS232.

There are 2 defines to be checked in the makefile:

* BOOTLOADER=x - 0 or 1
* CRYSTAL=xxxxx - your HY3131 crystal frequency

BOOTLOADER=1 excludes the TFT initialization, as this is done in the bootloader. CODE starts at 0x8002000.

BOOTLOADER=0 includes the TFT init, CODE starts at 0x8000000.


**Development based on and using ideas and code snippets from:**

* Application framework built with STM32 Cube (c) [STMicroelectronics](https://www.st.com)
* DIGILENT DMM Shield firmware (c) [DIGILENT](https://digilent.com/reference/add-ons/dmm-shield/start?redirect=1) by Cristian Fatu (cristian.fatu@digilent.ro)
* Adafruit GFX Library (c) [Adafruit Industries](https://www.adafruit.com)
* A lot of protocol and hardware engineering by [Petteri Aimonen](http://essentialscrap.com), GitHub: [OWON XDM2041 Info](https://github.com/PetteriAimonen/owon-xdm2041-info)

_This is work in progress ..._

