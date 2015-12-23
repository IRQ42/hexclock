# hexclock
Alarm clock that keeps time in binary by incrementing a 16-bit unsigned integer, and displays the time in hexadecimal via a multiplexed common anode seven segment LED display. The microcontroller used in this project is an ATMEL AVR ATtiny2313a MCU, featuring just 2kb flash program memory, 128b of SRAM, as well as 128b of EEPROM for use as nonvolatile memory which will retain values after reset or power cycle. It is important to note that the AVR architecture is a type of Harvard Architecture, in that program memory (code) and data memory (for data such as variables, and the stack) each having their own seperate address space, unlike the Von Neumann machine e.g. CPU archictures such as x86 where program code and data share the same memory address space. EEPROM also has it's own address space and cannot be accessed via ordinary pointers without using special instructions which are part of the AVR instruction set architecture; these features can be accessed using the C programming language using macros included in the relevant standard header files.

For more information about how time can be represented in hexadecimal or binary see the wikipedia link below: https://en.wikipedia.org/wiki/Hexadecimal_time

Depends:
  gcc-avr
  binutils-avr
  avrdude
  avr-libc
