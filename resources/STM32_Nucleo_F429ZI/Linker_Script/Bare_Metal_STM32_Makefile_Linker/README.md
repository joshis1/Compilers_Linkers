# Compile for final_semi.elf - semi hosting 

*tools-used - openocd*

## To start the openocd, run the below command.
* make load 

## Open another terminal to run gdb session

* make semi_hosting

*write image - final_semi.elf*

* arm-none-eabi-gdb.exe 

* (gdb) target remote localhost:3333

* (gdb) monitor reset init 

* (gdb) monitor flash write_image erase final_semi.elf
 

*openocd*
* (gdb) monitor arm semihosting enable 
* (gdb) monitor reset 
* (gdb) monitor halt
* (gdb) monitor resume
* (gdb) monitor shutdown


# Compile for final.elf without semi hosting printfs() but using syscall - support

*-- to do - implement write() in syscall to support printfs.*

* make all

*write image - final_semi.elf*

* arm-none-eabi-gdb.exe 

* (gdb) target remote localhost:3333

* (gdb) monitor flash write_image erase final.elf

*openocd*
* (gdb) monitor reset 
* (gdb) monitor halt
* (gdb) monitor shutdown


# Using gdb.
* arm-none-eabi-gdb.exe 
* (gdb) target remote localhost:3333
* (gdb) monitor reset init 


# Using telnet putty. 

* localhost - 4444 

* Just remove the monitor prefix and use all the commands described in gdb.

# Testing
* All this is tested on Windows 10 machine 
* GNU Makefile for Windows
* GNU ARM toolchain for Windows.
* Hardware - STM32 Nucleo -F429ZI - ARM Cortext M4. 

# References 

* https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/Common-Variable-Attributes.html#Common-Variable-Attributes

* https://gcc.gnu.org/onlinedocs/gcc-9.3.0/gcc/ARM-Function-Attributes.html#ARM-Function-Attributes
