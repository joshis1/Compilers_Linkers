#arm-none-eabi-gcc - cross compiler
cc=arm-none-eabi-gcc
#machine is cortex-m4 so that it can generate instructions for cortex-m4
#Mthumb - because cortex-m4 only supports thumb mode instructions
MACH=cortex-m4
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -Wall -o0
#LDFLAGS= -nostdlib -T stm32.ld -Wl,-Map=final.map
#LDFLAGS= -nostdlib -T stm32.ld
LDFLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -T stm32.ld -Wl,-Map=final.map
LDFLAGS_SEMI_HOSTING = -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -T stm32.ld -Wl,-Map=final.map

#cortex m4 - m4f - hardware floating point unit is supported

#c standard GNU 11
# -o0 - no optimization

#$@ is target 
#$^ is dependency


all: main.o led.o stm32_startup.o syscalls.o final.elf

semi_hosting: main.o led.o stm32_startup.o final_semi.elf

## format is target:dependency
main.o:main.c
	$(cc) $(CFLAGS) -o $@ $^
	
led.o:led.c
	$(cc) $(CFLAGS) -o $@ $^
	
stm32_startup.o:stm32_startup.c
	$(cc) $(CFLAGS) -o $@ $^

syscalls.o:syscalls.c
	$(cc) $(CFLAGS) -o $@ $^	
	
final.elf: main.o led.o stm32_startup.o syscalls.o
	$(cc) $(LDFLAGS) -o $@ $^

final_semi.elf: main.o led.o stm32_startup.o 
	$(cc) $(LDFLAGS_SEMI_HOSTING) -o $@ $^

#for linux  rm -rf *.0 *.elf

clean:
	del *.o *.elf


#openocd -f C:\Users\shreyas.joshi\Downloads\xpack-openocd-0.10.0-14-win32-x64\xpack-openocd-0.10.0-14\bin\stm32f4discovery.cfg
#openocd -f C:\Users\sjoshi\Downloads\xpack-openocd-0.10.0-14-win32-x64\xpack-openocd-0.10.0-14\bin\stm32f4discovery.cfg
load:
	openocd -f C:\Users\sjoshi\Downloads\xpack-openocd-0.10.0-14-win32-x64\xpack-openocd-0.10.0-14\bin\stm32f4discovery.cfg



