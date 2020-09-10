ENTRY(Reset_Handler)

/*Memory Map - STM32F429ZI - Figure 19*/

/** Linker file association - >arm-none-eabi-gcc -nostdlib -T stm32.ld *.o -o final.elf **/

/** Important syntax - ORIGIN space =<value>, LENGTH space =<value> **/

MEMORY
{
  FLASH(rx):ORIGIN =0x08000000,LENGTH =1024K
  SRAM(rwx):ORIGIN =0x20000000, LENGTH =128K
}

__max_heap_size = 0x400; /** A symbol declaration ..not a variable**/
__max_stack_size = 0x200; /** A symbol declaration..not a variable **/

SECTIONS
{
  .text :
  {
    *(.isr_vector)
    /** merge all text section of each input files **/
    *(.text)
	*(.text.*)
	*(.init)
	*(.fini)
	*(.rodata)
	*(.rodata.*)
	. = ALIGN(4); /** Align the address for each boundary addres -- word boundary **/
    /*merge all .isr vector section of all input files */
	/* merge all .text section of all input files */
	/* merge all .ro data section of all input files. */
	_etext = .; /** this symbol will hold the end of the text section **/
  }> FLASH /**Only LMA **/

  _la_data  = LOADADDR(.data);
  .data :
  {
    _sdata = .; /** assign a value to a symbol 'start_of_data' - VMA - SRAM address**/
    /* merge all .data section of all input files. */
	*(.data)
	*(.data.*)
	. = ALIGN(4); /** Align the address for each boundary addres -- word boundary **/
	_edata = .; 
  }>SRAM AT> FLASH /**VMA - SRAM and LMA - Flash **/
  
  
  .bss :
  {
  _sbss = .;
  __bss_start__ = _sbss; /*required by nano new lib .syscalls - mainCRTStartup **/
    *(.bss)
	*(.bss.*)
	*(COMMON)
	. = ALIGN(4); /** Align the address for each boundary addres -- word boundary **/
  _ebss = .;
  __bss_end__ = _ebss; /*required by nano new lib .syscalls - mainCRTStartup **/
  . = ALIGN(4); /** Align the address for each boundary addres -- word boundary **/
  end = .; /*required by nano new lib .syscalls - sbrk() **/
  __end__ = end; /* required by semi-hosting by openocd **/
  }>SRAM  /**only VMA **/

}


/**location counter - denoted '.' **/
/** with location information **/
/** lcoation counter should appear only inside the SECTIONS command **/
/** location counter is incremented by the isize of the output section **/

/** linker script symbol **/
/** Symbol is the name of an address **/
/** Symbol table -- main.o **/
/** in the main.o, the compiler maintains - symbol table **/
/** For compiler, the variable is a symbol **/
/** Symbol table - address and symbol **/
