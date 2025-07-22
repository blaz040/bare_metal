# bare metal STM32F413ZH

## make commands

	all:	 runs compile and debug 
	compile: compiles main.c,vector_table.S,core.S,STM32413.ld into main.elf
	debug: 	 loads main.elf to board
	clean: 	 removes *.o and *.elf
	
