# bare metal STM32F413ZH
make commands:
	-all: runs compile and debug rules
	-compile: compiles main.c,vector_table.S,core.S,STM32413.ld
	-debug: loads main.elf to board
	-clean: removes *.o and *.elf
	
