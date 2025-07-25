# bare metal STM32F413ZH

## make commands

	all:	 runs compile and debug 
	compile: compiles main.c,vector_table.S,core.S,STM32413.ld into main.elf
	debug: 	 loads main.elf to board
	clean: 	 removes *.o and *.elf
	
## Setup
 to connect with the board use st-util, can be downloaded with ```sudo apt install st-util```. Then run ```make``` to flush the program onto the board. 

 ## USART3

  Commands for communicating on serial terminal with board through USART3:

	LED ON -> turns on LED1
	LED OFF -> turns off LED1
	Press Enter -> to reset message
     
The blue LED2 flickers when sending data through USART3

## PIN positions 

	LED1 : PB0
	LED2 : PB7
	LED3 : PB14
	
	Button: PC13
	
