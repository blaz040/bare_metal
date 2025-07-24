#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "drivers/stm32f413xx.h"

#define RCC_AHB1ENR	((uint32_t *) 0x40023830)
#define RCC_APB2ENR	((uint32_t *) 0x40023844)
//#define GPIOB 		((uint32_t *) 0x40020400)
//#define GPIOB_MODER 	((uint32_t *) 0x40020400)
//#define GPIOB_OUDR	((uint32_t *) 0x40020414)
#define SYSCFG_EXTICR4	((uint32_t *) 0x40013814)
#define EXTI_IMR	((uint32_t *) 0x40013C00)
#define EXTI_EMR	((uint32_t *) 0x40013C04)
#define EXTI_RTSR	((uint32_t *) 0x40013C08)
#define EXTI_FTSR	((uint32_t *) 0x40013C0C)
#define EXTI_PR		((uint32_t *) 0x40013C14)

uint32_t core_clock_hz;
uint32_t time = 0;
uint32_t SysTick_tick; 

// LD1 PB0
// button  PC13
typedef struct _GPIO
{
	uint32_t MODER;
	uint32_t OTYPER;
	uint32_t OSPEEDR;
	uint32_t PUPDR;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;

}GPIO;

void GPIO_init(GPIO** gpio, char gp, uint32_t Pin, uint32_t MODER/*,uint32_t OTYPE,uint32_t OSPEEDR,uint32_t PUPDR,uint32_t IDR,uint32_t ODR,uint32_t BSRR*/)
{
	
	gp = gp - 'A';
	
	*RCC_AHB1ENR |= (1 << gp);
	*gpio = (GPIO *)( 0x40020000 + (0x400)*gp);
	
	(*gpio)->MODER &= ~(0x3 << (Pin*2));
	(*gpio)->MODER |= MODER << (Pin*2);
}
void GPIO_write(GPIO* gpio, uint32_t Pin, uint32_t val)
{
	if(val == 1)
	{
		gpio->ODR |= (1 << Pin);	
	}
	else
	{
		gpio->ODR &= ~(1 << Pin);
	}
	
}
void GPIO_toggle(GPIO* gpio, uint32_t Pin)
{
	gpio->ODR ^= 1<<Pin;
}
uint32_t  GPIO_read(GPIO* gpio,uint32_t Pin)
{
	return gpio->IDR & (1 << Pin);
}
void button_init(GPIO** BUTTON)
{
	GPIO_init(BUTTON,'C',13,0);
	(*BUTTON)->PUPDR &= ~(0x3 << (2*13));
	(*BUTTON)->PUPDR |= 0x2 << (2*13);
	
	// button  PC13
	*RCC_APB2ENR |= (1 << 14);
	//*RCC_APB2ENR |= (1 << 15);
	
	*SYSCFG_EXTICR4 &= ~(0xF << 4);
	*SYSCFG_EXTICR4 |= (0x2 << 4);
	
	*EXTI_IMR |= (1 << 13);
	*EXTI_RTSR |= (1 << 13); // on button RELEASE interrupt
	
	*EXTI_FTSR &= ~(1 << 13);

	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_SetPriority(EXTI15_10_IRQn, 0x10);
	
	
}
void FLASH_init()
{
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS ;

}
void PLL_init()
{
	FLASH_init();

	int m = 96; // more bit med 50<m<432
	int n = 8;	// 2< n <15
	int p = 0x1;
	int pp = 4;

	RCC->PLLCFGR &= ~( RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLM_Msk );
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI | ( m << RCC_PLLCFGR_PLLN_Pos)| (n << RCC_PLLCFGR_PLLM_Pos) | (p << RCC_PLLCFGR_PLLP_Pos));
		
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)) {}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
	
	core_clock_hz = (16000000*m/n)/pp; // 48 MHz
	SysTick_tick = core_clock_hz/1000; // vsako mikro sedundo
}
void TIM_init(TIM_TypeDef ** TIMx, uint32_t ms)
{
	*TIMx = (TIM_TypeDef*) (TIM2_BASE);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	RCC->APB1RSTR |=  (RCC_APB1RSTR_TIM2RST);
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM2RST);
	
	NVIC_SetPriority(TIM2_IRQn,0x3);
	NVIC_EnableIRQ(TIM2_IRQn);

	(*TIMx)->PSC = core_clock_hz/1000;
	(*TIMx)->ARR = ms;
	(*TIMx)->EGR  |= TIM_EGR_UG;
	(*TIMx)->DIER |= TIM_DIER_UIE;
	(*TIMx)->CR1 |= TIM_CR1_CEN;
}
void USART_init()
{
	//	USART3
	//		PD8 TX AF7
	//		PD9 RX AF7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	GPIOD->MODER |= (0x2 << (8*2))| 0x2 << (9*2);
	GPIOD->AFR[1] |= (0x7 << 0) | 0x7 << 1*4;

	USART3->BRR = 0x1A1;

	USART3->CR1 |= USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	
	//uint32_t usartDIV = core_clock_hz / 115200

	USART3->DR = '0';
	
	NVIC_SetPriority(USART3_IRQn,0x2);
	NVIC_EnableIRQ(USART3_IRQn);

}
void delay(int ms)
{
	//uint32_t multiplier = 1000 /(core_clock_hz / SysTick_tick);
	ms =  ms;
	int stop_time = time+ms;
	while(time <= stop_time){}
}
void send(char* buffer);

GPIO* LED1 = 0x0;
GPIO* LED2 = 0x0;
GPIO* LED3 = 0x0;
GPIO* BUTTON = 0x0;

TIM_TypeDef* TIMer;


int main()
{
	PLL_init();
	
	SysTick_Config(SysTick_tick); // vsako milisekundo
	
	GPIO_init(&LED1,'B',0,1);
	GPIO_init(&LED2,'B',7,1);
	GPIO_init(&LED3,'B',14,1);// PB14

	button_init(&BUTTON);

	//TIM_init(&TIMer,100);
	USART_init();
	
	while(1)
	{
	/*	if(USART3->SR & USART_SR_TXE)
			USART3->DR = 'a';
	*/
		/*
		if(BUTTON->IDR |= (1 << 13) == 1)
			GPIO_write(LED1,0,1);
		else
		GPIO_write(LED1,0,0);
	*/	
		
		//GPIO_write(LED1,0,1);
		//GPIO_write(LED2,7,0);
		//delay(1000);
		
		//GPIO_write(LED1,0,0);
		//GPIO_write(LED2,7,1);
		//delay(1000);
	}
}
int bounce_time;
void EXTI15_10_IRQHandler(void)
{
	if(*EXTI_PR & (1 << 13)){
		*EXTI_PR |= (1 << 13);
		if(bounce_time == 0 || time - bounce_time >= 100) 
		{
			if(GPIO_read(LED1,0) == 1)
				GPIO_write(LED1,0,0);
			else
				GPIO_write(LED1,0,1);
			bounce_time = time;
		}
	}
}
void TIM2_IRQHandler(void)
{
	LED3->ODR ^=  1 << 14;
	TIMer->SR &= ~(TIM_SR_UIF);
}

void send(char* buffer)
{
	while(*buffer != '\0')
	{
		GPIO_toggle(LED2,7);
		while(!(USART3->SR & USART_SR_TXE));
			USART3->DR = *buffer++;
	}
}

char buffer[9];
int ix = 0;
void USART3_IRQHandler(void)
{
	char txt = USART3->DR;
	if(txt == '\r')
	{
		send(" :<");
		send(buffer);
		send(">");
		for(int i = 0; i<9; i++)
			buffer[i] = 0;
		ix = 0;
		send("\r\n");
		return;
	} 

	buffer[ix++%8] = txt;
	
	if(strcmp(buffer,"LED ON") == 0)
	{
		GPIO_write(LED1,0,1);
	}
	else if(strcmp(buffer,"LED OFF") == 0)
	{
		GPIO_write(LED1,0,0);
	}
	char tmp[2] = {txt,0};
	send(tmp);
}
void SysTick_Handler(void)
{
	time++;
}
