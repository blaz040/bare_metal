#include <stdint.h>

#include "drivers/stm32f413xx.h"

#define RCC_AHB1ENR	((uint32_t *) 0x40023830)
#define RCC_APB2ENR	((uint32_t *) 0x40017044)
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
	int a = RCC->CFGR & RCC_CFGR_SW_Msk;
	int b = RCC->CR & RCC_CR_HSION_Msk;
	int c = RCC->CR & RCC_CR_HSIRDY_Msk; 
	//RCC->CR |= RCC_CR_HSION;
	//while(!(RCC->CR & RCC_CR_HSIRDY)) {}
	int m = 96;
	int n = 8;
	int p = 0x1;
	int pp = 4;

	RCC->PLLCFGR &= ~( RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLM_Msk );
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI | ( m << RCC_PLLCFGR_PLLN_Pos)| (n << RCC_PLLCFGR_PLLM_Pos) | (p << RCC_PLLCFGR_PLLP_Pos));
	
	int d = (RCC->PLLCFGR & RCC_PLLCFGR_PLLP_Msk )>> RCC_PLLCFGR_PLLP_Pos;
	
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY)) {}

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
	
	a = RCC->CFGR & RCC_CFGR_SW_Msk;
	b = RCC->CR & RCC_CR_HSION_Msk;
	c = RCC->CR & RCC_CR_HSIRDY_Msk; 

	core_clock_hz = (16000000*m/n)/pp; // 48MHz
	a = 0;
}


void delay(int wait)
{
	for(int i = 0; i<wait;i++)
	{
		i = i;
	}
}

GPIO* LED1 = 0x0;
GPIO* LED2 = 0x0;
GPIO* BUTTON = 0x0;

int main()
{
	//PLL_init();

			
	GPIO_init(&LED1,'B',0,1);
	GPIO_init(&LED2,'B',7,1);
	
	//button_init(&BUTTON);
	

	while(1)
	{
	/*
		if(BUTTON->IDR |= (1 << 13) == 1)
			GPIO_write(LED1,0,1);
		else
		GPIO_write(LED1,0,0);
	*/	
		
		//GPIO_write(LED1,0,1);
		GPIO_write(LED2,7,0);
		delay(1000*1000);
		
		//GPIO_write(LED1,0,0);
		GPIO_write(LED2,7,1);
		delay(1000*1000);
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(*EXTI_PR & (1 << 13)){
		if(GPIO_read(LED1,0) == 1)
			GPIO_write(LED1,0,0);
		else
			GPIO_write(LED1,0,1);
		//*EXTI_EMR &= ~(1 <<13);
		*EXTI_PR |= (1 << 13);
	}
}

