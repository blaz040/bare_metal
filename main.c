#include <stdint.h>

#define RCC_AHB1RSTR 	((uint32_t *) 0x40023830)
#define GPIOB 		((uint32_t *) 0x40020400)
#define GPIOB_MODER 	((uint32_t *) 0x40020400)
#define GPIOB_OUDR	((uint32_t *) 0x40020414)

// LD1 PB0

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

void GPIO_init(GPIO* gpio, char gp, uint32_t Pin, uint32_t MODER/*,uint32_t OTYPE,uint32_t OSPEEDR,uint32_t PUPDR,uint32_t IDR,uint32_t ODR,uint32_t BSRR*/)
{
	gp = gp - 'A';
	
	*RCC_AHB1RSTR |= (1 << gp);
	gpio = (GPIO *)( 0x40020000 + (0x400)*gp);
	
	gpio->MODER |= MODER << (Pin*2);
	
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

void delay(int wait)
{
	for(int i = 0; i<wait;i++)
	{
		i = i;
	}
}

int main()
{
	GPIO* LED1;
	GPIO* LED2;
		
	GPIO_init(LED1,'B',0,1);
	GPIO_init(LED2,'B',7,1);
	
	while(1)
	{
		GPIO_write(LED2,0,1);
		GPIO_write(LED2,7,0);
		//*GPIOB_OUDR |= (1 << 0);
		delay(1000*1000);
		
		GPIO_write(LED1,0,0);
		GPIO_write(LED1,7,1);
		//*GPIOB_OUDR &= ~(1 << 0);
		delay(1000*1000);
	}
}

