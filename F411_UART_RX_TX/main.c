#include "stm32f4xx.h"                  // Device header

#define INIT_OK					GPIOA->BSRR |= GPIO_BSRR_BS7;

uint32_t tim_delay;
uint8_t rx_status;
uint8_t rx_data;
uint8_t rx_bit;
uint8_t rx_buf[64];

void mcu_init(void)
{
	SystemInit();
	SysTick_Config(100000);
}

void SysTick_Handler(void)
{
	if(tim_delay != 0) tim_delay--;
}

void Delay_ms(uint32_t delay)
{
	tim_delay = delay;
	while(tim_delay != 0) 
	{
		if(rx_status == 1) return;
	}
}

void send_uart (uint_fast8_t data)
{
	while (!(USART1 -> SR & USART_SR_TC));
	USART1 -> DR = data;
}

void send_str (char* string)
{
	uint8_t i = 0;
	while (string[i])
	{
		send_uart(string[i]);
		i++;
	}
	send_uart('\r');
	send_uart('\n');
}

void _delay (uint32_t i)
{
	uint32_t j=0;
	//i*=7;
	for (j = 0; j<= i; j++)
	{}
	
}

void gpio_init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODER6;
	GPIOA->MODER |= GPIO_MODER_MODER6_0;

	GPIOA->MODER &= ~GPIO_MODER_MODER7;
	GPIOA->MODER |= GPIO_MODER_MODER7_0;
	//GPIOA->OTYPER &=~ GPIO_OTYPER_OT_7;
	//GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED7_0;
}

void uart_init(void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	GPIOA -> MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
	GPIOA -> AFR[1] |= 0x770;
	USART1 -> BRR = SystemCoreClock/115200; //??????????
	USART1 -> CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE);
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void)
{
	if((USART1 -> SR & USART_SR_RXNE) != 0)
	{
		rx_data = USART1 -> DR;
		rx_buf[rx_bit] = rx_data;
		if (rx_bit != 64) rx_bit++;
		else rx_bit = 0;
		if(rx_data == 0x0D) rx_status = 1;
	}
}

int main ()
{
	mcu_init();
	gpio_init();
	uart_init();
		
	INIT_OK;//PA7
	
	while(1)
	{
		GPIOA->BSRR |= GPIO_BSRR_BS6;
		
		Delay_ms(100);
		//_delay(10000000);
		GPIOA->BSRR |= GPIO_BSRR_BR6;
		send_str("stm32");
		rx_status = 0;
		
		Delay_ms(100);
		//_delay(10000000);
		
		if (rx_status == 1)
		{
			if (rx_buf[0] == '5')
			{
				rx_status = 0;
				rx_bit = 0;
				rx_buf[0] = 0;
				send_str("5 OK");
				Delay_ms(1000);
			}
		}
		
	}
	
}

