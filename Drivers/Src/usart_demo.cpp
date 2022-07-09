/**
 ******************************************************************************
 * @file           : spi_demo.cpp
 * @author         : Vaishnavi
 * @date           : 22-06-2022
 * @brief          : Main program body for data transmission
 ******************************************************************************
 */

#include "gpio.h"
#include "usart.h"
#include <cstring>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void GPIO_Init();
void GPIO_Button_Init(GPIO button);
void USART_Init();

USART usart2(USART2);

int main(void)
{
	GPIO button(GPIOC,13);

	GPIO_Init();
	GPIO_Button_Init(button);
	USART_Init();

	char data[] = "Hello, Device!";

	/* Loop forever */
	while(1)
	{
		while(!button.ReadPin());

		//Send data
		usart2.Transmit((uint8_t*)data, strlen(data));
	}
}

void GPIO_Init()
{
	//PD3-CTS PD4-RTS PD5-TX PD6-RX PD7-CK; AF7

	GPIO cts(GPIOD,3);
	cts.setMode(GPIO_MODE_ALTFN, (uint8_t)7);
	cts.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);

	GPIO rts(GPIOD,4);
	rts.setMode(GPIO_MODE_ALTFN, (uint8_t)7);
	rts.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);

	GPIO tx(GPIOD,5);
	tx.setMode(GPIO_MODE_ALTFN, (uint8_t)7);
	tx.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);

	GPIO rx(GPIOD,6);
	rx.setMode(GPIO_MODE_ALTFN, (uint8_t)7);
	rx.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);

	GPIO ck(GPIOD,7);
	ck.setMode(GPIO_MODE_ALTFN, (uint8_t)7);
	ck.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);
}

void GPIO_Button_Init(GPIO button)
{
	//Initialize User button: PC13
	button.setMode(GPIO_MODE_INPUT); //Set as input pin
	button.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE); //Initialize rest of the parameters
}

void USART_Init()
{
	usart2.Init(USART_WORD_8, USART_PARITY_EVEN, USART_MODE_TX, USART_STOP_1, USART_HW_NONE, USART_BAUD_115200);
}
