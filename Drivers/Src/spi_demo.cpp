/**
 ******************************************************************************
 * @file           : spi_demo.cpp
 * @author         : Vaishnavi
 * @date           : 22-06-2022
 * @brief          : Main program body for data transmission as a master device
 ******************************************************************************
 */

#include "gpio.h"
#include "spi.h"
#include <cstring>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void GPIO_Init();
void GPIO_Button_Init(GPIO button);
void SPI_Init();

SPI spi2(SPI2);

int main(void)
{
	GPIO button(GPIOC,13);

	GPIO_Init();
	GPIO_Button_Init(button);
	SPI_Init();

	char data[] = "Hello, Device!";

	/* Loop forever */
	while(1)
	{
		while(!button.ReadPin());

		//Send data length
		uint8_t len = strlen(data);
		spi2.Transmit(&len, 2);

		//Send data
		spi2.Transmit((uint8_t*)data, strlen(data));
		spi2.CloseCommunication();
	}
}

void GPIO_Init()
{
	//PB12-NSS, PB13-SCK, PB14-MISO, PB15-MOSI; AF5

	GPIO nss(GPIOB,12);
	nss.setMode(GPIO_MODE_ALTFN, (uint8_t)5);
	nss.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE);

	GPIO sck(GPIOB,13);
	sck.setMode(GPIO_MODE_ALTFN, (uint8_t)5);
	sck.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE);

	GPIO miso(GPIOB,14);
	miso.setMode(GPIO_MODE_ALTFN, (uint8_t)5);
	miso.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE);

	GPIO mosi(GPIOB,15);
	mosi.setMode(GPIO_MODE_ALTFN, (uint8_t)5);
	mosi.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE);
}

void GPIO_Button_Init(GPIO button)
{
	//Initialize User button: PC13
	button.setMode(GPIO_MODE_INPUT); //Set as input pin
	button.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE); //Initialize rest of the parameters
}

void SPI_Init()
{
	spi2.Init(SPI_DEV_MASTER, SPI_BUS_FD, FREQ_DIV_2, SPI_DFF_16, LOW, FALL, DIS);
}
