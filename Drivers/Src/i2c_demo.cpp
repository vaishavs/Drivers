/**
 ******************************************************************************
 * @file           : spi_demo.cpp
 * @author         : Vaishnavi
 * @date           : 22-06-2022
 * @brief          : Main program body for data transmission as a master device
 ******************************************************************************
 */

#include "gpio.h"
#include "i2c.h"
#include <cstring>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void GPIO_Init();
void GPIO_Button_Init(GPIO button);
void I2C_Init();

I2C i2c1(I2C1);

int main(void)
{
	GPIO button(GPIOC,13);

	GPIO_Init();
	GPIO_Button_Init(button);
	I2C_Init();

	char data[] = "Hello, Device!";
	uint8_t slave_addr = 0x80;

	/* Loop forever */
	while(1)
	{
		while(!button.ReadPin());

		//Send data
		i2c1.MasterTransmit(slave_addr, (uint8_t*)data, strlen(data));
	}
}

void GPIO_Init()
{
	//PB6-SCL, PB7-SDA; AF4

	GPIO scl(GPIOB,6);
	scl.setMode(GPIO_MODE_ALTFN, (uint8_t)4);
	scl.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);

	GPIO sda(GPIOB,7);
	sda.setMode(GPIO_MODE_ALTFN, (uint8_t)4);
	sda.Init(GPIO_SPEED_HIGH, GPIO_PULL_UP);
}

void GPIO_Button_Init(GPIO button)
{
	//Initialize User button: PC13
	button.setMode(GPIO_MODE_INPUT); //Set as input pin
	button.Init(GPIO_SPEED_HIGH, GPIO_PULL_NONE); //Initialize rest of the parameters
}

void I2C_Init()
{
	i2c1.Init(I2C_MODE_MASTER, I2C_SPEED_FM4K, 0x61, EN, I2C_DUTY_2);
}
