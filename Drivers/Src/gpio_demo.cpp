/**
 ******************************************************************************
 * @file           : gpio_demo.cpp
 * @author         : Vaishnavi
 * @date           : 22-06-2022
 * @brief          : Main program body
 ******************************************************************************
 */
#include "stm32f411xe.h"
#include "gpio.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
	//Initialize LED2: PA5
	GPIO led(GPIOA,5,GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	led.setMode(GPIO_MODE_OUTPUT, GPIO_TYPE_PP); //Set as output pin
	led.Init(); //Initialize rest of the parameters

	//Initialize User button: PC13
	GPIO button(GPIOC,13,GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	button.setMode(GPIO_MODE_INPUT); //Set as input pin
	button.Init(); //Initialize rest of the parameters

    /* Loop forever */
	for(;;) {
		if(!button.ReadPin())
			led.TogglePin();
	}
}
