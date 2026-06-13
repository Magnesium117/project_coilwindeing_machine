#include "stm32f446xx.h"
#include "stm32f4xx_ll_gpio.h"
#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#define COIL1_PORT GPIOB
#define COIL2_PORT GPIOC
// 13 und 14 sind die JTAG Pins -> man kann dann nicht gescheit
// uploaden
#define PIN_11 LL_GPIO_PIN_1
#define PIN_12 LL_GPIO_PIN_2
#define PIN_21 LL_GPIO_PIN_2
#define PIN_22 LL_GPIO_PIN_3
void Step(int N);
void initStepperDriver();

#endif // !STEPPER_DRIVER_H
#define STEPPER_DRIVER_H
