#ifndef MAIN_H
#define MAIN_H
#include "stm32f446xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_crc.h"
#include "stm32f4xx_ll_dac.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_dma2d.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_fmc.h"
#include "stm32f4xx_ll_fmpi2c.h"
#include "stm32f4xx_ll_fsmc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_lptim.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_rng.h"
#include "stm32f4xx_ll_rtc.h"
#include "stm32f4xx_ll_sdmmc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_usb.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_wwdg.h"

#include "clock_config.h"

#define USERLED_PORT GPIOA
#define USERLED_PIN LL_GPIO_PIN_5
#define USERBTN_PORT GPIOC
#define USERBTN_PIN LL_GPIO_PIN_13
//
// Motor Pins
//
#define SIG_PORT GPIOC // Pins 6 8
#define EN_PORT GPIOB  // Pins 1 2
#define L1_SIG_PIN LL_GPIO_PIN_6
// 13 und 14 sind die JTAG Pins -> man kann dann nicht gescheit
// uploaden
#define L1_EN_PIN LL_GPIO_PIN_1
#define L2_SIG_PIN LL_GPIO_PIN_8
#define L2_EN_PIN LL_GPIO_PIN_2
//
// Current Sensor
//
#define CURRENT_SENSOR_PORT GPIOA
#define CURRENT_SENSOR_PIN LL_GPIO_PIN_0
#define CURRENT_SENSOR_SENSITIVITY 185            // mV/A
#define CURRENT_SENSOR_ADC_FULL_SCALE_VOLTAGE 3.3 // V
#define CURRENT_SENSOR_ADC_FULL_SCALE 0b111111111111

typedef enum {
  STATE_HIGH = 0b11,
  STATE_LOW = 0b10,
  STATE_HIGHZ = 0b00,
} phaseState_t;
struct motorState_s {
  phaseState_t L1;
  phaseState_t L2;
};
typedef struct motorState_s motorState_t;

void setPWMvalue(float pwm);
void writePin(GPIO_TypeDef *port, uint32_t pin, int value);
void setPWMstate(uint32_t channel, int state);
void initMotorStates();
void SetPinsFromState(motorState_t *motorState);
void initADC();
double decodeCurrent(uint16_t adc_value);
void controlCurrent(double current);
void USARTSendBusyWaiting(char *msg, int len);
void initUART();

#endif // !MAIN_H
#define MAIN_H
