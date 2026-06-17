#include "motor_pwm.h"
#include "globals.h"
#include "used_stmlibs.h"
#define PWM_FREQ 10000000 // TIMER frequence of PWM

void initPwm() {
  /*
   * Define PWM Timer
   */
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  TIM_InitStruct.Prescaler = SYSCLK / PWM_FREQ - 1; // clk=10MHz
  TIM_InitStruct.Autoreload = PWM_ARR - 1;          // clk=40kHz
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.CompareValue = PWM_ARR / 2;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  // PC6 -> Current control
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  // PC8 -> current control
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH3);
  // PC7 -> speed_control
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  // PC9 -> speed_control
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
}
