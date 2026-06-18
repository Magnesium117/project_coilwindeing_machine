#include "speed_control.h"
#include "encoder.h"
#include "globals.h"
#include "motor_pwm.h"
#include "stm32f446xx.h"
#include "used_stmlibs.h"
#include <stdio.h>
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
static void controlSpeed();
static volatile double voltage = 0;
static volatile double N = 0;
static volatile double prev_N = 0;
static void setPWMvalue(float pwm);
// static void writePin(GPIO_TypeDef *port, uint32_t pin, int value);
static void setPWMstate(uint32_t channel, int state);
static void SetPinsFromState(motorState_t *motorState);
void initSpeedControl() {
  /*
   * COnfig GPIOS For Motor
   */
  LL_GPIO_InitTypeDef GPIO_Initstruct;
  LL_GPIO_StructInit(&GPIO_Initstruct);
  GPIO_Initstruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_NO;
  GPIO_Initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_Initstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  // GPIO_Initstruct.Pin = L1_EN_PIN | L2_EN_PIN;
  // LL_GPIO_Init(EN_PORT, &GPIO_Initstruct);
  GPIO_Initstruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_Initstruct.Alternate = LL_GPIO_AF_2;
  GPIO_Initstruct.Pin = L1_SIG_PIN | L2_SIG_PIN; // | LL_GPIO_PIN_7;
  LL_GPIO_Init(SIG_PORT, &GPIO_Initstruct);
  //
  // Config Timer for controller
  //
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
  LL_TIM_InitTypeDef TIM_InitStruct;
  TIM_InitStruct.Prescaler = 840 - 1; // clk=100kHz -> T=10 us
  TIM_InitStruct.Autoreload = 20 - 1; // T=200us
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM9, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM9);
  LL_TIM_EnableCounter(TIM9);
  // LL_TIM_OC_SetCompareCH1(TIM9, 10);
  // LL_TIM_EnableIT_CC1(TIM9);
  uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, encoded_priority);
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  motorState_t mState;
  mState.L1 = STATE_HIGH;
  mState.L2 = STATE_LOW;
  setPWMvalue(0);
  SetPinsFromState(&mState);
  LL_TIM_EnableIT_UPDATE(TIM9);
  setPWMvalue(1);
}
void TIM1_BRK_TIM9_IRQHandler(void) {
  if (LL_TIM_IsActiveFlag_UPDATE(TIM9)) {
    LL_TIM_ClearFlag_UPDATE(TIM9);
    controlSpeed();
  }
}
void SpeedControlWhile() {
  if (prev_N != N) {
    // printf("    \r\n");
    printf("%d\r\n", (int)N);
    prev_N = N;
  }
}
static void controlSpeed() { N = encoder_get_rpm(); }
static void SetPinsFromState(motorState_t *motorState) {
  // writePin(EN_PORT, L1_EN_PIN, motorState->L1 & 0b10);
  setPWMstate(LL_TIM_CHANNEL_CH2, motorState->L1 & 0b01);

  // writePin(EN_PORT, L2_EN_PIN, motorState->L2 & 0b10);
  setPWMstate(LL_TIM_CHANNEL_CH4, motorState->L2 & 0b01);
}
// Stes pin when anything otheer than 0 is given
static void setPWMstate(uint32_t channel, int state) {
  if (state == 0) {
    LL_TIM_CC_DisableChannel(TIM3, channel);
  } else {
    LL_TIM_CC_EnableChannel(TIM3, channel);
  }
}

static void setPWMvalue(float pwm) {
  if (pwm > 1) {
    pwm = 1;
  }
  if (pwm < 0) {
    pwm = 0;
  }
  int ccr = (int)(PWM_ARR * pwm);
  LL_TIM_OC_SetCompareCH2(TIM3, ccr);
  LL_TIM_OC_SetCompareCH4(TIM3, ccr);
  // LL_TIM_OC_SetCompareCH4(TIM3, ccr);
}
