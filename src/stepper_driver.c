#include "stepper_driver.h"
#include "globals.h"
#include "stm32f446xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_utils.h"
#include <stdint.h>
#define HALFSTEPPING
#ifdef HALFSTEPPING
#define N_MOTOR_STATES 8
#endif                /* ifdef HALFSTEPPING */
#define STEP_TIME 500 // us
#define STATE_CHANGE_CLOCK 10000
typedef enum {
  COIL1_POSITIVE = ((0xFFFFFFFF << 16) | PIN_11) & ~(PIN_12 << 16),
  COIL1_NEGATIVE = ((0xFFFFFFFF << 16) | PIN_12) & ~(PIN_11 << 16),
  COIL1_OFF = (0xFFFFFFFF << 16) & ~((PIN_11 | PIN_12) << 16),
  COIL2_POSITIVE = ((0xFFFFFFFF << 16) | PIN_21) & ~(PIN_22 << 16),
  COIL2_NEGATIVE = ((0xFFFFFFFF << 16) | PIN_22) & ~(PIN_21 << 16),
  COIL2_OFF = (0xFFFFFFFF << 16) & ~((PIN_21 | PIN_22) << 16),

} coilState_t;

typedef struct {
  coilState_t Coil1;
  coilState_t Coil2;
} motorState_t;

static volatile int nSteps = 0;
static volatile int steps = 0;
static motorState_t MotorStates[N_MOTOR_STATES];
static volatile int state_counter = 0;

static void initMotorStates();
static void SetPinsFromState(motorState_t *motorState);

void initStepperDriver() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  initMotorStates();
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_GPIO_InitTypeDef GPIO_Initstruct;
  LL_GPIO_StructInit(&GPIO_Initstruct);
  /*
   * Config GPIOS For Motor
   */
  GPIO_Initstruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_NO;
  GPIO_Initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_Initstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_Initstruct.Pin = PIN_11 | PIN_12;
  LL_GPIO_Init(COIL1_PORT, &GPIO_Initstruct);
  GPIO_Initstruct.Mode = LL_GPIO_MODE_OUTPUT;
  // GPIO_Initstruct.Alternate = LL_GPIO_AF_2;
  GPIO_Initstruct.Pin = PIN_21 | PIN_22;
  LL_GPIO_Init(COIL2_PORT, &GPIO_Initstruct);
  /*
   * Set Timer for state change
   */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_TIM_InitTypeDef TIM_InitStruct;
  TIM_InitStruct.Prescaler =
      SYSCLK / STATE_CHANGE_CLOCK - 1; // clk=100kHz -> T=10 us was: (840 - 1)
  TIM_InitStruct.Autoreload =
      STATE_CHANGE_CLOCK * STEP_TIME / 1e6 - 1; // was 50
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_DisableCounter(TIM1);
  NVIC_SetPriorityGrouping(priority_grouping);
  uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, encoded_priority);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  SetPinsFromState(&MotorStates[state_counter]);
  while (1) {
  }
}
static void initMotorStates() {
  // Halfstepping
  //  state 1
  MotorStates[0].Coil1 = COIL1_POSITIVE;
  MotorStates[0].Coil2 = COIL2_POSITIVE;
  // state 2
  MotorStates[1].Coil1 = COIL1_POSITIVE;
  MotorStates[1].Coil2 = COIL2_OFF;
  // state 3
  MotorStates[2].Coil1 = COIL1_POSITIVE;
  MotorStates[2].Coil2 = COIL2_NEGATIVE;
  // state 4
  MotorStates[3].Coil1 = COIL1_OFF;
  MotorStates[3].Coil2 = COIL2_NEGATIVE;
  // state 5
  MotorStates[4].Coil1 = COIL1_NEGATIVE;
  MotorStates[4].Coil2 = COIL2_NEGATIVE;
  // state 6
  MotorStates[5].Coil1 = COIL1_NEGATIVE;
  MotorStates[5].Coil2 = COIL2_OFF;
  // state 7
  MotorStates[6].Coil1 = COIL1_NEGATIVE;
  MotorStates[6].Coil2 = COIL2_POSITIVE;
  // state 8
  MotorStates[7].Coil1 = COIL1_OFF;
  MotorStates[7].Coil2 = COIL2_POSITIVE;
}
void Step(int N) { nSteps += N; }
void TIM1_UP_TIM10_IRQHandler() {
  if (LL_TIM_IsActiveFlag_UPDATE(TIM1)) {
    LL_TIM_ClearFlag_UPDATE(TIM1);
    if (steps < nSteps) {
      state_counter += 1;
      if (state_counter >= N_MOTOR_STATES) {
        state_counter = 0;
      }
      SetPinsFromState(&MotorStates[state_counter]);
      steps++;
    } else if (steps > nSteps) {
      state_counter -= 1;
      if (state_counter < 0) {
        state_counter = N_MOTOR_STATES - 1;
      }
      SetPinsFromState(&MotorStates[state_counter]);
      steps--;
    } else {
      steps = 0;
      nSteps = 0;
    }
  }
}
static void SetPinsFromState(motorState_t *motorState) {
  COIL1_PORT->ODR |= motorState->Coil1 & 0xFFFF;
  COIL2_PORT->ODR |= motorState->Coil2 & 0xFFFF;
  COIL1_PORT->ODR &= (motorState->Coil1 >> 16) & 0xFFFF;
  COIL2_PORT->ODR &= (motorState->Coil2 >> 16) & 0xFFFF;
}
