#include "stepper_driver.h"
#include "globals.h"
#include "stm32f446xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_utils.h"
#include "used_stmlibs.h"
#include <stdint.h>
#define PWM_ARR 250
#define PWM_FREQ 10000000 // #Hz //TIMER frequence of PWM
#define PWM_ON 70
#define HALFSTEPPING
#ifdef HALFSTEPPING
#define N_MOTOR_STATES 8
#endif                 /* ifdef HALFSTEPPING */
#define STEP_TIME 1000 // us
#define STATE_CHANGE_CLOCK 10000
// Optimised typedef for non pwm mode
// typedef enum {
//   COIL1_POSITIVE = ((0xFFFFFFFF << 16) | PIN_11) & ~(PIN_12 << 16),
//   COIL1_NEGATIVE = ((0xFFFFFFFF << 16) | PIN_12) & ~(PIN_11 << 16),
//   COIL1_OFF = (0xFFFFFFFF << 16) & ~((PIN_11 | PIN_12) << 16),
//   COIL2_POSITIVE = ((0xFFFFFFFF << 16) | PIN_21) & ~(PIN_22 << 16),
//   COIL2_NEGATIVE = ((0xFFFFFFFF << 16) | PIN_22) & ~(PIN_21 << 16),
//   COIL2_OFF = (0xFFFFFFFF << 16) & ~((PIN_21 | PIN_22) << 16),
//
// } coilState_t;
typedef enum {
  COIL_POSITIVE = 0b10,
  COIL_NEGATIVE = 0b01,
  COIL_OFF = 0b00,
} coilState_t;

typedef struct {
  coilState_t Coil1;
  coilState_t Coil2;
} motorState_t;

static volatile int nSteps = 0;
static volatile int steps = 0;
static volatile uint32_t position_um = 0;
static motorState_t MotorStates[N_MOTOR_STATES];
static volatile int state_counter = 0;
static volatile int in_home_position = 0;

static void initMotorStates();
static void SetPinsFromState(motorState_t *motorState);
static void initHomingExti();

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
  GPIO_Initstruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_NO;
  GPIO_Initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_Initstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_Initstruct.Pin = PIN_11 | PIN_12;
  GPIO_Initstruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(COIL1_PORT, &GPIO_Initstruct);
  // GPIO_Initstruct.Mode = LL_GPIO_MODE_OUTPUT;
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
  LL_TIM_EnableCounter(TIM1);
  /*
   * Define PWM Timer
   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
  TIM_InitStruct.Prescaler = SYSCLK / PWM_FREQ - 1; // clk=10MHz
  TIM_InitStruct.Autoreload = PWM_ARR - 1;          // clk=40kHz
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM4);
  LL_TIM_EnableCounter(TIM4);
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.CompareValue = PWM_ON;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  // PB6 -> Stepper PWM
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  // PB7 -> Stepper PWM
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH2);
  // PB8 -> Stepper PWM
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  // PB9 -> Stepper PWM
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH4);

  NVIC_SetPriorityGrouping(priority_grouping);
  uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, encoded_priority);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  SetPinsFromState(&MotorStates[state_counter]);
  initHomingExti();
}
static void initHomingExti() {
  LL_GPIO_InitTypeDef GPIO_Initstruct;
  LL_GPIO_StructInit(&GPIO_Initstruct);
  GPIO_Initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_Initstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_Initstruct.Pin = HOMING_PIN;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_UP;
  GPIO_Initstruct.Mode = LL_GPIO_MODE_INPUT;
  LL_GPIO_Init(HOMING_PORT, &GPIO_Initstruct);
  //
  // Configure Homing interrupt
  //
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE7);
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_7;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  EXTI_InitStruct.LineCommand = ENABLE;
  LL_EXTI_Init(&EXTI_InitStruct);
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
  uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(EXTI9_5_IRQn, encoded_priority);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
}
static void initMotorStates() {
  // Halfstepping
  //  state 1
  MotorStates[0].Coil1 = COIL_POSITIVE;
  MotorStates[0].Coil2 = COIL_POSITIVE;
  // state 2
  MotorStates[1].Coil1 = COIL_POSITIVE;
  MotorStates[1].Coil2 = COIL_OFF;
  // state 3
  MotorStates[2].Coil1 = COIL_POSITIVE;
  MotorStates[2].Coil2 = COIL_NEGATIVE;
  // state 4
  MotorStates[3].Coil1 = COIL_OFF;
  MotorStates[3].Coil2 = COIL_NEGATIVE;
  // state 5
  MotorStates[4].Coil1 = COIL_NEGATIVE;
  MotorStates[4].Coil2 = COIL_NEGATIVE;
  // state 6
  MotorStates[5].Coil1 = COIL_NEGATIVE;
  MotorStates[5].Coil2 = COIL_OFF;
  // state 7
  MotorStates[6].Coil1 = COIL_NEGATIVE;
  MotorStates[6].Coil2 = COIL_POSITIVE;
  // state 8
  MotorStates[7].Coil1 = COIL_OFF;
  MotorStates[7].Coil2 = COIL_POSITIVE;
}
void Step(int N) { nSteps += N; }
void EXTI9_5_IRQHandler() {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7)) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    in_home_position = 1;
    position_um = 0;
  }
}
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
      position_um += GANTRY_UM_PER_STEP;
    } else if (steps > nSteps) {
      state_counter -= 1;
      if (state_counter < 0) {
        state_counter = N_MOTOR_STATES - 1;
      }
      SetPinsFromState(&MotorStates[state_counter]);
      steps--;
      position_um -= GANTRY_UM_PER_STEP;
    } else {
      steps = 0;
      nSteps = 0;
    }
  }
}
void stepper_home() {
  while (!in_home_position) {
    nSteps = -1;
  }
}
void stepper_goto(uint32_t set_position_um) {
  int64_t distance = (int64_t)set_position_um - (int64_t)position_um;
  nSteps = 0;
  Step(distance / GANTRY_UM_PER_STEP);
}
static void SetPinsFromState(motorState_t *motorState) {
  if ((motorState->Coil1 & 0b10) != 0) {
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  } else {
    LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  }
  if ((motorState->Coil1 & 0b01) != 0) {
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
  } else {
    LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH2);
  }

  if ((motorState->Coil2 & 0b10) != 0) {
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  } else {
    LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  }

  if ((motorState->Coil2 & 0b01) != 0) {
    LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);
  } else {
    LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH4);
  }
}
