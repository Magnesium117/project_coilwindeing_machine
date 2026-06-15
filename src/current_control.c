#include "current_control.h"
#include "globals.h"
#include "main.h"
#define PWM_ARR 250
#define LOWPASS_ORDER 10
#define SET_CURRENT 250 // mA
#define K_ANTI_WINDUP 1000
static volatile double state[2] = {0};
static volatile double prev_state[1] = {0};
static volatile double voltage_sat_diff = 0;
static void setPWMvalue(float pwm);
// static void writePin(GPIO_TypeDef *port, uint32_t pin, int value);
static void setPWMstate(uint32_t channel, int state);
static void SetPinsFromState(motorState_t *motorState);
static void initADC();
static double decodeCurrent(uint16_t adc_value);
static void controlCurrent(double current);

void initCurrentControl() {
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
  // Configure GPIOS for Current_Sensor
  //
  GPIO_Initstruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_NO;
  GPIO_Initstruct.Pin = CURRENT_SENSOR_PIN;
  LL_GPIO_Init(CURRENT_SENSOR_PORT, &GPIO_Initstruct);
  /*
   * Set Timer for ADC COnversion
   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
  LL_TIM_InitTypeDef TIM_InitStruct;
  TIM_InitStruct.Prescaler = 840 - 1; // clk=100kHz -> T=10 us
  TIM_InitStruct.Autoreload = 20 - 1; // T=200us
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  // LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_EnableCounter(TIM2);
  /*
   * Define PWM Timer
   */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  TIM_InitStruct.Prescaler = 84 - 1;       // clk=1MHz
  TIM_InitStruct.Autoreload = PWM_ARR - 1; // clk=4kHz
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
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct); // PC6
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct); // PC8
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH3);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH3);

  initADC();

  uint32_t encoded_priority = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(EXTI15_10_IRQn, encoded_priority);
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  USERLED_PORT->ODR |= USERLED_PIN;
  motorState_t mState;
  mState.L1 = STATE_HIGH;
  mState.L2 = STATE_LOW;
  setPWMvalue(0);
  SetPinsFromState(&mState);
}
// void current_control_while() {}
static void initADC() {
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  LL_ADC_CommonInitTypeDef ADCCommomInitStruct;
  LL_ADC_CommonStructInit(&ADCCommomInitStruct);
  ADCCommomInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADCCommomInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(ADC123_COMMON, &ADCCommomInitStruct);

  LL_ADC_Disable(ADC1);
  while (LL_ADC_IsEnabled(ADC1))
    ;

  LL_ADC_InitTypeDef ADCInitStruct;
  LL_ADC_StructInit(&ADCInitStruct);
  ADCInitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADCInitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADCInitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADCInitStruct);
  LL_ADC_REG_InitTypeDef ADCRegInitStruct;
  LL_ADC_REG_StructInit(&ADCRegInitStruct);
  ADCRegInitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADCRegInitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADCRegInitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM2_TRGO;
  ADCRegInitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADCRegInitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  LL_ADC_REG_Init(ADC1, &ADCRegInitStruct);
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_5);
  // LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5,
                                LL_ADC_SAMPLINGTIME_480CYCLES);
  // LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6,
  // LL_ADC_SAMPLINGTIME_480CYCLES);

  LL_ADC_EnableIT_EOCS(ADC1);
  uint32_t encoded_priotity = NVIC_EncodePriority(priority_grouping, 0, 0);
  NVIC_SetPriority(ADC_IRQn, encoded_priotity);
  NVIC_EnableIRQ(ADC_IRQn);
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsEnabled(ADC1))
    ;
  LL_ADC_REG_StartConversionExtTrig(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
}

void ADC_IRQHandler() {
  if (LL_ADC_IsActiveFlag_EOCS(ADC1)) {
    LL_ADC_ClearFlag_EOCS(ADC1);
    uint16_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);
    double current = decodeCurrent(adc_value);
    controlCurrent(current);
  }
}
static double decodeCurrent(uint16_t adc_value) {
  double voltage = adc_value * CURRENT_SENSOR_ADC_FULL_SCALE_VOLTAGE /
                   CURRENT_SENSOR_ADC_FULL_SCALE;
  double current = (voltage) / CURRENT_SENSOR_SENSITIVITY;
  return current;
}
static void controlCurrent(double current) {
  double voltage = 0; // Voltage is normalized to 24V
  double voltage_sat = 0;
  current = current * 1e3; // current in mA
  voltage = 0.00021201626441857968 * state[0] +
            -0.00020280817475870627 * state[1] +
            0.00010370610979448802 * (SET_CURRENT - current);
  voltage_sat = voltage;
  if (voltage < 0) {
    voltage_sat = 0;
  } else if (voltage > 1) {
    voltage_sat = 1;
  }
  prev_state[0] = state[0];
  state[0] = 2.0 * state[0] + -1.0 * state[1] + 1.0 * (SET_CURRENT - current) +
             K_ANTI_WINDUP * (voltage_sat_diff);
  voltage_sat_diff = voltage_sat - voltage;
  state[1] = 1.0 * prev_state[0];
  setPWMvalue(voltage);
}

// double lowPass(double input) {
//   lowpass_array[lowpass_index++] = input;
//   if (lowpass_index >= LOWPASS_ORDER) {
//     lowpass_index = 0;
//   }
//   double sum = 0;
//   for (int i = 0; i < LOWPASS_ORDER; i++) {
//     sum += lowpass_array[i];
//   }
//   return sum / LOWPASS_ORDER;
// }

static void SetPinsFromState(motorState_t *motorState) {
  // writePin(EN_PORT, L1_EN_PIN, motorState->L1 & 0b10);
  setPWMstate(LL_TIM_CHANNEL_CH1, motorState->L1 & 0b01);

  // writePin(EN_PORT, L2_EN_PIN, motorState->L2 & 0b10);
  setPWMstate(LL_TIM_CHANNEL_CH3, motorState->L2 & 0b01);
}
// Stes pin when anything otheer than 0 is given
static void setPWMstate(uint32_t channel, int state) {
  if (state == 0) {
    LL_TIM_CC_DisableChannel(TIM3, channel);
  } else {
    LL_TIM_CC_EnableChannel(TIM3, channel);
  }
}
// static void writePin(GPIO_TypeDef *port, uint32_t pin, int value) {
//   if (value == 0) {
//     port->ODR &= ~pin;
//   } else {
//     port->ODR |= pin;
//   }
// }

static void setPWMvalue(float pwm) {
  if (pwm > 1) {
    pwm = 1;
  }
  if (pwm < 0) {
    pwm = 0;
  }
  int ccr = (int)(PWM_ARR * pwm);
  LL_TIM_OC_SetCompareCH1(TIM3, ccr);
  LL_TIM_OC_SetCompareCH3(TIM3, ccr);
  // LL_TIM_OC_SetCompareCH4(TIM3, ccr);
}
