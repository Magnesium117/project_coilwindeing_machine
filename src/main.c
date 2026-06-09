#include "main.h"
#include "current_control.h"
#include "globals.h"
#include "stm32f446xx.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_utils.h"
#include <stdint.h>
#include <stdio.h>
int main() {
  //
  // Configure Clock
  //
  // sysclk=84MHz
  SystemClock_Config();
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  //
  // Set NVIC Priority Grouping
  //
  NVIC_SetPriorityGrouping(priority_grouping);
  initUART();
  initDebugPins();
  init_current_control();
  while (1) {
  }
}
void initDebugPins() {
  //
  // Enable DEBUG GPIOS
  //
  LL_GPIO_InitTypeDef GPIO_Initstruct;
  LL_GPIO_StructInit(&GPIO_Initstruct);
  GPIO_Initstruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_Initstruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_Initstruct.Pin = USERLED_PIN;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_NO;
  GPIO_Initstruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(USERLED_PORT, &GPIO_Initstruct);

  GPIO_Initstruct.Pin = USERBTN_PIN;
  GPIO_Initstruct.Pull = LL_GPIO_PULL_UP;
  GPIO_Initstruct.Mode = LL_GPIO_MODE_INPUT;
  LL_GPIO_Init(USERBTN_PORT, &GPIO_Initstruct);
  //
  // Configure "Heartbeat" button Interrupt
  //
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  EXTI_InitStruct.LineCommand = ENABLE;
  LL_EXTI_Init(&EXTI_InitStruct);
}
void EXTI15_10_IRQHandler() {
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13)) {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    USERLED_PORT->ODR ^= USERLED_PIN;
    while (!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, 'A');
  }
}
void initUART() {
  LL_USART_InitTypeDef USARTInitStruct = {0};
  LL_GPIO_InitTypeDef GPIOInitStruct = {0};
  LL_USART_StructInit(&USARTInitStruct);
  LL_GPIO_StructInit(&GPIOInitStruct);

  /*
      clocks
  */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  /*
      PA2 -> USART2_TX
      PA3 -> USART2_RX
  */

  GPIOInitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIOInitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIOInitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIOInitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIOInitStruct.Pull = LL_GPIO_PULL_UP;
  GPIOInitStruct.Alternate = LL_GPIO_AF_7;

  LL_GPIO_Init(GPIOA, &GPIOInitStruct);

  /*
      USART config
  */

  USARTInitStruct.BaudRate = 115200;
  USARTInitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USARTInitStruct.StopBits = LL_USART_STOPBITS_1;
  USARTInitStruct.Parity = LL_USART_PARITY_NONE;
  USARTInitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USARTInitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USARTInitStruct.OverSampling = LL_USART_OVERSAMPLING_16;

  LL_USART_Init(USART2, &USARTInitStruct);

  LL_USART_Enable(USART2);
}

void USARTSendBusyWaiting(char *msg, int len) {
  for (int i = 0; i < len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, msg[i]);
  }
}
// hopefully makes USART work with printf
int _write(int file, char *ptr, int len) {
  for (int i = 0; i < len; i++) {
    while (!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, ptr[i]);
  }
  return len;
}
