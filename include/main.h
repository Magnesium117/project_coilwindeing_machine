#ifndef MAIN_H
#define MAIN_H
#include "clock_config.h"
#include "used_stmlibs.h"
void USARTSendBusyWaiting(char *msg, int len);
void initUART();
void initDebugPins();
#define USERLED_PORT GPIOA
#define USERLED_PIN LL_GPIO_PIN_5
#define USERBTN_PORT GPIOC
#define USERBTN_PIN LL_GPIO_PIN_13
#endif // !MAIN_H
#define MAIN_H
