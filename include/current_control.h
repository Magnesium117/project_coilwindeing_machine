#ifndef CURRENT_CONTROL_H
#define CURRENT_CONTROL_H
#include "used_stmlibs.h"
void init_current_control();
void current_control_while();
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
#define CURRENT_SENSOR_PIN LL_GPIO_PIN_5
#define CURRENT_SENSOR_SENSITIVITY 2              // V/A
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
double decodeCurrent(int32_t adc_value);
void controlCurrent(double current);
void USARTSendBusyWaiting(char *msg, int len);
void initUART();
double lowPass(double input);

#endif
