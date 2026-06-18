#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
//
// Motor Pins
//
#define SIG_PORT GPIOC // Pins 7 9
#define L1_SIG_PIN LL_GPIO_PIN_7
// 13 und 14 sind die JTAG Pins -> man kann dann nicht gescheit
// uploaden
#define L2_SIG_PIN LL_GPIO_PIN_9

void initSpeedControl();
void SpeedControlWhile();
void SetRPM(float setpoint_rpm);
float GetPRM();
#endif //! SPEED_CONTROL_H
#define SPEED_CONTROL_H
