#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/*
 * Lichtschranke A liegt auf PA0 / EXTI0.
 * Gezählt wird eine Flanke pro Schlitz.
 * Die Zeitbasis läuft über TIM5 mit 1 MHz, damit TIM2 für die Stromregelung
 * unverändert bleiben kann.
 */

#define ENCODER_SLOTS_PER_REVOLUTION 963u
#define ENCODER_SLOTS_PER_REVOLUTION_F 963.0f

void encoder_init(void);
void encoder_on_edge_a(void);

uint32_t encoder_get_edge_count(void);
uint32_t encoder_get_revolution_count(void);
uint8_t encoder_take_revolution_event(void);

uint32_t encoder_get_last_period_us(void);

float encoder_get_rps(void);
float encoder_get_rpm(void);
float encoder_get_omega_rad_s(void);

float encoder_get_rps_avg(void);
float encoder_get_rpm_avg(void);
float encoder_get_omega_rad_s_avg(void);
void encoderWhile();

#endif /* ENCODER_H */
