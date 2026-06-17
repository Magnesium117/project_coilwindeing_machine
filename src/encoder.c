#include "encoder.h"

#include "globals.h"
#include "stepper_driver.h"
#include "stm32f446xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_tim.h"
#include <stdio.h>

#define ENCODER_PORT GPIOA
#define ENCODER_PIN_A LL_GPIO_PIN_0

/*
 * Mindestabstand zwischen zwei gültigen Flanken.
 * Bei 250 rpm und 963 Schlitzen ist die Periodendauer ungefähr 249 us.
 * 20 us filtert Störimpulse, ohne echte Pulse zu verwerfen.
 */
#define MIN_EDGE_DISTANCE_US 1u

/* Nach dieser Zeit ohne Puls wird Drehzahl = 0 angenommen. */
#define SPEED_TIMEOUT_US 500000u

#define TWO_PI 6.28318530718f
/*
 * Winding/Gantry configuration
 *
 * GANTRY_STEPS_PER_REVOLUTION:
 *   Relative gantry motion commanded after each spindle revolution.
 *
 * GANTRY_TRAVEL_STEPS:
 *   Absolute commanded travel before the gantry direction is reversed.
 *
 * WINDING_TARGET_REVOLUTIONS:
 *   0 = endless winding. Otherwise stop commanding new gantry steps after this
 *   many spindle revolutions.
 */

// 1 Step = 0,6mm
#define GANTRY_STEPS_PER_REVOLUTION 3
#define GANTRY_TRAVEL_STEPS 30
#define WINDING_TARGET_REVOLUTIONS 0u
#define WINDING_LOG_EACH_REVOLUTION 1
#define GANTRY_REVOLUTIONS_TO_STEP 1

static int32_t gantry_position_steps = 0;
static int8_t gantry_direction = 1;
static uint8_t winding_active = 1;

static void winding_on_revolution(void);

static volatile uint32_t edge_count = 0;
static volatile uint32_t revolution_count = 0;
static volatile uint8_t revolution_event = 0;

static volatile uint32_t last_edge_time_us = 0;
static volatile uint32_t last_period_us = 0;

static volatile float rps = 0.0f;

/* Moving Average über ungefähr eine Umdrehung. */
static volatile uint32_t period_buffer[ENCODER_SLOTS_PER_REVOLUTION];
static volatile uint32_t period_sum_us = 0;
static volatile uint16_t period_index = 0;
static volatile uint16_t period_count = 0;

static uint32_t get_time_us(void) { return LL_TIM_GetCounter(TIM5); }
static uint32_t last_rev_print = 0;

static void period_average_add(uint32_t dt_us)
{
  if (period_count >= ENCODER_SLOTS_PER_REVOLUTION)
  {
    period_sum_us -= period_buffer[period_index];
  }
  else
  {
    period_count++;
  }

  period_buffer[period_index] = dt_us;
  period_sum_us += dt_us;

  period_index++;
  if (period_index >= ENCODER_SLOTS_PER_REVOLUTION)
  {
    period_index = 0;
  }
}

static void encoder_gpio_exti_init(void)
{
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  LL_GPIO_InitTypeDef gpio;
  LL_GPIO_StructInit(&gpio);
  gpio.Pin = ENCODER_PIN_A;
  gpio.Mode = LL_GPIO_MODE_INPUT;
  gpio.Pull = LL_GPIO_PULL_NO;
  gpio.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(ENCODER_PORT, &gpio);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  LL_EXTI_InitTypeDef exti;
  exti.Line_0_31 = LL_EXTI_LINE_0;
  exti.Mode = LL_EXTI_MODE_IT;
  exti.Trigger = LL_EXTI_TRIGGER_RISING;
  exti.LineCommand = ENABLE;
  LL_EXTI_Init(&exti);

  /* Sicherstellen, dass kein alter Pending-Interrupt sofort auslöst. */
  LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

  uint32_t prio = NVIC_EncodePriority(priority_grouping, 1, 0);
  NVIC_SetPriority(EXTI0_IRQn, prio);
  NVIC_EnableIRQ(EXTI0_IRQn);
}

static void encoder_timebase_init(void)
{
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  LL_TIM_InitTypeDef tim;
  LL_TIM_StructInit(&tim);
  tim.Prescaler = (SYSCLK / 1000000u) - 1u; /* 84 MHz -> 1 MHz */
  tim.Autoreload = 0xFFFFFFFFu;
  tim.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &tim);

  LL_TIM_SetCounter(TIM5, 0);
  LL_TIM_EnableCounter(TIM5);
}

void encoder_init(void)
{
  edge_count = 0;
  revolution_count = 0;
  revolution_event = 0;

  last_edge_time_us = 0;
  last_period_us = 0;
  rps = 0.0f;

  period_sum_us = 0;
  period_index = 0;
  period_count = 0;
  for (uint16_t i = 0; i < ENCODER_SLOTS_PER_REVOLUTION; i++)
  {
    period_buffer[i] = 0;
  }

  encoder_timebase_init();
  encoder_gpio_exti_init();
}

/*
 * Wird ausschließlich aus EXTI0_IRQHandler() aufgerufen.
 * Absichtlich kurz halten: keine UART-Ausgabe, kein printf, keine langen Loops.
 */
void encoder_on_edge_a(void)
{
  uint32_t now = get_time_us();

  if (last_edge_time_us != 0)
  {
    uint32_t dt_us = now - last_edge_time_us;

    if (dt_us < MIN_EDGE_DISTANCE_US)
    {
      return;
    }

    last_period_us = dt_us;

    float pulse_frequency_hz = 1000000.0f / (float)dt_us;
    rps = pulse_frequency_hz / ENCODER_SLOTS_PER_REVOLUTION_F;

    period_average_add(dt_us);
  }

  last_edge_time_us = now;
  edge_count++;

  if ((edge_count % ENCODER_SLOTS_PER_REVOLUTION) == 0u)
  {
    revolution_count++;
    revolution_event = 1u;
  }
}

uint32_t encoder_get_edge_count(void) { return edge_count; }

uint32_t encoder_get_revolution_count(void) { return revolution_count; }

uint8_t encoder_take_revolution_event(void)
{
  if (revolution_event)
  {
    revolution_event = 0u;
    return 1u;
  }
  return 0u;
}

uint32_t encoder_get_last_period_us(void) { return last_period_us; }

float encoder_get_rps(void)
{
  uint32_t now = get_time_us();

  if (last_edge_time_us == 0u)
  {
    return 0.0f;
  }

  if ((now - last_edge_time_us) > SPEED_TIMEOUT_US)
  {
    return 0.0f;
  }

  return rps;
}

float encoder_get_rpm(void) { return encoder_get_rps() * 60.0f; }

float encoder_get_omega_rad_s(void) { return TWO_PI * encoder_get_rps(); }

float encoder_get_rps_avg(void)
{
  uint32_t now = get_time_us();

  if (last_edge_time_us == 0u)
  {
    return 0.0f;
  }

  if ((now - last_edge_time_us) > SPEED_TIMEOUT_US)
  {
    return 0.0f;
  }

  if (period_count == 0u || period_sum_us == 0u)
  {
    return 0.0f;
  }

  float avg_period_us = (float)period_sum_us / (float)period_count;
  float avg_pulse_frequency_hz = 1000000.0f / avg_period_us;
  return avg_pulse_frequency_hz / ENCODER_SLOTS_PER_REVOLUTION_F;
}

float encoder_get_rpm_avg(void) { return encoder_get_rps_avg() * 60.0f; }

float encoder_get_omega_rad_s_avg(void)
{
  return TWO_PI * encoder_get_rps_avg();
}
void encoderWhile()
{
  /* One event is generated by encoder.c after every full spindle revolution. */
  if (encoder_take_revolution_event())
  {
    winding_on_revolution();
  }
}
static void winding_on_revolution(void)
{
  if (!winding_active)
  {
    return;
  }

  uint32_t rev = encoder_get_revolution_count();

  if (!(rev % GANTRY_REVOLUTIONS_TO_STEP))
  {

    if ((WINDING_TARGET_REVOLUTIONS != 0u) &&
        (rev >= WINDING_TARGET_REVOLUTIONS))
    {
      winding_active = 0u;
      return;
    }

    int32_t step_command = gantry_direction * GANTRY_STEPS_PER_REVOLUTION;

    /*
     * Erst den Schritt ausführen.
     * Danach die interne Position aktualisieren.
     */
    Step(step_command);
    gantry_position_steps += step_command;

    /*
     * Erst NACH dem Schritt prüfen, ob die Grenze erreicht wurde.
     */
    if (gantry_position_steps >= GANTRY_TRAVEL_STEPS)
    {
      gantry_position_steps = GANTRY_TRAVEL_STEPS;
      gantry_direction = -1;
    }
    else if (gantry_position_steps <= 0)
    {
      gantry_position_steps = 0;
      gantry_direction = 1;
    }

#if WINDING_LOG_EACH_REVOLUTION
    printf("REV=%lu GANTRY=%ld DIR=%d\r\n",
           (unsigned long)rev,
           (long)gantry_position_steps,
           (int)gantry_direction);
#endif
  }
}
