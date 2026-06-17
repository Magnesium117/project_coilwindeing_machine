#ifndef HX711_H
#define HX711_H

#include <stdint.h> // Required for int32_t and uint8_t

#define HX711_DATA_PORT GPIOC
#define HX711_DATA1_PIN LL_GPIO_PIN_11
#define HX711_DATA2_PIN LL_GPIO_PIN_10

#define HX711_SCK_PORT GPIOC
#define HX711_SCK_PIN LL_GPIO_PIN_12
// 1. Declare variables as 'extern'
extern volatile int32_t load1;
extern volatile int32_t load2;
extern volatile uint8_t data_ready1;
extern volatile uint8_t data_ready2;

// 2. Declare function prototypes
void hx711_init(void);
void hx711_zero(void);
void hx711While();
#endif /* HX711_H */
