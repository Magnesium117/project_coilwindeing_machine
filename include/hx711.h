#ifndef HX711_H
#define HX711_H

#include <stdint.h> // Required for int32_t and uint8_t


// 1. Declare variables as 'extern'
extern volatile int32_t load1;
extern volatile int32_t load2;
extern volatile uint8_t data_ready1;
extern volatile uint8_t data_ready2;

// 2. Declare function prototypes
void hx711_init(void);


#endif /* HX711_H */