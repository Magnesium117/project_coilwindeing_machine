#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_tim.h"
#include "hx711.h"

#define HX711_DATA_PORT  GPIOC
#define HX711_DATA1_PIN  LL_GPIO_PIN_11
#define HX711_DATA2_PIN  LL_GPIO_PIN_10

#define HX711_SCK_PORT   GPIOC
#define HX711_SCK_PIN    LL_GPIO_PIN_12

volatile int32_t load1 = 0;
volatile int32_t load2 = 0;

volatile uint8_t data_ready1 = 0;
volatile uint8_t data_ready2 = 0;

volatile int32_t prev_load1 = 0;
volatile int32_t prev_load2 = 0;

static volatile uint8_t hx_reading = 0;
static volatile uint8_t hx_clock_cycles = 0;

static volatile uint32_t hx_temp_data1 = 0;
static volatile uint32_t hx_temp_data2 = 0;

void hx711_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

    /* SCK output */
    GPIO_InitStruct.Pin = HX711_SCK_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(HX711_SCK_PORT, &GPIO_InitStruct);

    LL_GPIO_ResetOutputPin(HX711_SCK_PORT, HX711_SCK_PIN);

    /* DATA1 input */
    GPIO_InitStruct.Pin = HX711_DATA1_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HX711_DATA_PORT, &GPIO_InitStruct);

    /* DATA2 input */
    GPIO_InitStruct.Pin = HX711_DATA2_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(HX711_DATA_PORT, &GPIO_InitStruct);

    /*
     * TIM1 configuration
     *
     * TIM1 clock = 180 MHz
     * PSC = 179 -> 1 MHz timer clock
     * ARR = 49999 -> Update every 50 ms
     */
    LL_TIM_SetPrescaler(TIM1, 180 - 1);
    LL_TIM_SetAutoReload(TIM1, 50000);
    LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);

    /*
     * CC1 interrupt every ~1 us while reading
     */
    LL_TIM_OC_SetCompareCH1(TIM1, 1);
    LL_TIM_OC_SetCompareCH2(TIM1, 50000-1);
    //LL_TIM_EnableIT_UPDATE(TIM1);

    //NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);
    //NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    LL_TIM_ClearFlag_CC2(TIM1);
    LL_TIM_EnableIT_CC2(TIM1);

    NVIC_SetPriority(TIM1_CC_IRQn, 2);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    LL_TIM_EnableCounter(TIM1);
}

/*
 * Called every 50 ms.
 * Starts a read only when BOTH HX711s are ready.
 */
/*void TIM1_UP_TIM10_IRQHandler(void)
{
    if (!LL_TIM_IsActiveFlag_UPDATE(TIM1))
        return;

    LL_TIM_ClearFlag_UPDATE(TIM1);

    if (hx_reading)
        return;

    if (!LL_GPIO_IsInputPinSet(HX711_DATA_PORT, HX711_DATA1_PIN) &&
        !LL_GPIO_IsInputPinSet(HX711_DATA_PORT, HX711_DATA2_PIN))
    {
        hx_reading = 1;

        hx_clock_cycles = 0;

        hx_temp_data1 = 0;
        hx_temp_data2 = 0;

        LL_TIM_ClearFlag_CC1(TIM1);
        LL_TIM_SetCounter(TIM1, 0);

        LL_TIM_EnableIT_CC1(TIM1);
    }
}
*/
/*
 * Reads both HX711 devices simultaneously using the same SCK.
 *
 * 24 bits = 48 edges
 * +1 gain-select pulse = 2 edges
 * Total = 50 interrupt cycles
 */
void TIM1_CC_IRQHandler(void)
{  if (LL_TIM_IsActiveFlag_CC2(TIM1))
    {   
        LL_TIM_ClearFlag_CC2(TIM1);
        if (hx_reading)
        return;

    if (!LL_GPIO_IsInputPinSet(HX711_DATA_PORT, HX711_DATA1_PIN) &&
        !LL_GPIO_IsInputPinSet(HX711_DATA_PORT, HX711_DATA2_PIN))
    {
        hx_reading = 1;
        hx_clock_cycles = 0;
        hx_temp_data1 = 0;
        hx_temp_data2 = 0;

        /* prepare CC1 cycle */
        LL_TIM_ClearFlag_CC1(TIM1);
        LL_TIM_SetCounter(TIM1, 0);
        LL_TIM_EnableIT_CC1(TIM1);
    }
    }
    else if (LL_TIM_IsActiveFlag_CC1(TIM1))
    {
        LL_TIM_ClearFlag_CC1(TIM1);

        /* Schedule next interrupt ~1 us later */
        LL_TIM_SetCounter(TIM1, 0);

        if (hx_clock_cycles < 48)
        {
            if ((hx_clock_cycles & 1U) == 0U)
            {
                /* Rising edge */
                LL_GPIO_SetOutputPin(HX711_SCK_PORT,
                                    HX711_SCK_PIN);
            }
            else
            {
                /* Falling edge */

                LL_GPIO_ResetOutputPin(HX711_SCK_PORT,
                                    HX711_SCK_PIN);

                hx_temp_data1 <<= 1;
                hx_temp_data2 <<= 1;

                if (LL_GPIO_IsInputPinSet(HX711_DATA_PORT,
                                        HX711_DATA1_PIN))
                {
                    hx_temp_data1 |= 1U;
                }

                if (LL_GPIO_IsInputPinSet(HX711_DATA_PORT,
                                        HX711_DATA2_PIN))
                {
                    hx_temp_data2 |= 1U;
                }
            }
        }
        else if (hx_clock_cycles < 50)
        {
            /* 25th pulse: gain = 128 */

            if ((hx_clock_cycles & 1U) == 0U)
            {
                LL_GPIO_SetOutputPin(HX711_SCK_PORT,
                                    HX711_SCK_PIN);
            }
            else
            {
                LL_GPIO_ResetOutputPin(HX711_SCK_PORT,
                                    HX711_SCK_PIN);
            }
        }
        else
        {
            LL_TIM_DisableIT_CC1(TIM1);

            /* Sign extend DATA1 */
            if (hx_temp_data1 & 0x800000UL)
            {
                hx_temp_data1 |= 0xFF000000UL;
            }

            /* Sign extend DATA2 */
            if (hx_temp_data2 & 0x800000UL)
            {
                hx_temp_data2 |= 0xFF000000UL;
            }

            load1 = (int32_t)hx_temp_data1;
            load2 = (int32_t)hx_temp_data2;

            data_ready1 = 1;
            data_ready2 = 1;

            hx_reading = 0;

            return;
        }

        hx_clock_cycles++;
    }
    else
    {return 0;}
}