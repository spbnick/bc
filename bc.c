/*
 * Business Card Synth
 */
#include <init.h>
#include <rcc.h>
#include <gpio.h>

int
main(void)
{
    volatile unsigned int i;

    /* Basic init */
    init();

    /* Enable clock to GPIO port B */
    RCC->iopenr |= RCC_IOPENR_IOPBEN_MASK;

    /* Configure the PB8 pin for push-pull output */
    GPIO_B->otyper = (GPIO_B->otyper & ~GPIO_OTYPER_OT8_MASK) |
                     (GPIO_OTYPE_PUSH_PULL << GPIO_OTYPER_OT8_LSB);
    GPIO_B->moder = (GPIO_B->moder & ~GPIO_MODER_MODE8_MASK) |
                    (GPIO_MODE_OUTPUT << GPIO_MODER_MODE8_LSB);

    /* Configure the PB9 pin for fast push-pull output */
    GPIO_B->otyper = (GPIO_B->otyper & ~GPIO_OTYPER_OT9_MASK) |
                     (GPIO_OTYPE_PUSH_PULL << GPIO_OTYPER_OT9_LSB);
    GPIO_B->moder = (GPIO_B->moder & ~GPIO_MODER_MODE9_MASK) |
                    (GPIO_MODE_OUTPUT << GPIO_MODER_MODE9_LSB);
    GPIO_B->ospeedr = (GPIO_B->ospeedr & ~GPIO_OSPEEDR_OSPEED9_MASK) |
                      (GPIO_OSPEED_HIGH << GPIO_OSPEEDR_OSPEED9_LSB);

    /* Set PB8 high */
    GPIO_B->odr |= GPIO_ODR_OD8_MASK;
    /* Forever */
    while (1) {
        /* Wait */
        for (i = 0; i < 286; i++);
        /* Toggle PB9 */
        GPIO_B->odr ^= GPIO_ODR_OD9_MASK;
        /* Toggle PB8 */
        GPIO_B->odr ^= GPIO_ODR_OD8_MASK;
    }
}
