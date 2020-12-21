/*
 * Business Card Synth
 */
#include <init.h>
#include <rcc.h>
#include <nvic.h>
#include <gpio.h>
#include <usart.h>
#include <tsc.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize the debug USART to 115200 8N1.
 */
static void
usart_init(void)
{
    /* Configure TX (PA14) and RX (PA15) pins */
    gpio_pin_conf_af(GPIO_A, 14, 4, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_A, 15, 4, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);

    /* Enable clock to USART2 */
    RCC->apb1enr |= RCC_APB1ENR_USART2EN_MASK;

    /* Set baud rate to ~115200, accounting for Fck = 32MHz */
    USART2->brr = (USART2->brr & (~USART_BRR_BRR_MASK)) | 0x116;

    /* Enable USART, leave the default mode of 8N1 */
    USART2->cr1 |= USART_CR1_UE_MASK;

    /* Enable transmitter, sending an idle frame */
    USART2->cr1 |= USART_CR1_TE_MASK;
}

/**
 * Print a formatted message to debug USART (255 characters max).
 *
 * @param fmt   Format string for the message.
 * @param ...   Format arguments for the message.
 */
static void
usart_printf(const char *fmt, ...)
{
    char buf[256];
    const char *p;
    va_list ap;

    /* Format the message */
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* Output the message */
    for (p = buf; *p != 0; p++) {
        /* Wait for transmit register to empty */
        while (!(USART2->isr & USART_ISR_TXE_MASK));
        /* Write the byte */
        USART2->tdr = *p;
    }
    /* Wait for the transfer to complete, if done any */
    if (p > buf)
        while (!(USART2->isr & USART_ISR_TC_MASK));
}

/** Touch sense controller acquisition step description */
struct tsc_step {
    unsigned int iogcsr_mask;   /**< The IOGCSR mask for this step */
    unsigned int iogcsr_value;  /**< The IOGCSR value for this step */
    unsigned int ioccr_mask;    /**< The IOCCR mask for this step */
    unsigned int ioccr_value;   /**< The IOCCR value for this step */
    /**
     * A list of group number / value index pairs indicating which groups
     * (1-8) are acquiring values during this step, and where to write them
     * in the acquired value array. Terminated with a {0, X} pair.
     */
    size_t group_list[3][2];
};

const struct tsc_step TSC_STEP_LIST[] = {
    {
        .iogcsr_mask = TSC_IOGCSR_G4E_MASK | TSC_IOGCSR_G5E_MASK,
        .iogcsr_value = TSC_IOGCSR_G4E_MASK | TSC_IOGCSR_G5E_MASK,
        .ioccr_mask = TSC_IOCCR_G4_IO3_MASK | TSC_IOCCR_G4_IO4_MASK |
                      TSC_IOCCR_G5_IO2_MASK | TSC_IOCCR_G5_IO3_MASK |
                      TSC_IOCCR_G5_IO4_MASK,
        .ioccr_value = TSC_IOCCR_G4_IO3_MASK | TSC_IOCCR_G5_IO2_MASK,
        .group_list = {{4, 0}, {5, 2}},
    },
    {
        .iogcsr_mask = TSC_IOGCSR_G4E_MASK | TSC_IOGCSR_G5E_MASK,
        .iogcsr_value = TSC_IOGCSR_G4E_MASK | TSC_IOGCSR_G5E_MASK,
        .ioccr_mask = TSC_IOCCR_G4_IO3_MASK | TSC_IOCCR_G4_IO4_MASK |
                      TSC_IOCCR_G5_IO2_MASK | TSC_IOCCR_G5_IO3_MASK |
                      TSC_IOCCR_G5_IO4_MASK,
        .ioccr_value = TSC_IOCCR_G4_IO4_MASK | TSC_IOCCR_G5_IO3_MASK,
        .group_list = {{4, 1}, {5, 3}},
    },
    {
        .iogcsr_mask = TSC_IOGCSR_G4E_MASK | TSC_IOGCSR_G5E_MASK,
        .iogcsr_value = TSC_IOGCSR_G5E_MASK,
        .ioccr_mask = TSC_IOCCR_G4_IO3_MASK | TSC_IOCCR_G4_IO4_MASK |
                      TSC_IOCCR_G5_IO2_MASK | TSC_IOCCR_G5_IO3_MASK |
                      TSC_IOCCR_G5_IO4_MASK,
        .ioccr_value = TSC_IOCCR_G5_IO4_MASK,
        .group_list = {{5, 4}},
    },
};

#define TSC_STEP_NUM ARRAY_SIZE(TSC_STEP_LIST)

/** Values acquired from the touch sense controller */
volatile unsigned int TSC_VALUE_LIST[5];

/**
 * Start a touch sense controller acquisition step.
 *
 * @param step_idx  The index of the step to start.
 *                  Must be less than TSC_STEP_NUM.
 */
void
tsc_step_start(size_t step_idx)
{
    assert(step_idx < TSC_STEP_NUM);
    const struct tsc_step *step = &TSC_STEP_LIST[step_idx];
    TSC->iogcsr = (TSC->iogcsr & ~step->iogcsr_mask) | step->iogcsr_value;
    TSC->ioccr = (TSC->ioccr & ~step->ioccr_mask) | step->ioccr_value;
    TSC->cr |= TSC_CR_START_MASK;
}

/**
 * Output values acquired during a touch sense controller acquisition step
 * into TSC_VALUE_LIST.
 *
 * @param step_idx  The index of the step to output values of.
 *                  Must be less than TSC_STEP_NUM.
 */
void
tsc_step_output(size_t step_idx)
{
    assert(step_idx < TSC_STEP_NUM);
    const struct tsc_step *step = &TSC_STEP_LIST[step_idx];
    size_t i;
    for (i = 0; step->group_list[i][0] != 0; i++) {
        assert(step->group_list[i][0] < ARRAY_SIZE(TSC->iogxcr));
        assert(step->group_list[i][1] < ARRAY_SIZE(TSC_VALUE_LIST));
        TSC_VALUE_LIST[step->group_list[i][1]] =
            TSC->iogxcr[step->group_list[i][0] - 1];
    }
}

/** The index of the current touch sense controller acquisition step */
volatile size_t TSC_STEP_IDX;

/**
 * Handle touch sense controller interrupts.
 */
void tsc_irq_handler(void) __attribute__ ((isr));
void
tsc_irq_handler(void)
{
    if (TSC->isr & TSC_ISR_EOAF_MASK) {
        tsc_step_output(TSC_STEP_IDX);
        TSC_STEP_IDX++;
        if (TSC_STEP_IDX >= ARRAY_SIZE(TSC_STEP_LIST)) {
            TSC_STEP_IDX = 0;
        }
        tsc_step_start(TSC_STEP_IDX);
        TSC->icr = TSC_ICR_EOAIC_MASK;
    }
}

/**
 * Configure the touch sense controller.
 * Use I/O Groups 4 and 5.
 * Group 4 (IO 1,3,4): PA9 - sampling cap, PA11, PA12 - sensors.
 * Group 5 (IO 1,2,3,4): PB3 - sampling cap, PB4, PB6, PB7 - sensors.
 */
static void
tsc_init(void)
{
    size_t i;

    /* Configure group 4 */
    gpio_pin_conf_af(GPIO_A, 9, 3, GPIO_OTYPE_OPEN_DRAIN,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_A, 11, 3, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_A, 12, 3, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);

    /* Configure group 5 */
    gpio_pin_conf_af(GPIO_B, 3, 3, GPIO_OTYPE_OPEN_DRAIN,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_B, 4, 3, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_B, 6, 3, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);
    gpio_pin_conf_af(GPIO_B, 7, 3, GPIO_OTYPE_PUSH_PULL,
                     GPIO_PUPD_NONE, GPIO_OSPEED_HIGH);

    /* Enable clock to the TSC */
    RCC->ahbenr |= RCC_AHBENR_TSCEN_MASK;

    /* Configure and enable the touch sense controller */
    TSC->cr = (TSC->cr & ~(TSC_CR_PGPSC_MASK |
                           TSC_CR_CTPH_MASK |
                           TSC_CR_CTPL_MASK |
                           TSC_CR_MCV_MASK)) |
              /* Divide 32MHz AHB clock by 8 to get 4MHz PGCLK */
              (3 << TSC_CR_PGPSC_LSB) |
              /* Use four cycles of PGCLK to get 1us charge transfer high */
              /* TODO: Make sure this charges the sampling cap fully */
              (3 << TSC_CR_CTPH_LSB) |
              /* Use four cycles of PGCLK to get 1us charge transfer low */
              (3 << TSC_CR_CTPL_LSB) |
              /* Set max count value to 16383 */
              (TSC_CR_MCV_VAL_16383 << TSC_CR_MCV_LSB) |
              /* Enable the TSC */
              TSC_CR_TSCE_MASK;

    /* Disable hysteresis on all used I/Os */
    TSC->iohcr &= ~(TSC_IOHCR_G4_IO1_MASK |
                    TSC_IOHCR_G4_IO3_MASK |
                    TSC_IOHCR_G4_IO4_MASK |
                    TSC_IOHCR_G5_IO1_MASK |
                    TSC_IOHCR_G5_IO2_MASK |
                    TSC_IOHCR_G5_IO3_MASK |
                    TSC_IOHCR_G5_IO4_MASK);

    /* Enable sampling channels */
    TSC->ioscr |= TSC_IOSCR_G4_IO1_MASK | TSC_IOSCR_G5_IO1_MASK;

    /* Initialize retrieved counters */
    for (i = 0; i < ARRAY_SIZE(TSC_VALUE_LIST); i++)
        TSC_VALUE_LIST[i] = 0;

    /* Start with the first step */
    TSC_STEP_IDX = 0;

    /* Enable the end-of-acquisition interrupt */
    TSC->ier |= TSC_IER_EOAIE_MASK;

    /* Enable TSC interrupts */
    nvic_int_set_enable(NVIC_INT_TSC);

    /* Start the first acquisition step */
    tsc_step_start(TSC_STEP_IDX);
}

#if 0
/**
 * Initialize LEDs
 */
void
leds_init(void)
{
}
#endif

int
main(void)
{
    /* Basic init */
    init();
    /* Enable clock to I/O port A */
    RCC->iopenr |= RCC_IOPENR_IOPAEN_MASK | RCC_IOPENR_IOPBEN_MASK;

    usart_init();
    tsc_init();
    /* Clear the screen and hide the cursor */
    usart_printf("\e[2J\e[?25l");
    while (true) {
        usart_printf("\r%x %x %x %x %x                            ",
                     TSC_VALUE_LIST[0],
                     TSC_VALUE_LIST[1],
                     TSC_VALUE_LIST[2],
                     TSC_VALUE_LIST[3],
                     TSC_VALUE_LIST[4]);
    }
#if 0
    /* Enable clock to GPIO ports A and B */
    RCC->iopenr |= RCC_IOPENR_IOPAEN_MASK | RCC_IOPENR_IOPBEN_MASK;

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

    /* Enable TSC */
    /* Enable LED GPIO */
    /* Enable the serial port */

    /* Stat LED lighting cycle */
    /* Start TSC acquisition */
    /* Forever */
        /* Wait for a new TSC reading */
        /* Output the reading to the serial port */
#endif
}
