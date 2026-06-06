/* main.c - STM32F407 bare-metal motor speed controller
 *
 * Architecture:
 *   SysTick ISR @ 1 kHz  ->  PID tick + plant update + UART CSV output
 *   ADC1 channel 0 (PA0) ->  potentiometer setpoint  (0..300 rad/s)
 *   TIM4 CH1  (PD12)     ->  PWM output to motor driver (10 kHz)
 *   USART2 (PA2/PA3)     ->  115200-8N1, "t,sp,omega\n" CSV stream
 *   Ethernet (lwIP)      ->  Modbus TCP holding register 40001 = omega*10
 *
 * Build:  see Makefile
 * Sim:    renode motor_control.resc
 * Debug:  arm-none-eabi-gdb firmware.elf  -> target remote :3333
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pid.h"
#include "motor_model.h"
#include "stm32f4xx.h"   /* CMSIS peripheral definitions */

/* ---- compile-time constants ---- */
#define SYSCLK_HZ     168000000UL
#define CTRL_HZ       1000UL          /* 1 kHz control loop    */
#define PWM_HZ        10000UL         /* 10 kHz PWM carrier    */
#define UART_BAUD     115200UL
#define ADC_MAX       4095.0f
#define OMEGA_MAX     300.0f          /* rad/s full-scale      */
#define V_SUPPLY      24.0f

/* ---- PID gains (same as host-sim and Wokwi sketch) ---- */
#define KP  0.08f
#define KI  6.0f
#define KD  0.0008f

/* ---- globals shared between ISR and main ---- */
static volatile float g_setpoint = 0.0f;
static volatile float g_omega    = 0.0f;   /* current motor speed [rad/s] */
static volatile uint32_t g_tick  = 0;

static PID    g_pid;
static Motor  g_motor;    /* software plant (see motor_model.h) */

/* ======================================================
 * Peripheral init helpers
 * ====================================================== */

static void clock_init(void) {
    /* Enable HSE, wait ready, configure PLL -> 168 MHz */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN   | (5u << FLASH_ACR_LATENCY_Pos);
    RCC->CFGR  = (4u << RCC_CFGR_PPRE2_Pos) |   /* APB2 /2 */
                 (5u << RCC_CFGR_PPRE1_Pos);      /* APB1 /4 */
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE |
                   (4u  << RCC_PLLCFGR_PLLM_Pos)  |
                   (168u << RCC_PLLCFGR_PLLN_Pos)  |
                   (0u  << RCC_PLLCFGR_PLLP_Pos);   /* /2 */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

static void gpio_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    /* PA0: analog (ADC)  PA2/PA3: AF7 (USART2 TX/RX) */
    GPIOA->MODER  |= (3u<<0) | (2u<<4) | (2u<<6);
    GPIOA->AFR[0] |= (7u<<8) | (7u<<12);
    /* PD12: AF2 (TIM4 CH1 PWM) */
    GPIOD->MODER  |= (2u<<24);
    GPIOD->AFR[1] |= (2u<<16);
}

static void usart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR   = (SYSCLK_HZ / 4) / UART_BAUD;  /* APB1 = SYSCLK/4 */
    USART2->CR1   = USART_CR1_TE | USART_CR1_UE;
}

static void usart2_puts(const char *s) {
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)*s++;
    }
}

static void adc1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->SQR3    = 0;          /* channel 0 = PA0 */
    ADC1->CR2     = ADC_CR2_ADON;
}

static uint16_t adc1_read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

static void tim4_pwm_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    uint32_t period = (SYSCLK_HZ / 2) / PWM_HZ - 1;  /* APB1 timer x2 */
    TIM4->PSC  = 0;
    TIM4->ARR  = period;
    TIM4->CCR1 = 0;
    TIM4->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1  |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void pwm_set(float duty) {
    /* duty 0.0..1.0 */
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    TIM4->CCR1 = (uint32_t)(duty * TIM4->ARR);
}

static void systick_init(void) {
    SysTick_Config(SYSCLK_HZ / CTRL_HZ);
}

/* ======================================================
 * SysTick ISR - 1 kHz control tick
 * ====================================================== */
void SysTick_Handler(void) {
    /* 1. Read setpoint from ADC */
    float sp = (adc1_read() / ADC_MAX) * OMEGA_MAX;
    g_setpoint = sp;

    /* 2. PID update */
    float v_cmd = pid_update(&g_pid, sp, g_omega);

    /* 3. Advance software motor model (100 x 10 us RK4 steps) */
    motor_step(&g_motor, v_cmd, 100);
    g_omega = g_motor.omega;

    /* 4. Drive PWM  (v_cmd normalised to duty cycle) */
    pwm_set((v_cmd + V_SUPPLY) / (2.0f * V_SUPPLY));

    /* 5. UART CSV every 10 ms (every 10th tick) to keep throughput sane */
    if (++g_tick % 10 == 0) {
        char buf[48];
        int  n = snprintf(buf, sizeof(buf), "%.3f,%.1f,%.4f\n",
                          g_tick * 0.001f, sp, g_omega);
        (void)n;
        usart2_puts(buf);
    }

    /* 6. Update Modbus holding register 40001 (omega * 10, integer) */
    modbus_set_omega((uint16_t)(g_omega * 10.0f));
}

/* ======================================================
 * main
 * ====================================================== */
int main(void) {
    clock_init();
    gpio_init();
    usart2_init();
    adc1_init();
    tim4_pwm_init();

    pid_init  (&g_pid,   KP, KI, KD, V_SUPPLY, 1.0f / CTRL_HZ);
    motor_init(&g_motor);
    modbus_tcp_init();   /* start lwIP + Modbus TCP listener on port 502 */

    systick_init();      /* starts the 1 kHz ISR */

    usart2_puts("t,setpoint,omega\n");  /* CSV header */

    for (;;) {
        /* Idle loop: lwIP polling + Ethernet DMA handling */
        ethernetif_poll();
    }
}
