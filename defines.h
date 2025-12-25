#pragma once

// We use STM32 HAL pin macros (GPIO_PIN_x) in this file.
// Pull in the HAL definitions safely for both C and C++.
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
}
#endif

// ======================================================
// CONFIG
// ======================================================

// DMA + TIM8 paced shifter
#ifndef USE_DMA
#define USE_DMA 1
#endif

// Total logical framebuffer geometry (full chain)
#ifndef PANEL_WIDTH
#define PANEL_WIDTH   256
#endif
#ifndef PANEL_HEIGHT
#define PANEL_HEIGHT  32
#endif

// 1:8 multiplex
#ifndef MUX_PHASES
#define MUX_PHASES    8
#endif

// Shift pacing (TIM8 update rate). One DMA word per TIM8 update.
// With 4 words per pixel-column (data clr, data set, clk hi, clk lo),
// the resulting SHIFT clock is tim8_word_hz_actual / 4.
#ifndef SHIFT_WORD_HZ_TARGET
#define SHIFT_WORD_HZ_TARGET 3000000UL
#endif

// Row tick (TIM3). Must be slower than the time to shift one row.
#ifndef ROW_TICK_HZ
#define ROW_TICK_HZ 1000
#endif

// Latch width (using DWT cycle counter)
#ifndef LATCH_PULSE_US
#define LATCH_PULSE_US 1
#endif

// Framebuffer packing bit order inside a byte.
// FB_MSB_FIRST=1: bit7 corresponds to x%8==0
// FB_MSB_FIRST=0: bit0 corresponds to x%8==0
#ifndef FB_MSB_FIRST
#define FB_MSB_FIRST 1
#endif

// Optional logical flips (applied in fb_set_pixel via map_x/map_y)

#define FB_X_FLIP 1


#define FB_Y_FLIP 0


// PWM for PE14: TIM1_CH4, 8-bit @ ~10 kHz
#ifndef VCTRL_PWM_TOP
#define VCTRL_PWM_TOP 255
#endif
#ifndef VCTRL_PWM_PSC
// Prescaler value used directly in TIM1->PSC.
#define VCTRL_PWM_PSC 64
#endif

// DMA stream selection for TIM8_UP
// 1 = DMA2_Stream1, 2 = DMA2_Stream2
#ifndef TIM8_DMA_STREAM_SEL
#define TIM8_DMA_STREAM_SEL 1
#endif

// ======================================================
// PIN MAP
// ======================================================

// GPIOE: data + shift clocks (4 banks)
#define D0_PIN   GPIO_PIN_4
#define D1_PIN   GPIO_PIN_5
#define D2_PIN   GPIO_PIN_6
#define D3_PIN   GPIO_PIN_7

#define CLK0_PIN GPIO_PIN_0
#define CLK1_PIN GPIO_PIN_1
#define CLK2_PIN GPIO_PIN_2
#define CLK3_PIN GPIO_PIN_3

#define DATA_MASK (D0_PIN | D1_PIN | D2_PIN | D3_PIN)
#define CLK_MASK  (CLK0_PIN | CLK1_PIN | CLK2_PIN | CLK3_PIN)

// GPIOB: row select + control
#define ROW_A_PIN   GPIO_PIN_5
#define ROW_B_PIN   GPIO_PIN_6
#define ROW_C_PIN   GPIO_PIN_7
#define ROW_MASK    (ROW_A_PIN | ROW_B_PIN | ROW_C_PIN)

#define LATCH_PIN   GPIO_PIN_11  // ColLDA
#define OE_PIN      GPIO_PIN_12  // ColENA (active LOW) keep LOW to enable
#define ENA_PIN     GPIO_PIN_13  // ENA keep HIGH
#define ROW_EN_PIN  GPIO_PIN_10  // ROWenA keep HIGH
#define CLR_PIN     GPIO_PIN_14  // CLRA keep HIGH

// PWM pin
#define VCTRL_PWM_PIN GPIO_PIN_14 // PE14 = TIM1_CH4 (AF1)

// Debug pin to see DMA IRQ activity
#define DBG_GPIO_PORT GPIOB
#define DBG_PIN       GPIO_PIN_15
