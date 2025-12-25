#include "pixeldriver.h"
#include "pixelfonts.h"
#include "pixelfonts_builtin.h" // PFONT5X7_IndexForChar(), PFONT_5X7_GLYPHS
#include <string.h>

extern "C"
{
#include "stm32f4xx_hal.h"
}

// ======================================================
// BUFFERS
// ======================================================
static uint8_t framebuffer[PANEL_HEIGHT][PANEL_WIDTH / 8];

// 4 words per column (DATA clr, DATA set, CLK↑, CLK↓)
static uint32_t dma_linebuf[PANEL_WIDTH * 4];

// ======================================================
// HAL handles
// ======================================================
static TIM_HandleTypeDef htim_row; // TIM3 row tick
static TIM_HandleTypeDef htim_bit; // TIM8 shift pacing
static TIM_HandleTypeDef htim_pwm; // TIM1 PWM PE14

static volatile uint8_t current_phase = 0;
static volatile uint8_t dma_busy = 0;

static volatile uint32_t dma_start_ok = 0;
static volatile uint32_t dma_cplt_count = 0;
static volatile uint32_t dma_err_count = 0;

static uint32_t tim8_word_hz_actual = 0;

// ======================================================
// DWT delay for latch pulse
// ======================================================
static uint32_t cpu_hz = 0;

static inline void dwt_ensure_enabled(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void dwt_init_local(void)
{
  cpu_hz = HAL_RCC_GetHCLKFreq();
  dwt_ensure_enabled();
  DWT->CYCCNT = 0;
}

static inline void delay_cycles_fallback(uint32_t cycles)
{
  for (volatile uint32_t i = 0; i < cycles; i++)
    __NOP();
}

static inline void delay_cycles(uint32_t cycles)
{
  dwt_ensure_enabled();
  uint32_t start = DWT->CYCCNT;
  uint32_t guard = 0;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles)
  {
    if (++guard > 1000000UL)
    {
      delay_cycles_fallback(cycles / 4 + 100);
      return;
    }
  }
}

static inline void latch_pulse_us(uint32_t us)
{
  if (cpu_hz == 0)
    cpu_hz = HAL_RCC_GetHCLKFreq();

  GPIOB->BSRR = LATCH_PIN;

  uint32_t cycles = (uint32_t)((uint64_t)cpu_hz * us / 1000000ULL);
  if (cycles < 300)
    cycles = 300;

  delay_cycles(cycles);

  GPIOB->BSRR = (uint32_t)LATCH_PIN << 16;
}

// ======================================================
// Timer clock helper
// ======================================================
static uint32_t get_apb_timer_clock_hz(bool apb2)
{
  uint32_t pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  uint32_t cfgr = RCC->CFGR;
  uint32_t ppre = apb2 ? ((cfgr >> RCC_CFGR_PPRE2_Pos) & 0x7)
                       : ((cfgr >> RCC_CFGR_PPRE1_Pos) & 0x7);
  if (ppre >= 4)
    return pclk * 2;
  return pclk;
}

static inline uint16_t map_x(uint16_t x)
{
#if FB_X_FLIP
  return (uint16_t)((PANEL_WIDTH - 1) - x);
#else
  return x;
#endif
}

static inline uint16_t map_y(uint16_t y)
{
#if FB_Y_FLIP
  return ((PANEL_HEIGHT - 1) - y);
#else
  return y;
#endif
}

// ======================================================
// FRAMEBUFFER ACCESS
// ======================================================
static inline uint8_t fb_get_pixel(uint16_t y, uint16_t x)
{

  if (y >= PANEL_HEIGHT || x >= PANEL_WIDTH)
    return 0;
  const uint8_t byte = framebuffer[y][x >> 3];

#if FB_MSB_FIRST
  const uint8_t bit = 7 - (x & 0x07);
#else
  const uint8_t bit = (x & 0x07);
#endif

  return (byte >> bit) & 0x01;
}

static inline void fb_set_pixel(uint16_t y_in, uint16_t x_in, uint8_t value)
{

  uint16_t x = 0;
  uint16_t y = 0;

  x = map_x(x_in);
  y = map_y(y_in);

  /*#if FB_Y_FLIP
  y =  ((PANEL_HEIGHT - 1) - y_in);
#else
  y = y_in;
#endif
*/

  if (y >= PANEL_HEIGHT || x >= PANEL_WIDTH)
    return;

  uint8_t *p = &framebuffer[y][x >> 3];

#if FB_MSB_FIRST
  const uint8_t bit = 7 - (x & 0x07);
#else
  const uint8_t bit = (x & 0x07);
#endif

  const uint8_t mask = (uint8_t)(1U << bit);

  if (value)
    *p |= mask;
  else
    *p &= (uint8_t)~mask;
}

// ======================================================
// Panel helpers
// ======================================================
static inline void set_row_address(uint8_t phase)
{
  uint16_t set = 0;
  if (phase & 0x01)
    set |= ROW_A_PIN;
  if (phase & 0x02)
    set |= ROW_B_PIN;
  if (phase & 0x04)
    set |= ROW_C_PIN;

  GPIOB->BSRR = (uint32_t)ROW_MASK << 16;
  GPIOB->BSRR = set;
}

// ======================================================
// Build ONE DMA line buffer for the current phase
// 4 writes per column: DATA clr, DATA set, CLK↑, CLK↓
// ======================================================
static void build_dma_linebuf_for_phase(uint8_t phase)
{
  for (int x = 0; x < PANEL_WIDTH; x++)
  {
    uint8_t b0 = fb_get_pixel((uint16_t)(phase + 0), (uint16_t)x);
    uint8_t b1 = fb_get_pixel((uint16_t)(phase + 8), (uint16_t)x);
    uint8_t b2 = fb_get_pixel((uint16_t)(phase + 16), (uint16_t)x);
    uint8_t b3 = fb_get_pixel((uint16_t)(phase + 24), (uint16_t)x);

    uint16_t set_data = 0;
    if (b0)
      set_data |= D0_PIN;
    if (b1)
      set_data |= D1_PIN;

    // ==================================================
    // IMPORTANT WEIRDNESS (KEEP AS-IS)
    // ==================================================
    if (b2)
      set_data |= 0b01001000; // <-- keep

    if (b3)
      set_data |= D3_PIN;

    uint32_t w_data_clr = ((uint32_t)DATA_MASK << 16);
    uint32_t w_data_set = (uint32_t)set_data;
    uint32_t w_clk_hi = (uint32_t)CLK_MASK;
    uint32_t w_clk_lo = ((uint32_t)CLK_MASK << 16);

    dma_linebuf[4 * x + 0] = w_data_clr;
    dma_linebuf[4 * x + 1] = w_data_set;
    dma_linebuf[4 * x + 2] = w_clk_hi;
    dma_linebuf[4 * x + 3] = w_clk_lo;
  }
}

// ======================================================
// GPIO init
// ======================================================
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef gi = {0};

  // GPIOE: matrix pins
  gi.Pin = DATA_MASK | CLK_MASK;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &gi);

  // GPIOB: controls + DBG
  gi.Pin = ROW_MASK | LATCH_PIN | OE_PIN | ENA_PIN | ROW_EN_PIN | CLR_PIN | DBG_PIN;
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gi);

  // Permanent states
  GPIOB->BSRR = ENA_PIN;                // ENA high
  GPIOB->BSRR = CLR_PIN;                // CLRA high
  GPIOB->BSRR = ROW_EN_PIN;             // ROWenA high
  GPIOB->BSRR = (uint32_t)OE_PIN << 16; // ColENA low (enable)

  GPIOE->BSRR = (uint32_t)(DATA_MASK | CLK_MASK) << 16;
  DBG_GPIO_PORT->BSRR = (uint32_t)DBG_PIN << 16;

  latch_pulse_us(LATCH_PULSE_US);
}

// ======================================================
// TIM8 shift pacing init
// ======================================================
static void MX_TIM8_Init(void)
{
#if USE_DMA
  __HAL_RCC_TIM8_CLK_ENABLE();

  uint32_t tim8_clk = get_apb_timer_clock_hz(true);
  uint32_t arr = (tim8_clk / SHIFT_WORD_HZ_TARGET);
  if (arr < 2)
    arr = 2;
  arr -= 1;

  tim8_word_hz_actual = tim8_clk / (arr + 1);

  htim_bit.Instance = TIM8;
  htim_bit.Init.Prescaler = 0;
  htim_bit.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_bit.Init.Period = (uint16_t)arr;
  htim_bit.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_bit.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim_bit);

  __HAL_TIM_ENABLE_DMA(&htim_bit, TIM_DMA_UPDATE);
  HAL_TIM_Base_Stop(&htim_bit);
#endif
}

// ======================================================
// DMA stream config (direct register control)
// ======================================================
#if TIM8_DMA_STREAM_SEL == 1
#define DMAx DMA2
#define DMA_STREAM DMA2_Stream1
#define DMA_IRQn DMA2_Stream1_IRQn
#define DMA_TCIF_MASK DMA_LISR_TCIF1
#define DMA_TEIF_MASK DMA_LISR_TEIF1
#define DMA_CTCIF_MASK DMA_LIFCR_CTCIF1
#define DMA_CTEIF_MASK DMA_LIFCR_CTEIF1
#define DMA_CHTIF_MASK DMA_LIFCR_CHTIF1
#define DMA_CDMEIF_MASK DMA_LIFCR_CDMEIF1
#define DMA_CFEIF_MASK DMA_LIFCR_CFEIF1
#define DMA_CLEAR_ALL() (DMAx->LIFCR = (DMA_CTCIF_MASK | DMA_CTEIF_MASK | DMA_CHTIF_MASK | DMA_CDMEIF_MASK | DMA_CFEIF_MASK))
#elif TIM8_DMA_STREAM_SEL == 2
#define DMAx DMA2
#define DMA_STREAM DMA2_Stream2
#define DMA_IRQn DMA2_Stream2_IRQn
#define DMA_TCIF_MASK DMA_LISR_TCIF2
#define DMA_TEIF_MASK DMA_LISR_TEIF2
#define DMA_CTCIF_MASK DMA_LIFCR_CTCIF2
#define DMA_CTEIF_MASK DMA_LIFCR_CTEIF2
#define DMA_CHTIF_MASK DMA_LIFCR_CHTIF2
#define DMA_CDMEIF_MASK DMA_LIFCR_CDMEIF2
#define DMA_CFEIF_MASK DMA_LIFCR_CFEIF2
#define DMA_CLEAR_ALL() (DMAx->LIFCR = (DMA_CTCIF_MASK | DMA_CTEIF_MASK | DMA_CHTIF_MASK | DMA_CDMEIF_MASK | DMA_CFEIF_MASK))
#else
#error "Invalid TIM8_DMA_STREAM_SEL"
#endif

static void MX_DMA_Init(void)
{
#if USE_DMA
  __HAL_RCC_DMA2_CLK_ENABLE();

  DMA_STREAM->CR &= ~DMA_SxCR_EN;
  while (DMA_STREAM->CR & DMA_SxCR_EN)
  {
  }

  DMA_CLEAR_ALL();

  DMA_STREAM->CR =
      (7U << DMA_SxCR_CHSEL_Pos) |
      DMA_SxCR_DIR_0 |
      DMA_SxCR_MINC |
      DMA_SxCR_MSIZE_1 |
      DMA_SxCR_PSIZE_1 |
      DMA_SxCR_TCIE |
      DMA_SxCR_TEIE |
      DMA_SxCR_PL_1;

  DMA_STREAM->FCR = 0;

  HAL_NVIC_SetPriority(DMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA_IRQn);
#endif
}

// ======================================================
// TIM3 row tick init
// ======================================================
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  uint32_t tim3_clk = get_apb_timer_clock_hz(false);

  uint32_t psc = (tim3_clk / 10000UL);
  if (psc < 2)
    psc = 2;
  psc -= 1;
  uint32_t base = tim3_clk / (psc + 1);

  uint32_t arr = (base / ROW_TICK_HZ);
  if (arr < 2)
    arr = 2;
  arr -= 1;

  htim_row.Instance = TIM3;
  htim_row.Init.Prescaler = (uint16_t)psc;
  htim_row.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_row.Init.Period = (uint16_t)arr;
  htim_row.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_row.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim_row);

  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

// ======================================================
// TIM1 PWM PE14 init
// ======================================================
static void MX_TIM1_PWM_PE14_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef gi = {0};
  gi.Pin = VCTRL_PWM_PIN;
  gi.Mode = GPIO_MODE_AF_PP;
  gi.Pull = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gi.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &gi);

  htim_pwm.Instance = TIM1;
  htim_pwm.Init.Prescaler = VCTRL_PWM_PSC;
  htim_pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim_pwm.Init.Period = VCTRL_PWM_TOP;
  htim_pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim_pwm.Init.RepetitionCounter = 0;
  htim_pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_pwm);

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim_pwm, &oc, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim_pwm, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim_pwm);
}

static inline void vctrl_set_duty_u8(uint8_t duty)
{
  __HAL_TIM_SET_COMPARE(&htim_pwm, TIM_CHANNEL_4, duty);
}

// ======================================================
// IRQ handlers
// ======================================================
extern "C" void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim_row);
}

extern "C" void DMA2_Stream1_IRQHandler(void);
extern "C" void DMA2_Stream2_IRQHandler(void);

#if TIM8_DMA_STREAM_SEL == 1
extern "C" void DMA2_Stream1_IRQHandler(void)
{
  DBG_GPIO_PORT->ODR ^= DBG_PIN;

  if (DMA2->LISR & DMA_TCIF_MASK)
  {
    DMA_CLEAR_ALL();

    HAL_TIM_Base_Stop(&htim_bit);
    __HAL_TIM_DISABLE_DMA(&htim_bit, TIM_DMA_UPDATE);
    TIM8->SR = 0;

    dma_busy = 0;
    dma_cplt_count++;

    latch_pulse_us(LATCH_PULSE_US);
    set_row_address(current_phase);

    current_phase++;
    if (current_phase >= MUX_PHASES)
      current_phase = 0;
    return;
  }

  if (DMA2->LISR & DMA_TEIF_MASK)
  {
    DMA_CLEAR_ALL();
    dma_busy = 0;
    dma_err_count++;
  }
}
#elif TIM8_DMA_STREAM_SEL == 2
extern "C" void DMA2_Stream2_IRQHandler(void)
{
  DBG_GPIO_PORT->ODR ^= DBG_PIN;

  if (DMA2->LISR & DMA_TCIF_MASK)
  {
    DMA_CLEAR_ALL();

    HAL_TIM_Base_Stop(&htim_bit);
    __HAL_TIM_DISABLE_DMA(&htim_bit, TIM_DMA_UPDATE);
    TIM8->SR = 0;

    dma_busy = 0;
    dma_cplt_count++;

    latch_pulse_us(LATCH_PULSE_US);
    set_row_address(current_phase);

    current_phase++;
    if (current_phase >= MUX_PHASES)
      current_phase = 0;
    return;
  }

  if (DMA2->LISR & DMA_TEIF_MASK)
  {
    DMA_CLEAR_ALL();
    dma_busy = 0;
    dma_err_count++;
  }
}
#endif

// ======================================================
// TIM3 row tick callback: build linebuf + start DMA shift
// ======================================================
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
#if USE_DMA
    if (dma_busy)
      return;

    build_dma_linebuf_for_phase(current_phase);

    const uint32_t length = PANEL_WIDTH * 4;

    DMA_STREAM->CR &= ~DMA_SxCR_EN;
    while (DMA_STREAM->CR & DMA_SxCR_EN)
    {
    }

    DMA_CLEAR_ALL();

    DMA_STREAM->PAR = (uint32_t)&GPIOE->BSRR;
    DMA_STREAM->M0AR = (uint32_t)dma_linebuf;
    DMA_STREAM->NDTR = length;

    dma_busy = 1;
    DMA_STREAM->CR |= DMA_SxCR_EN;

    TIM8->SR = 0;
    __HAL_TIM_ENABLE_DMA(&htim_bit, TIM_DMA_UPDATE);

    __HAL_TIM_SET_COUNTER(&htim_bit, 0);
    HAL_TIM_Base_Start(&htim_bit);

    TIM8->EGR = TIM_EGR_UG;

    dma_start_ok++;
#endif
  }
}

// ======================================================
// ================= FONT SUPPORT (via pixelfonts.*) =====
// ======================================================

// The active font is managed by PixelDriver_SetFont().
// Use PixelFonts_Select5x7()/Select15()/Select32() after PixelDriver_Init().
static const PixelFont *g_font = NULL;

void PixelDriver_SetFont(const PixelDriver_Font *font)
{
  if (font && font->data && font->width && font->height)
    g_font = (const PixelFont *)font;
}

const PixelDriver_Font *PixelDriver_GetFont(void)
{
  return (const PixelDriver_Font *)g_font;
}

void PixelDriver_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
  for (uint16_t yy = 0; yy < h; yy++)
  {
    uint16_t ry = (uint16_t)(y + yy);
    if (ry >= PANEL_HEIGHT)
      break;
    for (uint16_t xx = 0; xx < w; xx++)
    {
      uint16_t rx = (uint16_t)(x + xx);
      if (rx >= PANEL_WIDTH)
        break;
      fb_set_pixel(ry, rx, color);
    }
  }
}

void PixelDriver_DrawChar(int16_t x, uint16_t y, char c, uint8_t color)
{
  if (!g_font)
    return;

  // Map chars outside the active font range to space.
  // (So you can use full ASCII fonts without hardcoding ranges here.)
  char cc = c;
  if ((uint8_t)cc < g_font->firstChar || (uint8_t)cc > g_font->lastChar)
    cc = ' ';

  const uint8_t w = g_font->width;
  const uint8_t h = g_font->height;

  if (g_font->fmt == PIXELFONT_FMT_COL5x7)
  {
    // Column-major 5x7 source, sparse mapping handled by helper
    const int idx = PFONT5X7_IndexForChar(cc);
    const uint8_t *glyph = &PFONT_5X7_GLYPHS[idx * g_font->bytesPerGlyph];

    for (uint8_t col = 0; col < w; col++)
    {
      uint8_t bits = glyph[col];
      for (uint8_t row = 0; row < h; row++)
      {
        uint8_t on = (bits >> row) & 0x01;
        if (x >= 0)
        {
          if (on)
            fb_set_pixel((uint16_t)(y + row), (uint16_t)(x + col), color);
          else
            fb_set_pixel((uint16_t)(y + row), (uint16_t)(x + col), 0);
        }
      }
    }
    return;
  }

  // Row-major 1bpp (LSB-first in each byte)
  // bytesPerRow = ceil(width/8)
  const uint8_t bytesPerRow = (uint8_t)((w + 7U) / 8U);
  const uint32_t ci = (uint32_t)((uint8_t)cc - (uint8_t)g_font->firstChar);
  const uint8_t *glyph = &g_font->data[ci * g_font->bytesPerGlyph];

  for (uint8_t row = 0; row < h; row++)
  {
    const uint8_t *rowPtr = &glyph[(uint32_t)row * bytesPerRow];
    for (uint8_t col = 0; col < w; col++)
    {
      uint8_t v = (rowPtr[col >> 3] >> (col & 7U)) & 1U;
      if (v)
        fb_set_pixel((uint16_t)(y + row), (uint16_t)(x + col), color);
      else
        fb_set_pixel((uint16_t)(y + row), (uint16_t)(x + col), 0);
    }
  }
}

void PixelDriver_DrawText(int16_t x, uint16_t y, const char *s, uint8_t color)
{
  if (!s || !g_font)
    return;
  int16_t cx = x;
  const uint8_t adv = g_font->advance ? g_font->advance : (uint8_t)(g_font->width + 1);
  while (*s)
  {
    PixelDriver_DrawChar(cx, y, *s, color);
    cx = (int16_t)(cx + adv);
    s++;
  }
}

// ======================================================
// ================= BLIT FUNCTIONS =====================
// ======================================================

static inline uint8_t bitstream_get(const uint32_t *dataBits, uint32_t bitIndex)
{
  // bit0 of word0 is first pixel
  uint32_t word = dataBits[bitIndex >> 5];
  uint32_t bit = bitIndex & 31U;
  return (word >> bit) & 1U;
}

void PixelDriver_BlitRowBits(uint16_t startx, uint16_t stopx, uint16_t rowY,
                             const uint32_t *dataBits)
{
  if (!dataBits)
    return;
  if (rowY >= PANEL_HEIGHT)
    return;
  if (startx >= PANEL_WIDTH)
    return;
  if (stopx >= PANEL_WIDTH)
    stopx = PANEL_WIDTH - 1;
  if (stopx < startx)
    return;

  uint32_t bi = 0; // bit index in the stream

  for (uint16_t x = startx; x <= stopx; x++, bi++)
  {
    uint8_t v = bitstream_get(dataBits, bi);
    fb_set_pixel(rowY, x, v);
  }
}

void PixelDriver_BlitRowsBits(uint16_t startx, uint16_t stopx, uint16_t startY,
                              uint16_t rowCount, const uint32_t *dataBits)
{
  if (!dataBits)
    return;
  if (startY >= PANEL_HEIGHT)
    return;
  if (rowCount == 0)
    return;
  if (startx >= PANEL_WIDTH)
    return;
  if (stopx >= PANEL_WIDTH)
    stopx = PANEL_WIDTH - 1;
  if (stopx < startx)
    return;

  const uint32_t rowWidthBits = (uint32_t)(stopx - startx + 1);

  for (uint16_t r = 0; r < rowCount; r++)
  {
    uint16_t y = (uint16_t)(startY + r);
    if (y >= PANEL_HEIGHT)
      break;

    const uint32_t *rowPtr = dataBits + ((uint32_t)r * ((rowWidthBits + 31U) >> 5));

    uint32_t bi = 0;
    for (uint16_t x = startx; x <= stopx; x++, bi++)
    {
      uint8_t v = bitstream_get(rowPtr, bi);
      fb_set_pixel(y, x, v);
    }
  }
}

// ======================================================
// Public API
// ======================================================
void PixelDriver_Init(void)
{
  dwt_init_local();
  MX_GPIO_Init();
#if USE_DMA
  MX_TIM8_Init();
  MX_DMA_Init();
#endif
  MX_TIM3_Init();
  MX_TIM1_PWM_PE14_Init();

  // Prepare built-in fonts (safe if called multiple times)
  //PixelFonts_Init();

  memset(framebuffer, 0, sizeof(framebuffer));

  HAL_TIM_Base_Start_IT(&htim_row);
  vctrl_set_duty_u8(80);

  // Default to 15px font unless you select another in your sketch.
  PixelFonts_Select15();
}

uint8_t PixelDriver_GetPixel(uint16_t y, uint16_t x) { return fb_get_pixel(y, x); }
void PixelDriver_SetPixel(uint16_t y, uint16_t x, uint8_t value)
{
  fb_set_pixel(y, x, value);
}

void PixelDriver_Clear(void) { memset(framebuffer, 0, sizeof(framebuffer)); }
void PixelDriver_Fill(uint8_t byteValue) { memset(framebuffer, byteValue, sizeof(framebuffer)); }

void PixelDriver_SetVctrlDuty(uint8_t duty) { vctrl_set_duty_u8(duty); }

uint32_t PixelDriver_GetDmaStartOk(void) { return dma_start_ok; }
uint32_t PixelDriver_GetDmaCplt(void) { return dma_cplt_count; }
uint32_t PixelDriver_GetDmaBusy(void) { return dma_busy; }
uint32_t PixelDriver_GetDmaNdtr(void) { return DMA_STREAM->NDTR; }
uint32_t PixelDriver_GetDmaErrCnt(void) { return dma_err_count; }
uint32_t PixelDriver_GetTim8WordHzActual(void) { return tim8_word_hz_actual; }
