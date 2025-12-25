#pragma once

#include <stdint.h>
#include <stddef.h>
#include "defines.h"
#include "pixelfonts.h"  // PixelFont + PixelFontFormat

#ifdef __cplusplus
extern "C" {
#endif

// =========================
// Init / basic API
// =========================
void PixelDriver_Init(void);

// Framebuffer access (single pixel)
uint8_t PixelDriver_GetPixel(uint16_t y, uint16_t x);
void    PixelDriver_SetPixel(uint16_t y, uint16_t x, uint8_t value);

void    PixelDriver_Clear(void);
void    PixelDriver_Fill(uint8_t byteValue);

// PWM on PE14 (0..255)
void PixelDriver_SetVctrlDuty(uint8_t duty);

// Debug counters
uint32_t PixelDriver_GetDmaStartOk(void);
uint32_t PixelDriver_GetDmaCplt(void);
uint32_t PixelDriver_GetDmaBusy(void);
uint32_t PixelDriver_GetDmaNdtr(void);
uint32_t PixelDriver_GetDmaErrCnt(void);
uint32_t PixelDriver_GetTim8WordHzActual(void);

// =========================
// Blit helpers (1bpp)
// =========================
//
// Bitstream format:
// - dataBits is a packed 1bpp stream in uint32_t words
// - Bit 0 of dataBits[0] corresponds to x = startx
// - Next pixel is bit 1, etc.
//
// stopx is inclusive.
//
void PixelDriver_BlitRowBits(uint16_t startx, uint16_t stopx, uint16_t rowY,
                             const uint32_t *dataBits);

void PixelDriver_BlitRowsBits(uint16_t startx, uint16_t stopx, uint16_t startY,
                              uint16_t rowCount, const uint32_t *dataBits);

// =========================
// Font support
// =========================
// We use the PixelFont type from pixelfonts.h to keep font handling in one place.
// This preserves the PixelDriver_* API while allowing PixelFonts_Select*() helpers.
typedef PixelFont PixelDriver_Font;

void PixelDriver_SetFont(const PixelDriver_Font *font);
const PixelDriver_Font* PixelDriver_GetFont(void);

// Draw a character at (x,y) top-left
void PixelDriver_DrawChar(int16_t x, uint16_t y, char c, uint8_t color);

// Draw a NUL-terminated string (monospace advance = font->width + 1)
void PixelDriver_DrawText(int16_t x, uint16_t y, const char *s, uint8_t color);

// Optional: clear a char cell region (useful for overwriting)
void PixelDriver_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);

#ifdef __cplusplus
}
#endif
