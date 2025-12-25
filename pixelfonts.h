#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Font formats supported by the driver
typedef enum
{
  PIXELFONT_FMT_COL5x7 = 0,   // column-major bytes (classic 5x7)
  PIXELFONT_FMT_ROW1BPP = 1   // row-major 1bpp bitmap (LSB-first per byte)
} PixelFontFormat;

typedef struct
{
  uint8_t width;
  uint8_t height;
  uint8_t advance;      // x advance; if 0 driver falls back to width+1
  PixelFontFormat fmt;

  uint8_t firstChar;    // inclusive, e.g. 32 for ' '
  uint8_t lastChar;     // inclusive, e.g. 126 for '~'

  const uint8_t *data;  // glyph data
  uint16_t bytesPerGlyph;
} PixelFont;

// -----------------------------------------------------------------------------
// Convenience initializer
// You said you want to "point at font data and tell width/height".
// For monospace fonts, this fills the PixelFont descriptor.
void PixelFonts_MakeMonospaceFont(PixelFont *out,
                                  const uint8_t *glyphData,
                                  uint8_t glyphW,
                                  uint8_t glyphH,
                                  uint8_t firstChar,
                                  uint8_t lastChar,
                                  PixelFontFormat fmt,
                                  uint8_t advance);

void PixelFonts_SelectCustom(const uint8_t* data,
                             uint8_t glyphW, uint8_t glyphH,
                             uint8_t firstChar, uint8_t lastChar,
                             PixelFontFormat fmt,
                             uint8_t advance);

// Built-in font selectors (call after PixelDriver_Init)
void PixelFonts_Select5x7(void);
void PixelFonts_Select15(void);
void PixelFonts_Select32(void);

// Optional: call once at startup (safe to call multiple times)


// -------- SD-card / external font future hook --------
typedef bool (*PixelFontReadFn)(void *ctx, uint32_t offset, uint8_t *dst, uint32_t len);

bool PixelFonts_LoadFromReader(void *ctx, PixelFontReadFn reader,
                              uint32_t baseOffset,
                              PixelFont *outFont,
                              uint8_t *glyphStorage,
                              uint32_t glyphStorageBytes);

#ifdef __cplusplus
}
#endif
