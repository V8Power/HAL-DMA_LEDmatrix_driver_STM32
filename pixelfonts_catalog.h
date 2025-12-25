#pragma once
#include <stdint.h>
#include "pixelfonts.h"
#include "pixelfonts_builtin.h"   // PFONT_... glyph arrays & size defines

typedef enum {
  PFONT_ID_5X7 = 0,
  PFONT_ID_12X15,
  PFONT_ID_24X32,
  PFONT_ID_COUNT
} PixelFontId;

typedef struct {
  const uint8_t* data;
  uint8_t width;
  uint8_t height;
  uint8_t advance;
  PixelFontFormat fmt;
  uint8_t firstChar;
  uint8_t lastChar;
} PixelFontInfo;

// NOTE: This lives in flash (const).
static const PixelFontInfo g_PixelFontCatalog[PFONT_ID_COUNT] = {
  // 5x7
  {
    .data = PFONT_5X7_GLYPHS,
    .width = 5,
    .height = 7,
    .advance = 6,
    .fmt = PIXELFONT_FMT_COL5x7,
    .firstChar = PFONT_ASCII_FIRST,
    .lastChar = PFONT_ASCII_LAST
  },

  // 12x15 ASCII 32..126
  {
    .data = PFONT_12X15_ASCII_32_126,
    .width = PFONT_12X15_W,
    .height = PFONT_12X15_H,
    .advance = (uint8_t)(PFONT_12X15_W + 1),
    .fmt = PIXELFONT_FMT_ROW1BPP,
    .firstChar = PFONT_ASCII_FIRST,
    .lastChar = PFONT_ASCII_LAST
  },

  // 24x32 ASCII 32..126
  {
    .data = PFONT_ARIAL_24X32_ASCII_32_126,
    .width = PFONT_ARIAL_24X32_W,
    .height = PFONT_ARIAL_24X32_H,
    .advance = (uint8_t)(PFONT_ARIAL_24X32_W + 1),
    .fmt = PIXELFONT_FMT_ROW1BPP,
    .firstChar = PFONT_ASCII_FIRST,
    .lastChar = PFONT_ASCII_LAST
  }
};
