#include "pixelfonts.h"
#include "pixelfonts_catalog.h"
#include "pixeldriver.h"
#include <string.h>

static PixelFont g_activeFont;   // ONE RAM instance only

static uint16_t bytes_per_glyph(uint8_t w, uint8_t h, PixelFontFormat fmt)
{
  if (fmt == PIXELFONT_FMT_COL5x7) {
    return w; // 5 bytes for the 5x7 glyphs in your format
  }
  // row-major 1bpp
  uint16_t bytesPerRow = (uint16_t)((w + 7U) / 8U);
  return (uint16_t)(bytesPerRow * (uint16_t)h);
}

void PixelFonts_ClearActive(void)
{
  memset(&g_activeFont, 0, sizeof(g_activeFont));
  // You can also set the driver font to NULL if you want, but your
  // PixelDriver_SetFont currently ignores NULL; that’s fine.
}

static void apply_info_to_active(const PixelFontInfo* info)
{
  // Clear first as requested
  PixelFonts_ClearActive();

  // Copy metadata
  g_activeFont.width = info->width;
  g_activeFont.height = info->height;
  g_activeFont.advance = info->advance;
  g_activeFont.fmt = info->fmt;
  g_activeFont.firstChar = info->firstChar;
  g_activeFont.lastChar = info->lastChar;

  // Only “fill data” from the selected font
  g_activeFont.data = info->data;

  // Derived
  g_activeFont.bytesPerGlyph = bytes_per_glyph(info->width, info->height, info->fmt);
}

void PixelFonts_SelectCustom(const uint8_t* data,
                             uint8_t glyphW, uint8_t glyphH,
                             uint8_t firstChar, uint8_t lastChar,
                             PixelFontFormat fmt,
                             uint8_t advance)
{
  PixelFonts_ClearActive();

  g_activeFont.width = glyphW;
  g_activeFont.height = glyphH;
  g_activeFont.advance = advance;
  g_activeFont.fmt = fmt;
  g_activeFont.firstChar = firstChar;
  g_activeFont.lastChar = lastChar;
  g_activeFont.data = data;
  g_activeFont.bytesPerGlyph = bytes_per_glyph(glyphW, glyphH, fmt);

  PixelDriver_SetFont((const PixelDriver_Font*)&g_activeFont);
}


void PixelFonts_SelectById(PixelFontId id)
{
  if ((int)id < 0 || id >= PFONT_ID_COUNT) return;
  apply_info_to_active(&g_PixelFontCatalog[id]);
  PixelDriver_SetFont((const PixelDriver_Font*)&g_activeFont);
}

// Keep your old API:
void PixelFonts_Select5x7(void) { PixelFonts_SelectById(PFONT_ID_5X7); }
void PixelFonts_Select15(void)  { PixelFonts_SelectById(PFONT_ID_12X15); }
void PixelFonts_Select32(void)  { PixelFonts_SelectById(PFONT_ID_24X32); }
