#include "Arduino.h"
#include "pixeldriver.h"
#include "defines.h"
#include "pixelfonts_builtin.h"

void setup()
{
  Serial.begin(115200);
  delay(200);

  Serial.println("PixelDriver init");
  PixelDriver_Init();
  // PixelFonts_Select5x7();
  PixelFonts_Select32();

  Serial.print("TIM8 word_hz actual = ");
  Serial.println(PixelDriver_GetTim8WordHzActual());

#if FB_MSB_FIRST
  Serial.println("FB bit order: MSB-first");
#else
  Serial.println("FB bit order: LSB-first");
#endif
}

void loop()
{
  PixelDriver_SetVctrlDuty(90);

  static uint32_t t1 = 0;
  if (millis() - t1 > 500)
  {
    t1 = millis();

    Serial.print("DMA ok=");
    Serial.print(PixelDriver_GetDmaStartOk());
    Serial.print(" cplt=");
    Serial.print(PixelDriver_GetDmaCplt());
    Serial.print(" busy=");
    Serial.print(PixelDriver_GetDmaBusy());
    Serial.print(" NDTR=");
    Serial.print(PixelDriver_GetDmaNdtr());
    Serial.print(" errCnt=");
    Serial.println(PixelDriver_GetDmaErrCnt());

    // Your test sweep (kept)
    /*
    for (int y = 0; y < 32; y++)
    {
      for (int x = 0; x <= 64; x++)
      {
        PixelDriver_SetPixel(y,  x, 1);
        delay(1);
      }
    }

    delay(500);
    PixelDriver_Clear();
    delay(500);
*/

    // should be top-left
    PixelDriver_SetPixel(0, 0, 1);
    delay(100);
    PixelDriver_SetPixel(0, 63, 1); // bottom-left
    delay(100);
    PixelDriver_SetPixel(31, 0, 1); // bottom-left
    delay(100);
    PixelDriver_SetPixel(31, 63, 1); // bottom-right
    delay(100);
    PixelDriver_DrawText(0, 0, "MERRY CHRISTMAS", 1);
    // PixelDriver_Fill(255);
    delay(5000);
    PixelDriver_Clear();
    while (1)
    {
      for (int x = 63; x > -406; x--)
      {
        PixelDriver_Clear();
        PixelDriver_DrawText(x, 0, "Merry Christmas", 1);
        delay(30);
      }
    }
  }
}
