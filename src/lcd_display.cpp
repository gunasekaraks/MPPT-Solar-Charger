#include "lcd_display.h"

#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C lcd(U8G2_R0, U8X8_PIN_NONE);

static void drawTextScreen(const char *line1, const char *line2) {
  lcd.clearBuffer();
  lcd.setFont(u8g2_font_6x10_tf);
  lcd.setFontPosTop();
  lcd.drawStr(0, 0, line1 ? line1 : "");
  lcd.drawStr(0, 16, line2 ? line2 : "");
  lcd.sendBuffer();
}

void lcd_display_init() {
  lcd.begin();
  drawTextScreen("MPPT Solar Charger", "LCD display ready");
}

void lcd_display_clear() {
  lcd.clearBuffer();
  lcd.sendBuffer();
}

void lcd_display_show_text(const char *line1, const char *line2) {
  drawTextScreen(line1, line2);
}