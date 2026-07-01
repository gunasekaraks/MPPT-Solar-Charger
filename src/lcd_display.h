#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>

void lcd_display_init();
void lcd_display_clear();
void lcd_display_show_text(const char *line1, const char *line2);

#endif