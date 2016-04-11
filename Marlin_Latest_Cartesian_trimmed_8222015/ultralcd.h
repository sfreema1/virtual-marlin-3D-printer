#ifndef ULTRALCD_H
#define ULTRALCD_H

#include "Marlin.h"

FORCE_INLINE void lcd_update() {}
FORCE_INLINE void lcd_init() {}
FORCE_INLINE void lcd_setstatus(const char* message) {}
FORCE_INLINE void lcd_buttons_update() {}
FORCE_INLINE void lcd_reset_alert_level() {}
FORCE_INLINE void lcd_buzz(long duration, uint16_t freq) {}

#define LCD_MESSAGEPGM(x)
#define LCD_ALERTMESSAGEPGM(x)

char *itostr2(const uint8_t &x);
char *itostr31(const int &xx);
char *itostr3(const int &xx);
char *itostr3left(const int &xx);
char *itostr4(const int &xx);

char *ftostr3(const float &x);
char *ftostr31ns(const float &x); // float to string without sign character
char *ftostr31(const float &x);
char *ftostr32(const float &x);
char *ftostr5(const float &x);
char *ftostr51(const float &x);
char *ftostr52(const float &x);

#endif //ULTRALCD
