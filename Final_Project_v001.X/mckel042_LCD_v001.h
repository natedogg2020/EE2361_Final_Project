/* 
 * File:   mckel042_LCD_v001.h
 * Author: Nate
 *
 * Created on November 23, 2020, 8:25 PM
 */

#ifndef MCKEL042_LCD_V001_H
#define	MCKEL042_LCD_V001_H

#ifdef	__cplusplus
extern "C" {
#endif


    void msecs(int n);
    void lcd_cmd(unsigned char command);
    void lcd_init(void);
    void lcd_setCursor(char x, char y);
    void lcd_printChar(char myChar);
    void lcd_printStr(const char s[]);
    void move();



#ifdef	__cplusplus
}
#endif

#endif	/* MCKEL042_LCD_V001_H */

