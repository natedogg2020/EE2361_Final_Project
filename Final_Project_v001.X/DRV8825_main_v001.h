/* 
 * File:   DRV8825_main_v001.h
 * Author: Nate
 *
 * Created on November 12, 2020, 11:06 PM
 */

#ifndef DRV8825_MAIN_V001_H
/*
 * File:   DRV8825_main_v001.h
 * Author: Nate McKelvey and Hai Nguyen
 *
 * Created on November 12, 2020
 */

#define	DRV8825_MAIN_V001_H

#ifdef	__cplusplus
extern "C" {
#endif

//  I wanted to have these files made, but not implemented yet until we know
//  exactly how we want to have the functions declared from Final_Project_main_v001.c
    
    //Globals for IC2 motor control
    extern unsigned int set;
    extern unsigned int run;
    
    //DRV8825 Library Functions
    void DRV8825_Setup(void);
    void LCD_SpecialPrint(const char top[], const char bottom[]);
    void msecs(int);
    void delay_us(unsigned int);
    void setMode(unsigned char);
    void button_Setup();
    void full_Step(int dir, int steps, int delay);          // 1:1 Stepping
    void half_Step(int dir,int steps, int delay);
    void quarter_Step(int dir, int steps, int delay);       // 1:4 Stepping
    void eighth_Step(int dir, int steps, int delay);
    void sixteenth_Step(int dir, int steps, int delay);     // 1:16 Stepping
    void thirtieth_Step(int dir, int steps, int delay);     // 1:32 Stepping
    void fancy_Step(int dir, long long steps, unsigned char mode,
                    int accel, int decel, unsigned int  mult, 
                    int initial_speed, int max_speed,int end_speed);

#ifdef	__cplusplus
}
#endif

#endif	/* DRV8825_MAIN_V001_H */

