/* 
 * File:   DRV8825_main_v001.h
 * Author: Nate
 *
 * Created on November 12, 2020, 11:06 PM
 */

#ifndef DRV8825_MAIN_V001_H
#define	DRV8825_MAIN_V001_H

#ifdef	__cplusplus
extern "C" {
#endif

//  I wanted to have these files made, but not implemented yet until we know
//  exactly how we want to have the functions declared from Final_Project_main_v001.c
    void msecs(int);
    void delay_us(unsigned int);
    void setMode(unsigned char);
    void full_Step(int dir, int steps, int delay);          // 1:1 Stepping
//    void half_Step();
    void quarter_Step(int dir, int steps, int delay);       // 1:4 Stepping
//    void eighth_Step();
    void sixteenth_Step(int dir, int steps, int delay);     // 1:16 Stepping
    void thirtieth_Step(int dir, int steps, int delay);      // 1:32 Stepping

#ifdef	__cplusplus
}
#endif

#endif	/* DRV8825_MAIN_V001_H */

