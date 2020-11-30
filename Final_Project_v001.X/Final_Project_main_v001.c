/*
 * File:   Final_Project_main_v001.c
 * Author: Nate McKelvey and Hai Nguyen
 *
 * Created on November 12, 2020, 11:07 PM
 */

#include "xc.h"
#include "DRV8825_main_v001.h"
#include "mckel042_LCD_v001.h"


// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))

#pragma config POSCMOD = NONE

void LCD_SpecialPrint(const char top[], const char bottom[]);

void setup(void){
    CLKDIVbits.RCDIV = 0;  //Set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M
    AD1PCFG = 0x9fff;            //sets all pins to digital I/O
    
    DRV8825_Setup();
    LCD_Setup();
}

void LCD_SpecialPrint(const char top[], const char bottom[]){
    lcd_setCursor(0, 0);
    lcd_printStr(top);
    lcd_setCursor(0, 1);
    lcd_printStr(bottom);
}

int main(void) {
    setup();
    int dir = 0;
    msecs(50);
    
    
    while(1){
        run = set;
        dir = !dir;  
        if (run == 1){
            msecs(500);           
            LCD_SpecialPrint("FullStep", "RUNNING ");
            full_Step(dir, 200, 5000);
            LCD_SpecialPrint("FullStep", "FINISHED");                
        }            
        if (run == 2){
            msecs(500);           
            LCD_SpecialPrint("1:2_Step", "RUNNING ");
            half_Step(dir, 2 *200, 3600);
            LCD_SpecialPrint("1:2_Step", "FINISHED");                
        }
        if (run == 3){
            msecs(500);           
            LCD_SpecialPrint("1:4_Step", "RUNNING ");
            quarter_Step(dir, 4 *200, 2500);
            LCD_SpecialPrint("1:4_Step", "FINISHED");                
        }
        if (run ==4){
            msecs(500);           
            LCD_SpecialPrint("1:8_Step", "RUNNING ");
            eighth_Step(dir, 8*200, 1600);
            LCD_SpecialPrint("1:8_Step", "FINISHED");                
        }
        if (run == 5){
            msecs(500);           
            LCD_SpecialPrint("16thStep", "RUNNING ");
            sixteenth_Step(dir, 16 *200, 800);
            LCD_SpecialPrint("16thStep", "FINISHED");                
        }            
        if (run == 6){
            LCD_SpecialPrint("32ndStep", "RUNNING ");
            thirtieth_Step(dir, 32 *200, 300);
            LCD_SpecialPrint("32ndStep","FINISHED");
            msecs(500);
        }               
        if (run == 7){
            LCD_SpecialPrint("FncyStep", "RUNNING ");
            fancy_Step(dir, (long long)5*32*200, 5, 1, 2, 2*32, 250 ,17, 300);
            LCD_SpecialPrint("FncyStep","FINISHED");
            msecs(500);
        }                 
        if (run == 8){
            LCD_SpecialPrint("FullStep", "RUNNING ");
            full_Step(dir, 200, 5000);
            LCD_SpecialPrint("FullStep", "FINISHED");
            msecs(500);

            LCD_SpecialPrint("HalfStep", "RUNNING ");
            half_Step(dir, 2 *200, 3600);
            LCD_SpecialPrint("HalfStep", "FINISHED");
            msecs(500);

            LCD_SpecialPrint("4th Step", "RUNNING ");
            quarter_Step(dir, 4 *200, 2500);
            LCD_SpecialPrint("4th Step","FINISHED");
            msecs(500);

            LCD_SpecialPrint("8th Step", "RUNNING ");
            eighth_Step(dir, 8*200, 1600);
            LCD_SpecialPrint("8th Step","FINISHED");
            msecs(500);

            LCD_SpecialPrint("16thStep", "RUNNING ");
            sixteenth_Step(dir, 16 *200, 800);
            LCD_SpecialPrint("16thStep","FINISHED");
            msecs(500);

            LCD_SpecialPrint("32ndStep", "RUNNING ");
            thirtieth_Step(dir, 32 *200, 300);
            LCD_SpecialPrint("32ndStep","FINISHED");
            msecs(500);

            LCD_SpecialPrint("FncyStep", "RUNNING ");
            fancy_Step(dir, (long long)5*32*200, 5, 1, 2, 2*32, 250 ,17, 300);
            LCD_SpecialPrint("FncyStep","FINISHED");
            msecs(500);
        }      
    }
    return 0;
}



