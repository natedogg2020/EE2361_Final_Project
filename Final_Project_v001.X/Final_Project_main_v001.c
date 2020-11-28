/*
 * File:   Final_Project_main_v001.c
 * Author: Nate McKelvey and Hai Nguyen
 *
 * Created on November 12, 2020, 11:07 PM
 */

/*


 * TODO:
 *  1.  Could incorporate interrupt to find faults in stepping
 *  2.  Complete full_Step function
 *  3.  Figure out what wiring we should use (Hai do you have a preferred wiring?)

 *      I was thinking we could use RB15 and go down to RB12 or however far we
 *      need for out outputs to the DRV8825. Then RA0 and up could be used for
 *      outputs coming from the DRV8825 to be read. 
 
 We can use any wiring, I tried with RB8 and output compare mode and the motor works. 
 we can also use others way
  
 
 *      If this doesn't sound good, I'd like to collaborate on this.
 * 
 *  4.  Feel free to add/remove things from this list. If this is helpful to have,
 *      I could make a TODO/Completed/Review google doc for us to collaborate
 *      more effectively if you'd be open to it.
 *   
 */
#include "xc.h"
#include "mckel042_LCD_v001.h"
#include "DRV8825_main_v001.h"

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

//#define _ISR __attribute__((interrupt (no_auto_psv)))


int START_SPEED = 1000; //Set starting speed to 5000 us bursts, 10,000 step period
int MAX_SPEED = 560;    //Max speed for full_Stepping is in microseconds (min microseconds)

void delay_us(unsigned int);
void LCD_SpecialPrint(const char top[], const char bottom[]);

void setup(void){
    CLKDIVbits.RCDIV = 0;  //Set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M
    AD1PCFG = 0x9fff;            //sets all pins to digital I/O
    
    DRV8825_Setup();
    LCD_Setup();
}

/* Function:        half_Step
 * Original Author: Hai Nguyen
 * Description:     
 * Parameters:
 *      steps   : 
 *      delay   : 
 */
void half_Step(int steps, int delay){
    setMode(1);
    int i = 0;
     while(i<steps){
         _RB14 = 1;
         delay_us(delay);
         _RB14 = 0;
         delay_us(delay);
         i++;
     }     
}

void eighth_Step(int steps, int delay){
     setMode(3);
    int i = 0;
     while(i<steps){
         _RB14 = 1;
         delay_us(delay);
         _RB14 = 0;
         delay_us(delay);
         i++;
     }    
}

void fancy_Step(int dir, long long steps, unsigned char mode, int accel, int decel,unsigned int  mult, int initial_speed, int max_speed,int end_speed){
    _RB15 = dir;                //Set the direction pin
    long long i = 0;                  //Initialize our iterator, i
    int j =1 ;
    int speed = initial_speed;    //Set initial speed to START_SPEED
    setMode(mode);              //Set microstepping mode
    
    //Start Stepping
     while(i<steps-1){
         _RB14 = 1;
         delay_us(speed);
         _RB14 = 0;
         //speed-1 to compensate for the time to complete speed calculations
         delay_us(speed-2); 
         // speed calculations to determine if speed should be 
         // increased, decreased, or maintained
         if(j >= mult){
            
            if(decel >0){
                if(((steps - i-1) <= (long long)(j*((end_speed-speed)/decel))) && 
                   (speed <= end_speed)){ 
                   //Start decelerating
                   speed += decel;
               }else if(speed > max_speed){   
                    //accelerate
                    speed -= accel;
                }
            }else if(speed > max_speed){   
                //accelerate
                speed -= accel;
            }  //else, keep the same speed, which could be 
               //min: speed, max: speed + 2*decel -1
            j = 1;
         }
        
         i++;
         j++;
         
     } 
    _RB14 = 1;
    delay_us(speed);
    _RB14 = 0;
    //speed-1 to compensate for the time to complete speed calculations
    delay_us(speed-3); 
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
    int i=0;
    while(1){
        
        dir = !dir;
//        _RB15 = !_RB15;
//         msecs(50);
//        LCD_SpecialPrint("FullStep", "RUNNING ");
//        full_Step(dir, 200, 5000);
//        LCD_SpecialPrint("FullStep", "FINISHED");
//        msecs(500);
//
//        LCD_SpecialPrint("HalfStep", "RUNNING ");
//        half_Step(2 *200, 3600); 
//        LCD_SpecialPrint("HalfStep", "FINISHED");
//        msecs(500);
//
//        LCD_SpecialPrint("4th Step", "RUNNING ");
//        quarter_Step(dir, 4 *200, 2500);
//        LCD_SpecialPrint("4th Step","FINISHED");
//        msecs(500);
//
//        LCD_SpecialPrint("8th Step", "RUNNING ");
//        eighth_Step(8*200, 1600);
//        LCD_SpecialPrint("8th Step","FINISHED");
//        msecs(500);
//
//        LCD_SpecialPrint("16thStep", "RUNNING ");
//        sixteenth_Step(dir, 16 *200, 800);
//        LCD_SpecialPrint("16thStep","FINISHED");
//        msecs(500);
//
//        LCD_SpecialPrint("32ndStep", "RUNNING ");
//        thirtieth_Step(dir, 32 *200, 300);
//        LCD_SpecialPrint("32ndStep","FINISHED");
//        msecs(500);

        /*
         4th stepping extra special speed acceleration
         fancy_Step(dir, 32*200, 2, 1, 0,START_SPEED ,MAX_SPEED, MAX_SPEED);
        fancy_Step(dir, 32*200, 2, 1, 0, MAX_SPEED, 150, 150);
        fancy_Step(dir, 32*200, 2, 1, 0, 150, 135, 135);
        fancy_Step(dir, 32*200, 2, 1, 0, 135, 130, 130);
        fancy_Step(dir, 32*200, 2, 1, 5, 130, 130,START_SPEED); 
         */
        LCD_SpecialPrint("FncyStep", "RUNNING ");
        fancy_Step(dir, (long long)5*32*200, 5, 1, 2, 2*32, 300 ,17, 17);
        fancy_Step(dir, (long long)10*32*200, 5, 1, 2, 128, 17 ,17, 300);
//        fancy_Step(dir, (long long)10*200, 1, 5, 5, 4, START_SPEED ,300, START_SPEED);
//        fancy_Step(dir, (long long)5*32*200, 5, 1, 0, 32, 25 ,23, 23);
//        fancy_Step(dir, (long long)5*32*200, 5, 1, 1, 32, 23 ,20, START_SPEED);
//        fancy_Step(dir, 204800, 5, 1, 0,20 ,20, 20);
//        fancy_Step(dir, 32*200, 5, 1, 0, MAX_SPEED, 150, 150);
//        fancy_Step(dir, 32*200, 5, 1, 0, 150, 135, 135);
//        fancy_Step(dir, 32*200, 5, 1, 0, 135, 130, 130);
//        fancy_Step(dir, 32*200, 5, 1, 0, 135, 130, 130);
//        fancy_Step(dir, 32*200, 5, 1, 0, 135, 130, 130);
//        fancy_Step(dir, 32*200, 5, 1, 5, 130, 130,START_SPEED); 
//        fancy_Step(dir, 32*200, 2, 1, 5,START_SPEED ,MAX_SPEED,START_SPEED);
        LCD_SpecialPrint("FncyStep","FINISHED");
        msecs(500);
    }
    return 0;
}


