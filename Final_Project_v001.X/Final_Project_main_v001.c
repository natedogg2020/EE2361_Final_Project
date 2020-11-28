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
unsigned int set = 1;
unsigned int run = 1;

int START_SPEED = 2000; //Set starting speed to 5000 us bursts, 10,000 step period
int MAX_SPEED = 170;    //Max speed is in microseconds (min microseconds)

void delay_us(unsigned int);
void LCD_SpecialPrint(const char top[], const char bottom[]);

void setup(void){
    CLKDIVbits.RCDIV = 0;  //Set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M
    AD1PCFG = 0x9fff;            //sets all pins to digital I/O
    
    DRV8825_Setup();
    LCD_Setup();
 

//BUTTON  connect RP5_pin
   
     T3CONbits.TON = 0;
    T3CONbits.TCKPS = 0b11;         //.11 = 1:256 prescale value
    T3CONbits.TCS = 0b0;            //.0 = Internal clock (FOSC/2)
    T3CONbits.TGATE = 0b0;      //.0 = Gated time accumulation disabled (when TCS = 0)
    
    TMR3 = 0;
    PR3 = 0xffff;
    T3CONbits.TON = 1; // Start 16-bit Timer2
    
   
    IFS0bits.T3IF = 0; // Clear T2 Interrupt Status Flag
    IEC0bits.T3IE = 1; // Enable T2 interrupt
    
    
    IC2CONbits.ICTMR = 0;   // Select Timer3 for IC1 Time
    IC2CONbits.ICI = 0b00;  // Interrupt on every capture event
    IC2CONbits.ICM = 0b011; // Generate capture event on every Rising edge
    
  //  IPC0bits.IC2IP = 2; // Setup IC1 interrupt priority level
    IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC2IE = 1; // Enable IC1 interrupt
        __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR7bits.IC2R = 5;
   // RPOR3bits.RP6R = 18; // Use Pin RP6 for Output Compare 1 = "18" (Table 10-3)
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
 
}


void fancy_Step(int dir, int steps, unsigned char mode, int accel, int decel){
    _RB15 = dir;                //Set the direction pin
    int i = 0;                  //Initialize our iterator, i
    int speed = START_SPEED;    //Set initial speed to START_SPEED
    setMode(mode);              //Set microstepping mode
    
    //Start Stepping
     while(i<steps){
         _RB14 = 1;
         delay_us(speed);
         _RB14 = 0;
         //speed-1 to compensate for the time to complete speed calculations
         delay_us(speed-1); 
         // speed calculations to determine if speed should be 
         // increased, decreased, or maintained
         if(((steps - i) <= ((START_SPEED-speed)/decel)) && 
                (speed <= START_SPEED)){ 
             //Start decelerating
             speed += decel;
         }else if(speed > MAX_SPEED){   
             //accelerate
             speed -= accel;
         }  //else, keep the same speed, which could be 
            //min: speed, max: speed + 2*decel -1
         i++;
     } 
}

void LCD_SpecialPrint(const char top[], const char bottom[]){
    lcd_setCursor(0, 0);
    lcd_printStr(top);
    lcd_setCursor(0, 1);
    lcd_printStr(bottom);
}


void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void) {
    _T3IF = 0;      
}
void __attribute__((__interrupt__, __auto_psv__)) _IC2Interrupt(void) {
    _IC2IF = 0;
   msecs(10);
   set++;
   if(set == 9)set = 1;
   
        if(set == 1){
             LCD_SpecialPrint("SETTING", "Full_step");
        }
        if(set == 2){
             LCD_SpecialPrint("SETTING", "Halt_step");
        }
        if(set == 3){
             LCD_SpecialPrint("SETTING", "Quater_step");
        }
        if(set == 4){
             LCD_SpecialPrint("SETTING", "Eighth_step");
        }
        if(set == 5){
             LCD_SpecialPrint("SETTING", "16thStep");
        }  
        if(set == 6){
             LCD_SpecialPrint("SETTING", "32ndStep");
        } 
        if(set == 7){
             LCD_SpecialPrint("SETTING", "FncyStep");
        }  
        if(set == 8){
             LCD_SpecialPrint("SETTING", "ALL_Step");
        } 

   msecs(1000);
}


int main(void) {
    setup();
    int dir = 0;
    msecs(50);
    set = 8;  // run all steps
     
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
                LCD_SpecialPrint("half_Step", "RUNNING ");
                half_Step(dir, 2 *200, 3600);
                LCD_SpecialPrint("half_Step", "FINISHED");                
            }
            if (run == 3){
                msecs(500);           
                LCD_SpecialPrint("quarter_Step", "RUNNING ");
                quarter_Step(dir, 4 *200, 2500);
                LCD_SpecialPrint("quarter_Step", "FINISHED");                
            }
            if (run ==4){
                msecs(500);           
                LCD_SpecialPrint("eighth_Step", "RUNNING ");
                eighth_Step(dir, 8*200, 1600);
                LCD_SpecialPrint("eighth_Step", "FINISHED");                
            }
            if (run == 5){
                msecs(500);           
                LCD_SpecialPrint("16th_Step", "RUNNING ");
                sixteenth_Step(dir, 16 *200, 800);
                LCD_SpecialPrint("16th_Step", "FINISHED");                
            }            
            if (run == 6){
                LCD_SpecialPrint("32ndStep", "RUNNING ");
                thirtieth_Step(dir, 32 *200, 300);
                LCD_SpecialPrint("32ndStep","FINISHED");
                msecs(500);
            }               
            if (run == 7){
                LCD_SpecialPrint("FncyStep", "RUNNING ");
                fancy_Step(dir, 32*200, 2, 1, 5);
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
                fancy_Step(dir, 32*200, 2, 1, 5);
                LCD_SpecialPrint("FncyStep","FINISHED");
                msecs(500);
            }      
    }
    return 0;
}


