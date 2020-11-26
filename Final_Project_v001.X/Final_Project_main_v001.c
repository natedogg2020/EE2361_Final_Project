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

//#include "DRV8825_main_v001.h"

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

int START_SPEED = 2000; //Set starting speed to 5000 us bursts, 10,000 step period
int MAX_SPEED = 170;    //Max speed is in microseconds (min microseconds)

void delay_us(unsigned int);

void setup(void){
    CLKDIVbits.RCDIV = 0;  //Set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M
    AD1PCFG = 0x9fff;            //sets all pins to digital I/O
    // Setup T1 for 1 milli-second delay
    T1CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR1 = 0x00; //Clear contents of the timer register
    PR1 = 249 ; //Set period to reset after 249
    T1CONbits.TCKPS = 0b10; //Set prescale to 1:64
    IFS0bits.T1IF = 0; //Clear the T1 interrupt flag
    IEC0bits.T1IE = 0; //Disable T1 interrupts
    T1CONbits.TON = 1; //Start T1 
    
    //DRV8825 Initializations
//    TRISA = 0b0000000000011111;  //set port A to inputs, 
    TRISB = 0b0000000000000011;  //and port B to outputs
//    LATA = 0xffff;               //Set all of port A to HIGH
    LATB = 0xffff;               //and all of port B to HIGH
    //RB15 is DIR (Direction) pin, RB14 is STEP pin
    
    //LCD Initialization
    I2C2CONbits.I2CEN = 0;      // Disable I2C2
    I2C2BRG = 0x9D;             // Set Baud Rate Generator
    I2C2CONbits.I2CEN = 1;      // Enable I2C2
    IFS3bits.MI2C2IF = 0;       // Clear flag
    lcd_init();
    lcd_setCursor(0, 0);
}

/* Function:        msecs 
 * Original Author: Nathaniel McKelvey
 * Description:     delays n amount of miilliseconds
 * Parameters:      
 *      n    : determines amount of milliseconds to delay
 */
//void msecs(int n)
//{
//    TMR1 = 0x00; //Clear T1 register
//    _T1IF = 0; //Clear the T1 interrupt flag
//    int i; 
//    for(i=0; i<n; i++){
//        while (!_T1IF); //Wait for TMR1 to overflow
//        _T1IF=0; 
//    }
//}

/* Function:        setMode 
 * Original Author: Nathaniel McKelvey
 * Description:     setMode sets the DRV8825 with variable microstepping
 *      mode = 0 :  Full Stepping
 *             1 :  Half Stepping
 *             2 :  1/4th Stepping
 *             3 :  1/8th Stepping
 *             4 :  1/16th Stepping
 *             5 :  1/32nd Stepping
 * Parameters:      
 *      mode    : Determines the direction of rotation (1 for CW, 0 for CCW)
 */
void setMode(unsigned char mode){
    if(mode <8 ){   //Actually good mode number
        _RB13 = mode >>2;       //Plugged into M2
        _RB12 = (mode >>1)%2;   //Plugged into M1
        _RB11 = mode%2;         //Plugged into M0
    }
}

void full_Step(int dir, int steps, int delay){
     _RB15 = dir;                //Set the direction pin
    setMode(0);
    int i = 0;
     while(i<steps){
         _RB14 = 1;
         delay_us(delay);
         _RB14 = 0;
         delay_us(delay);
         i++;
     } 

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

/* Function:        quarter_Step 
 * Original Author: Nathaniel McKelvey
 * Description:     quarter_Step drives the DRV8825 with 1/4th microstepping
 * Parameters:      
 *      dir     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   : Number of steps to rotate
 *      delay   : delay between steps in microseconds. 2*delay is the period
 */
void quarter_Step(int dir, int steps, int delay){
     _RB15 = dir;                //Set the direction pin
    setMode(2);
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

/* Function:        sixteenth_Step 
 * Original Author: Nathaniel McKelvey
 * Description:     sixteenth_Step drives the DRV8825 with 1/16th microstepping
 * Parameters:      
 *      dir     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   : Number of steps to rotate
 *      delay   : delay between steps in microseconds. 2*delay is the period
 */
void sixteenth_Step(int dir, int steps, int delay){
    _RB15 = dir;                //Set the direction pin
    setMode(4);
    int i = 0;
     while(i<steps){
         _RB14 = 1;
         delay_us(delay);
         _RB14 = 0;
         delay_us(delay);
         i++;
     } 
}

/* Function:        thirtieth_Step 
 * Original Author: Nathaniel McKelvey
 * Description:     thirtieth_Step drives the DRV8825 with 1/32nd microstepping
 * Parameters:      
 *      dir     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   : Number of steps to rotate
 *      delay   : delay between steps in microseconds. 2*delay is the period
 */
void thirtieth_Step(int dir, int steps, int delay){
    _RB15 = dir;                //Set the direction pin
    setMode(5);
    int i = 0;
     while(i<steps){
         _RB14 = 1;
         delay_us(delay);
         _RB14 = 0;
         delay_us(delay);
         i++;
     } 
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

//Adds an additional 1.25us to the delay
void delay_us(unsigned int us){
    while(us-- >0){
        asm("repeat #3");
        asm("nop");
        asm("nop");
    }
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
        
        dir = !dir;
//        _RB15 = !_RB15;
//         msecs(50);
        LCD_SpecialPrint("FullStep", "RUNNING ");
        full_Step(dir, 200, 5000);
        LCD_SpecialPrint("FullStep", "FINISHED");
        msecs(500);
        
        LCD_SpecialPrint("HalfStep", "RUNNING ");
        half_Step(2 *200, 3600); 
        LCD_SpecialPrint("HalfStep", "FINISHED");
        msecs(500);
        
        LCD_SpecialPrint("4th Step", "RUNNING ");
        quarter_Step(dir, 4 *200, 2500);
        LCD_SpecialPrint("4th Step","FINISHED");
        msecs(500);
        
        LCD_SpecialPrint("8th Step", "RUNNING ");
        eighth_Step(8*200, 1600);
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
    return 0;
}
