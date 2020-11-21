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

int START_SPEED = 2500; //Set starting speed to 5000 us bursts, 10,000 step period
int MAX_SPEED = 50;    //Max speed is in microseconds (min microseconds)

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
}

void msecs(int n)
{
    TMR1 = 0x00; //Clear T1 register
    _T1IF = 0; //Clear the T1 interrupt flag
    int i; 
    for(i=0; i<n; i++){
        while (!_T1IF); //Wait for TMR1 to overflow
        _T1IF=0; 
    }
}

void setMode(unsigned char mode){
    if(mode <8 ){   //Actually good mode number
        _RB13 = mode >>2;       //Plugged into M2
        _RB12 = (mode >>1)%2;   //Plugged into M1
        _RB11 = mode%2;         //Plugged into M0
    }
}

void full_Step(int steps, int delay){
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

void quarter_Step(int steps, int delay){
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

void sixteenth_Step(int steps, int delay){
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

void thirtieth_Step(int steps, int delay){
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

void fancy_Step( int steps, unsigned char mode, int accel, int decel){
    int i = 0;
    int speed = START_SPEED;
    
    setMode(mode);
     while(i<steps){
         _RB14 = 1;
         delay_us(speed);
         _RB14 = 0;
         delay_us(speed-1);
         if(((steps - i) <= ((START_SPEED-speed)/decel)) && 
            (speed <= START_SPEED)){ //Start decelerating
             
             speed += decel;
         }else if(speed > MAX_SPEED){   //accelerate
             speed -= accel;
         }  //else, keep the same speed
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

int main(void) {
    setup();
    int dir = 0;
//    _RB15 = dir;
    msecs(50);
    delay_us(1);
    delay_us(2);
    delay_us(5);
    delay_us(10);
    delay_us(50000);
    
    msecs(10);
    while(1){
        
        
        _RB15 = !_RB15;
         msecs(50);
        fancy_Step(32*200, 5, 5, 10);
        msecs(500);
//        full_Step(200, 5000);
//        msecs(500);
//        half_Step(2 *200, 3600); 
//        msecs(500);
//        quarter_Step(4 *200, 2500);
//        msecs(500);
//        eighth_Step(8*200, 1600);
//        msecs(500);
//        sixteenth_Step(16 *200, 800);
//        msecs(500);
//        thirtieth_Step(32 *200, 300);
//        msecs(500);
       
    }
    return 0;
}
