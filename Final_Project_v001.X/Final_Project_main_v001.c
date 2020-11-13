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
 * 
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

void setup(void){
    CLKDIVbits.RCDIV = 0;  //Set RCDIV=1:1 (default 2:1) 32MHz or FCY/2=16M
    // Setup T1 for 1 milli-second delay
    T1CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR1 = 0x00; //Clear contents of the timer register
    PR1 = 249 ; //Set period to reset after 249
    T1CONbits.TCKPS = 0b10; //Set prescale to 1:64
    IFS0bits.T1IF = 0; //Clear the T1 interrupt flag
    IEC0bits.T1IE = 0; //Disable T1 interrupts
    T1CONbits.TON = 1; //Start T1 
    
    
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

void full_Step(char dir, int steps){
    
}

void half_Step(char dir, int steps){
    
}

void quarter_Step(char dir, int steps){
    
}

void eighth_Step(char dir, int steps){
    
}

void sixteenth_Step(char dir, int steps){
    
}

void fancy_Step(char dir, int steps, int accel, int decel){
    
}

int main(void) {
    setup();
    
    while(1){
        
    }
    return 0;
}
