/*
 * File:   DRV8825_main_v001.c
 * Author: Nate McKelvey and Hai Nguyen
 *
 * Created on November 12, 2020
 */

#include "xc.h"                 //For the _IC1Interrupt
#include "DRV8825_main_v001.h"

    /*
<<<<<< PIC Port |   DRV8825 Port/Pin        |   Active High/Low >>>>>>
     * RB15     |   DIR (Direction) pin,    |   CCW/CW (External Shaft to Motor View)
     * RB14     |   STEP pin                |   HIGH
     * RB13     |   (M2) Mode2 Pin          |   See setMode function for description
     * RB12     |   (M1) Mode1 Pin          |   ^^^
     * RB11     |   (M0) Mode0 Pin          |   ^^^
     * RB10     |   Sleep Pin               |   LOW
     * RB9      |   Reset Pin               |   LOW
     * RB8      |   Enable Pin              |   LOW 
     * RB4      |   FAULT Pin               |   LOW (IC1 is detecting falling edge)
    */

//Globals for IC1 fault detection
int MAX_FAULTS = 2;      //Disable DRV8825 if Fault is reached n amount of times
int NUM_FAULTS = 0;

//Globals for IC2 motor control
unsigned int set = 1;
unsigned int run = 1;


void DRV8825_Setup(void){
//DRV8825 Initializations
    TRISB = 0b0000000000010011;  //and port B to outputs
    TRISBbits.TRISB15 = 0;          // Output   |   DIR (Direction) pin
    TRISBbits.TRISB14 = 0;          // Output   |   STEP pin 
    TRISBbits.TRISB13 = 0;          // Output   |   (M2) Mode2 Pin 
    TRISBbits.TRISB12 = 0;          // Output   |   (M1) Mode1 Pin 
    TRISBbits.TRISB11 = 0;          // Output   |   (M0) Mode0 Pin   
    TRISBbits.TRISB10 = 0;          // Output   |   Sleep Pin
    TRISBbits.TRISB9 = 0;           // Output   |   Reset Pin  
    TRISBbits.TRISB8 = 0;           // Output   |   Enable Pin
    TRISBbits.TRISB4 = 1;           // Input    |   Fault Pin
    LATB |= 0xfe00;               //and all used port B to HIGH except RB8 (Enable Pin = 0v)

// Setup T1 for 1 milli-second delay
    T1CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR1 = 0x00; //Clear contents of the timer register
    PR1 = 249 ; //Set period to reset after 249
    T1CONbits.TCKPS = 0b10; //Set prescale to 1:64
    IFS0bits.T1IF = 0; //Clear the T1 interrupt flag
    IEC0bits.T1IE = 0; //Disable T1 interrupts
    T1CONbits.TON = 1; //Start T1 
    
// Input Capture 1/ Timer 2 Setup for Fault Detection
    RPINR7bits.IC1R = 4;  // assign IC1 to RP4
    T2CON = 0x0000;        // prescale 1:1,     
    TMR2 = 0x0000;         // initialize to 0    
    PR2 = 0XFFFF;          // max period
    /* input compare       */    
    IC1CON = 0x0000;       // Reset IC1    
    IC1CON = 0x0082;       // Turn on Input Capture 1 Module
    /* configure interrupts */       
    IPC0bits.IC1IP = 4;    // Set module interrupt priority as 1    
    IFS0bits.IC1IF = 0;    // Clear the IC1 interrupt status flag    
    IEC0bits.IC1IE = 1;    // Enable IC1 interrupts    
    T2CONbits.TON = 1;     // enable Timer2
    
 // Input Capture 2/ Timer 3 Setup for motor control via BUTTON
    //BUTTON  connect RP5_pin
    TMR3 = 0;
    PR3 = 0xfffe;
    T3CONbits.TCKPS = 0b00;         //prescale value 1:1
    T3CONbits.TCS = 0b0;            //Internal clock (FOSC/2)
    T3CONbits.TGATE = 0b0;      //Gated time accumulation disabled (when TCS = 0)
    T3CONbits.TON = 1;  
    
    IC2CONbits.ICTMR = 0;   // Select Timer3 for IC2 Time
    IC2CONbits.ICI = 0b00;  // Interrupt on every capture event
    IC2CONbits.ICM = 0b011; // Generate capture event on every Rising edge
    
  //  IPC0bits.IC2IP = 3; // Setup IC1 interrupt priority level
    IFS0bits.IC2IF = 0; // Clear IC1 Interrupt Status Flag
    IEC0bits.IC2IE = 1; // Enable IC1 interrupt
        __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPINR7bits.IC2R = 5;   // Use pin RP5 for IC2
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock PPS
}

/* Function:        msecs 
 * Original Author: Nathaniel McKelvey
 * Description:     delays 'n' amount of milliseconds
 * Parameters:      
 *      n    : determines amount of milliseconds to delay
 */ 
void msecs(int n){
    TMR1 = 0x00; //Clear T1 register
    _T1IF = 0; //Clear the T1 interrupt flag
    int i; 
    for(i=0; i<n; i++){
        while (!_T1IF); //Wait for TMR1 to overflow
        _T1IF=0; 
    }
}

/* Function:        delay_us 
 * Original Author: Nathaniel McKelvey
 * Description:     delays 'us' amount of  microseconds. NOTE that this delay 
 *                  adds an additional 1.25us to the input 'us' delay
 * Parameters:      
 *      us    : determines amount of microseconds ('us' + 1.25 microsec) to delay
 */
void delay_us(unsigned int us){
    while(us-- >0){
        asm("repeat #3");
        asm("nop");
        asm("nop");
    }
}

/* Function:        setMode 
 * Original Author: Nathaniel McKelvey
 * Description:     setMode sets the DRV8825 with variable micro-stepping
 *      mode = 0 :  Full Stepping
 *             1 :  Half Stepping
 *             2 :  1/4th Stepping
 *             3 :  1/8th Stepping
 *             4 :  1/16th Stepping
 *             5 :  1/32nd Stepping
 * Parameters:      
 *      mode    : Determines the type of micro-stepping used
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
 *      dir     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   : Number of steps to rotate
 *      delay   : delay between steps in microseconds. 2*delay is the period
 */
void half_Step(int dir,int steps, int delay){
      _RB15 = dir; 
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

/* Function:        eighth_Step
 * Original Author: Hai Nguyen
 * Description:     
 * Parameters:
 *      dir     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   : Number of steps to rotate
 *      delay   : delay between steps in microseconds. 2*delay is the period
 */
void eighth_Step(int dir, int steps, int delay){
    _RB15 = dir;
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

/* Function:        fancy_Step 
 * Description:     fancy_Step drives the DRV8825 with any microstepping, along 
 *                  with acceleration, deceleration, and an accel/decel 
 *                  multiplier (mult) that allows for even slower accel/decel.
 * Parameters:      
 *      dir     ---     : Determines the direction of rotation (1 for CW, 0 for CCW)
 *      steps   ---     : Number of steps to rotate
 *      mode    ---     : Sets the steping mode. SetMode states:
 *                          mode =  0 :  Full Stepping
 *                                  1 :  Half Stepping
 *                                  2 :  1/4th Stepping
 *                                  3 :  1/8th Stepping
 *                                  4 :  1/16th Stepping
 *                                  5 :  1/32nd Stepping
 *      accel   ---     : determines the acceleration of the change in stepping delay.
 *                          This means the stepping period is decreased by 2*accel every
 *                          'mult' steps (with a mult of 1, the step period is decreased
 *                          by 2*accel every step until 'max_speed' is achieved.
 *      decel   ---     : determines the rate of deceleration in stepping delay.
 *                          This is just like acceleration, except deceleration is 
 *                          calculated to begin once the amount of steps required = the 
 *                          amount of steps remaining
 *      mult    ---     : Allows for a slower accel/decel by forcing 'mult' amount of 
 *                          steps before accel/decel. mult must be at a minimum of 1.
 * 
 *      initial_speed   : This is the initial delay (speed or period/2) that the function starts at. 
 *      max_speed       : This is the max speed (smallest delay) that the delay can reach.
 *      end_speed       : This is the target end speed that the function ends with
 */
void fancy_Step(int dir, long long steps, unsigned char mode, int accel, int decel,
                unsigned int  mult, int initial_speed, int max_speed,int end_speed){
    _RB15 = dir;                //Set the direction pin
    long long i = 0;            //Initialize our iterator, i
    int j =1 ;                  // Initialize j iterator for accel/decel multiplier (mult)
    int speed = initial_speed;  //Set initial speed
    setMode(mode);              //Set microstepping mode
    
    //Start Stepping
     while(i<steps-1){
         //Command DRV8825 to step
         _RB14 = 1;
         delay_us(speed);
         _RB14 = 0;
         //speed-2 to compensate for the time to complete speed calculations
         delay_us(speed-2); 
         // speed calculations to determine if speed should be 
         // increased, decreased, or maintained
         if(j >= mult){
            // Make sure the user wants to decelerate
            if(decel >0){ 
                //Check if its time to start deceleration, continue acceleration,
                //or maintain current speed (do nothing)
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
            }  //else, keep the same speed
            
            j = 1; // Reset j, since mult has been reached.
         }
        
         i++;
         j++;
     }
    //Step one last time with less delay at the end to accommodate for return time
    _RB14 = 1;
    delay_us(speed);
    _RB14 = 0;
    //speed-3 to compensate for the time to return from function
    delay_us(speed-3); 
}

/* Function:        _IC1Interrupt 
 * Original Author: Nathaniel McKelvey
 * Description:     _IC1Interrupt is utilized to halt the DRV8825 if the MAX_FAULTS
 *                  is reached. If the MAX_FAULTS isn't reached, the DRV8825
 * Parameters: None
 */
void __attribute__((interrupt, auto_psv)) _IC1Interrupt(void){              
    
    NUM_FAULTS++;    //Increase Number of faults detected
    if(NUM_FAULTS >= MAX_FAULTS){
        _RB9 = 0;   //Reset Pin set to 0V
        _RB8 = 1;   //Disable DRV8825
        while(1){
            msecs(50);
            LCD_SpecialPrint("MaxFault","Reached");
            msecs(4000);
            LCD_SpecialPrint("Check   ","Wiring  ");
            msecs(4000);
        }
    }else{
        msecs(200);
        LCD_SpecialPrint("FAULT   ", "DETECTED ");
        _RB9 = 0;   //Reset Pin set to 0V
        msecs(4000);
        LCD_SpecialPrint("Reseting", "DRV8825 ");
        msecs(3000);
        //Reset the flag just before the DRV8825 is restored power to look for
        //new faults that may occur right away.
        IFS0bits.IC1IF = 0; // Clear the IC1 interrupt status flag 
        _RB9 = 1;   //Reset Pin given 3.3V
    }
}



void __attribute__((__interrupt__, __auto_psv__)) _IC2Interrupt(void) {
    set++;
    if(set == 9)           // We can choose one in eight different modes
        set = 1;
   
    if(set == 1){
         LCD_SpecialPrint("SETTING ", "1:1_step");
        //Displaying the setting mode
    }
    if(set == 2){
         LCD_SpecialPrint("SETTING ", "1/2_step");
    }
    if(set == 3){
         LCD_SpecialPrint("SETTING ", "Qtr_step");
    }
    if(set == 4){
         LCD_SpecialPrint("SETTING ", "8th_step");
    }
    if(set == 5){
         LCD_SpecialPrint("SETTING ", "16thStep");
    }  
    if(set == 6){
         LCD_SpecialPrint("SETTING ", "32ndStep");
    } 
    if(set == 7){
         LCD_SpecialPrint("SETTING ", "FncyStep");
    }  
    if(set == 8){
         LCD_SpecialPrint("SETTING ", "ALL_Step");
    } 

    msecs(1000);   //Wait 1_sec for setting is comlete
    _IC2IF = 0;  //Reset Flag at the end to help debounce the button
}



