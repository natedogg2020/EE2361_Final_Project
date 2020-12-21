/*
 * File:   Final_Project_main_v001.c
 * Author: Nate McKelvey and Hai Nguyen
 *
 * Created on November 12, 2020, 11:07 PM
 */

#include "xc.h"
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



void setup(void){
    DRV8825_Setup();
    LCD_Setup();
}



int main(void) {
    setup();        // Setup all peripherals
//    run = 8;        // Initially Start with running with all operations
    int dir = 0;    // Set direction of stepper motor (1 or 0/HIGH or LOW)
    msecs(50);      // Wait 50 milliseconds to allow DRV8825 to catch-up initially
    
    while(1){
        // Drive Stepper Motor with certain settings given the 'run' setting.
        // See _IC2Interrupt in DRV_main_v001.c to see how this is changed.
        if (run == 1){
            LCD_SpecialPrint("FullStep", "RUNNING ");   // Display Stepping Status Update (DSSU)
            // Step 200 steps (1 rotation) with a ~10,001.25us period per step using 1:1 microstepping.
            full_Step(dir, 200, 5000);   
            LCD_SpecialPrint("FullStep", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 2){
            LCD_SpecialPrint("1:2_Step", "RUNNING ");   // DSSU
            // Step 400 steps (1 rotation) with a ~7,202.5us period per step using 1:2 microstepping.
            half_Step(dir, 2 *200, 3600);
            LCD_SpecialPrint("1:2_Step", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 3){
            LCD_SpecialPrint("1:4_Step", "RUNNING ");   // DSSU
            // Step 800 steps (1 rotation) with a ~5,002.5us period per step using 1:4 microstepping.
            quarter_Step(dir, 4*200, 2500);
            LCD_SpecialPrint("1:4_Step", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 4){
            LCD_SpecialPrint("1:8_Step", "RUNNING ");   // DSSU
            // Step 1,600 steps (1 rotation) with a ~3,202.5us period per step using 1:8 microstepping.
            eighth_Step(dir, 8*200, 1600);
            LCD_SpecialPrint("1:8_Step", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 5){
            LCD_SpecialPrint("16thStep", "RUNNING ");   // DSSU
            // Step 3,200 steps (1 rotation) with a ~1,602.5us period per step using 1:16 microstepping.
            sixteenth_Step(dir, 16*200, 800);
            LCD_SpecialPrint("16thStep", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 6){
            LCD_SpecialPrint("32ndStep", "RUNNING ");   // DSSU
            // Step 6,400 steps (1 rotation) with a ~302.5us period per step using 1:32 microstepping.
            thirtieth_Step(dir, 32*200, 300);
            LCD_SpecialPrint("32ndStep","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else if (run == 7){
            LCD_SpecialPrint("FncyStep", "RUNNING ");   // DSSU
            /* Step 32,000 steps (5 rotations) using 1:32 microstepping. 
             * Acceleration of ~2us (Input Parameter (IP) of 1) per step, deceleration of 
             * ~4us (IP of 2) per step, with a multiplier of 64 (IP of 2*32) to
             * further slow the accel/decel. Start Speed period is ~502.5us (IP of 250),
             * Max Speed period is ~36.5us (IP of 17), and End Speed is 602.5us
             * (IP of 300).
             */
            fancy_Step(dir, (long long)11*16*200,5, 1, 5, 2*16, 250 ,25, 300);
            LCD_SpecialPrint("FncyStep","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }else{  //run == 8
            LCD_SpecialPrint("FullStep", "RUNNING ");   // Display Stepping Status Update (DSSU)
            // Step 200 steps (1 rotation) with a ~10,001.25us period per step using 1:1 microstepping.
            full_Step(dir, 200, 5000);
            LCD_SpecialPrint("FullStep", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("HalfStep", "RUNNING ");   // DSSU
            // Step 400 steps (1 rotation) with a ~7,202.5us period per step using 1:2 microstepping.
            half_Step(!dir, 2 *200, 3600);
            LCD_SpecialPrint("HalfStep", "FINISHED");   // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("4th Step", "RUNNING ");   // DSSU
            // Step 800 steps (1 rotation) with a ~5,002.5us period per step using 1:4 microstepping.
            quarter_Step(dir, 4 *200, 2500);
            LCD_SpecialPrint("4th Step","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("8th Step", "RUNNING ");   // DSSU
            // Step 1,600 steps (1 rotation) with a ~3,202.5us period per step using 1:8 microstepping.
            eighth_Step(!dir, 8*200, 1600);
            LCD_SpecialPrint("8th Step","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("16thStep", "RUNNING ");   // DSSU
            // Step 3,200 steps (1 rotation) with a ~1,602.5us period per step using 1:16 microstepping.
            sixteenth_Step(dir, 16 *200, 800);
            LCD_SpecialPrint("16thStep","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("32ndStep", "RUNNING ");   // DSSU
            // Step 6,400 steps (1 rotation) with a ~302.5us period per step using 1:32 microstepping.
            thirtieth_Step(!dir, 32 *200, 300);
            LCD_SpecialPrint("32ndStep","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message

            LCD_SpecialPrint("FncyStep", "RUNNING ");   // DSSU
            /* Step 35,200 steps (5.5 rotations) using 1:32 microstepping 
             * (Input Parameter (IP) of 5). Acceleration of ~2us (IP of 1) per step, 
             * deceleration of ~10us (IP of 5) per step, with a multiplier of 32 
             * (IP of 2*16) to further slow the accel/decel. Start Speed period 
             * is ~502.5us (IP of 250), Max Speed period is ~52.5us (IP of 25), 
             * and End Speed is 602.5us (IP of 300).
             */
            fancy_Step(dir, (long long)11*16*200,5, 1, 5, 2*16, 250 ,25, 300);
            LCD_SpecialPrint("FncyStep","FINISHED");    // DSSU
            msecs(500);     // wait half a second to allow user to read LCD message
        }     
        
        dir = !dir;     // Reverse Direction of Stepper Motor  
    } 
    return 0;
    
}



