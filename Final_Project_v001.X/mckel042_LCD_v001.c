/*
 * File:   mckel042_LCD_v001.c
 * Date: 11/26/2020
 * Author: Nathaniel McKelvey
 * Course number: EE2361
 * Term: Fall 2020
 * Lab/assignment number: Lab 5 LCD Library
 * Short Program Description: This program utilizes the I2C2 SDA and SCL pins to
 *      drive the given LCD to display characters, strings, and shift the 
 *      characters across the display
 */

#include "xc.h"
#include "mckel042_LCD_v001.h"

/* Function:        LCD_Setup 
 * Description:     This is the setup function to initialize all required outputs
 *                  and anything else necessary for the LCD library to have 
 *                  full functionality.
 * Parameters:      None
 * Output:          None
 */ 
void LCD_Setup(){
    //LCD Initialization
    I2C2CONbits.I2CEN = 0;      // Disable I2C2
    I2C2BRG = 0x9D;             // Set Baud Rate Generator
    I2C2CONbits.I2CEN = 1;      // Enable I2C2
    IFS3bits.MI2C2IF = 0;       // Clear flag
    lcd_init();
    lcd_setCursor(0, 0);
}

/* Function:        lcd_cmd 
 * Description:     This function is used to send LCD commands to the LCD
 * Parameters:      
 *      command : 8-bit command to send to display
 * Output:          None
 */ 
void lcd_cmd(unsigned char command)
{
    I2C2CONbits.SEN = 1;        // Begin Start sequence
    while(I2C2CONbits.SEN);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = 0b01111100;       // Slave address and R/nw bit
    
    while(!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = 0b00000000;       // Control byte
    
    while(!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = command;          // Data byte
    
    while(!IFS3bits.MI2C2IF);
    I2C2CONbits.PEN = 1;        // Begin Stop sequence
    while(I2C2CONbits.PEN);
}

/* Function:        lcd_init 
 * Description:     This is the required initialization sequence to initially
 *                  start the LCD
 * Parameters:      None
 * Output:          None
 */ 
void lcd_init(void)
{
    msecs(250);
    lcd_cmd(0b00111000); // function set, normal instruction mode
    msecs(1);
    lcd_cmd(0b00111001); // function set, extended instruction mode
    msecs(1);
    lcd_cmd(0b00010100); // interval osc
    msecs(1);
    lcd_cmd(0x70);  //contrast set
    msecs(1);
    lcd_cmd(0x56);  //contrast control
    msecs(1);
    lcd_cmd(0b01101100); // follower control
    msecs(210);
    lcd_cmd(0b00111000); // function set, normal instruction mode
    msecs(1);
    lcd_cmd(0b00001100); // Display On
    msecs(1);
    lcd_cmd(0b00000001); // Clear Display
    msecs(20);
}

/* Function:        lcd_setCursor 
 * Description:     This function allows for setting the cursor to a specific
 *                  character position on a 40x2 character space (note the 
 *                  display can only show 8x2 characters at a time. move() must
 *                  utilized to make the remaining characters show across 
 *                  the display
 * Parameters:      
 *          x   :   The x-position for the character cursor. Range = [0, 39]
 *          y   :   The y-position for the character cursor. can be 0 or 1.
 * Output:          None
 */ 
void lcd_setCursor(char x, char y)
{
    int i = 0;
    lcd_cmd(0b00000010);    //Return cursor to home
    msecs(1);
    while( i < (x + (0x28 * y))){
        lcd_cmd(0b00010100); //Move cursor to the right
        msecs(1);
        i++; 
    }
}

/* Function:        lcd_printChar 
 * Description:     This function prints a single character to the LCD at the
 *                  pre-set cursor position.
 * Parameters:      
 *          myChar  : ASCII character to be displayed to the LCD  
 * Output:          None
 */ 
void lcd_printChar(char myChar)
{
    I2C2CONbits.SEN = 1;        // Begin Start sequence
    while(I2C2CONbits.SEN);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = 0b01111100;       // Slave address and R/nw bit
    
    while(!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = 0b01000000;       // Control byte
    
    while(!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0; 
    I2C2TRN = myChar;           // Data byte
    
    while(!IFS3bits.MI2C2IF);
    I2C2CONbits.PEN = 1;        // Begin Stop sequence
    while(I2C2CONbits.PEN);
}

/* Function:        lcd_printStr 
 * Description:     This function prints a character array to the LCD at the
 *                  pre-set cursor position.
 * Parameters:      
 *          s[]  :  ASCII character array to be displayed to the LCD  
 * Output:          None
 */ 
void lcd_printStr(const char s[])
{
    int len = strlen(s);
    int i;
    
    I2C2CONbits.SEN = 1;        // Begin Start sequence 
    while (I2C2CONbits.SEN);
    IFS3bits.MI2C2IF = 0;
    I2C2TRN = 0b01111100;       // Slave address and R/nw bit
    
    while (!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0;
    for (i = 0; i < len - 1; i++) 
    {
        I2C2TRN = 0b11000000;       // Control byte
        while (!IFS3bits.MI2C2IF);
        IFS3bits.MI2C2IF = 0;
        I2C2TRN = s[i];     // Data byte
        while (!IFS3bits.MI2C2IF);
        IFS3bits.MI2C2IF = 0;
    }
    
    I2C2TRN = 0b01000000;           // Control byte
    while (!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0;
    I2C2TRN = s[len-1];             // Data byte
    while (!IFS3bits.MI2C2IF);
    IFS3bits.MI2C2IF = 0;
    I2C2CONbits.PEN = 1;            // Begin Stop sequence
    while (I2C2CONbits.PEN);

}

/* Function:        move 
 * Description:     Shifts characters to the left across display.
 * Parameters:      None 
 * Output:          None
 */ 
void move(void) {
    lcd_cmd(0b00011000);    // Shifts characters to the left across display.
}


