#include "xc.h"
#include "mckel042_LCD_v001.h"



void msecs(int n)
{
    TMR1 = 0x00; 
    IFS0bits.T1IF = 0; 
    int i; 
    for(i=0; i<n; i++)
    {
        while (!_T1IF);
        _T1IF=0; 
    }
}

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

void move(void) {
    lcd_cmd(0b00011000);
}


