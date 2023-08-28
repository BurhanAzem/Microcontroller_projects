/*
 * File:   Pic_App.c
 * Author: lenovo
 *
 * Created on April 11, 2023, 12:57 PM
 */




#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include "my_ser.h"
#include "my_adc.h"
#include "lcd_x8.h"
#define STARTVALUE  3036

unsigned int count_sec = 0;
unsigned int count_min = 0;
unsigned int count_hour = 0;
unsigned int Clock_Mode = 0;
unsigned int Setup_Mode = 0;
unsigned int coolar = 0;
unsigned int heater = 0;
unsigned char BufferSer[32];
float AN[3];



unsigned char ReceivedChar ;
void update_lcd(void);
void update_lcd_timer(void);
void update_lcd_setup(void);
void lcd_welcome(void);
void updete_lcd_AN(void);
void reloadTimer0(void)
{  
    TMR0H = (unsigned char) ((STARTVALUE >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE & 0x00FF );   
}

void Timer0_isr(void)
{
    INTCONbits.TMR0IF = 0;// Must be cleared by software
    count_sec++;
    if(count_sec > 59) { 
        count_min++;
        count_sec = 0;
    }
    if(count_min > 59) { 
        count_hour++;
        count_min = 0;
    }
    if(count_hour > 23){
       count_hour = 0;
    }
    reloadTimer0();
}

void set_clock_mode(void) {
    INTCON3bits.INT1F = 0;  
    delay_ms(20);
    if(PORTBbits.RB1)
       return; 
    if(Clock_Mode)
        Clock_Mode = 0;
    else
        Clock_Mode = 1;
    }
void set_setup_mode(void){
    INTCON3bits.INT2F = 0;
        delay_ms(20);
    if(PORTBbits.RB2)
       return; 
    if(Setup_Mode == 2)
        Setup_Mode = 0;
    else
        Setup_Mode ++;
    }
void __interrupt(high_priority) highIsr (void)
//void interrupt high_priority highIsr(void)
{
    if(INTCONbits.TMR0IF) Timer0_isr();
    else if(INTCON3bits.INT1F) {
            set_clock_mode(); 
            update_lcd();
    }
    else if(INTCON3bits.INT2F) {
            set_setup_mode(); 
            update_lcd();
    }
    //else if(PIR1bits.TXIF &&  PIE1bits.TXIE ) TX_isr();
   
}

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

// This function is needed for measuring speed
void initTimer0(void) {
    T0CON = 0;
    //T0CONbits.T0CS = 0;
    //T0CONbits.PSA = 0;
    //T0CONbits.T08BIT = 1;
    INTCONbits.T0IF = 0;
    T0CONbits.T0PS0 = 1; // 16 prescalar
    T0CONbits.T0PS1 = 1;
    T0CONbits.T0PS2 = 0;
    TMR0H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR0L = (unsigned char) (STARTVALUE & 0x00FF);
    INTCON2 = 0x00;
    INTCONbits.GIE = 1;
    INTCONbits.GIEH = 1;//enable only timer 0 interrupt
    INTCONbits.T0IE = 1;

}

int set_baoundares(){
    if(count_sec > 59) { 
        count_min++;
        count_sec = 0;
    }
    if(count_min > 59) { 
        count_hour++;
        count_min = 0;
    }
    if(count_hour > 23){
       count_hour = 0;
    }
    
    if(count_sec <= 0 && !PORTBbits.RB4 && Setup_Mode == 0) { 
    count_min--;
    count_sec = 59;
    }
    if(count_min <= 0 && !PORTBbits.RB4 && Setup_Mode == 1) { 
        count_hour--;
        count_min = 59;
    }
    if(count_hour <= 0 && !PORTBbits.RB4 && Setup_Mode == 2){
       count_hour = 23;
    }
}

void update_lcd_temp(void){
    unsigned char Buffer[16];
    lcd_gotoxy(9, 1);
    sprintf(Buffer, "/T=%4.2fC", AN[2]);
    lcd_puts(Buffer); 
}


void update_lcd_COOLER(void){
    unsigned char Buffer[16];
    lcd_gotoxy(1, 2);
    char *mode;
    if(coolar){
        mode = "ON";
    }
    else{
       mode = "OFF";
    }
    sprintf(Buffer, "C:%s", mode);
    lcd_puts(Buffer);
    if(heater){
        mode = "ON";
    }
    else{
       mode = "OFF";
    }
    sprintf(Buffer, " | H:%s\t", mode);
    lcd_puts(Buffer);
}


void update_lcd_timer(void){
    unsigned char Buffer[16];
    set_baoundares();
    int ss = count_sec;
    int mm = count_min;
    int hh = count_hour;
    
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "%02d:%02d:%02d", hh, mm, ss); // Display Speed
    lcd_puts(Buffer); 
}

void update_lcd_setup(void){
    unsigned char Buffer[16];
    set_baoundares();
    int ss = count_sec;
    int mm = count_min;
    int hh = count_hour;
    if(!Clock_Mode){
        lcd_gotoxy(1, 3);
        sprintf(Buffer, "Normal         ");
        lcd_puts(Buffer);
    }
    else{
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "setup/%02d:%02d:%02d", hh, mm, ss); // Display Speed
    lcd_puts(Buffer);
    }
}

void updete_lcd_AN(void){
    unsigned char Buffer[16];    
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "%4.2fV | %4.2fV    ", AN[0], AN[1]);
    lcd_puts(Buffer);
}
void update_lcd(void){
    update_lcd_timer();
    update_lcd_temp();
    update_lcd_COOLER();
    update_lcd_setup();
    updete_lcd_AN();
}

void initMode(void){
    
}
// used also for measuring speed
//void interrupt high_priority highIsr(void)//old syntax

void lcd_welcome(void){
    unsigned char Buffer[16];
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "  WELCOME TO  ");
    lcd_puts(Buffer); 
    delay_ms(2000);
    
    lcd_gotoxy(1, 3);
    sprintf(Buffer, " OUR GREAT WORK ");
    lcd_puts(Buffer);
    delay_ms(2000);

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "                ");
    lcd_puts(Buffer); 
    delay_ms(200);
    
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "                ");
    lcd_puts(Buffer); 
    delay_ms(200);
    
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "                ");
    lcd_puts(Buffer);
    delay_ms(200);
    
    lcd_gotoxy(1, 4);
    sprintf(Buffer, "                ");
    lcd_puts(Buffer); 
    delay_ms(200);
 

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "WELCOME FROM ME "); // Display Speed
    lcd_puts(Buffer); 
    delay_ms(2000);

    lcd_gotoxy(1, 3);
    sprintf(Buffer, "BURHAN AZEM AND ");
    lcd_puts(Buffer);
    delay_ms(2000);

    lcd_gotoxy(1, 1);
    sprintf(Buffer, "FROM MY PARTNER "); // Display Speed
    lcd_puts(Buffer);
    delay_ms(2000);

   
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "BASSAM TUFFAHA ");
    lcd_puts(Buffer);
    delay_ms(4000);

}


int main(void) {
    /* Replace with your application code */
    int clock_mode, setup_mode = 0;
    unsigned char channel;
    float voltage;
    float temp;
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    //PORTCbits.RC4 =1;
    lcd_welcome();
    PORTCbits.RC5 = 1;
    send_string_no_lib((unsigned char *) "\r\rReading AN0, AN1, AN2\r\r");
    lcd_putc('\f'); //clears the display
    unsigned char RecvedChar = 0;
    unsigned char SendToSerial = 0;
    int RPS;
    initTimer0();   // These will be used to measure the speed
    initMode();
    INTCON3 = 0x18;
    TRISCbits.RC0 = 1;
    T0CONbits.TMR0ON = 1;
    while (1) {
        clock_mode = Clock_Mode;
        setup_mode = Setup_Mode;
        CLRWDT(); // no need for this inside the delay below
        PORTCbits.RC5 = !PORTCbits.RC5;
        //delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            if(channel == 2)
            {
                voltage = read_adc_voltage((unsigned char) channel);
                AN[channel] = voltage * 100;
            }
            else{
                voltage = read_adc_voltage((unsigned char) channel);
                AN[channel] = voltage;
            }
        }
        delay_ms(20);
        update_lcd();
        if(coolar){
            PORTCbits.RC2 = 1;
        }
        else{
            PORTCbits.RC2 = 0;
        }
        if(heater){
            PORTCbits.RC4 = 1;
        }
        else{
            PORTCbits.RC4 = 0;
        }
        
        if(!clock_mode){
            update_lcd();
            T0CONbits.TMR0ON = 1;
        }
        else{
            update_lcd();
            T0CONbits.TMR0ON = 0;
            switch(setup_mode){
                case 0:
                    if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            count_sec ++;
                        }
                        update_lcd();
                    }
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                            count_sec --;
                        }
                        update_lcd();
                    }
                    break;
                case 1:
                    if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            count_min ++;
                        }
                        update_lcd();
                    }
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                           count_min --;
                        }
                        update_lcd();
                    }
                    break;
                case 2:
                    if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            count_hour ++;
                        }
                        update_lcd();
                    }
                    
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                            count_hour --;
                        }
                        update_lcd();
                    }
                    break;
            }
            reloadTimer0();
            T0CONbits.TMR0ON = 1;
        }
    }
}
