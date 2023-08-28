/*
 * File:   Cooking_Main.c
 * Author: lenovo
 *
 * Created on May 2, 2023, 12:17 PM
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
#include <stdlib.h>
#include <string.h>
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE0  3036
#define STARTVALUE3  0x63C0

unsigned int Buzzer_ON = 0;
int count_sec = 0;
 int count_min = 0;
 int count_hour = 0;
float Cooking_Temp = 0;
float SetUp_Point = 0;
unsigned int Setting_Mode = 0;
unsigned int Cooking_Mode = 0;
unsigned int Cooking = 0;
unsigned int Heating = 0;
unsigned int Hysteresis = 0;
unsigned int Percent_Heat_Counter = 0;
unsigned int Int_Heat_Counter = 0;
float AN[3];

void update_lcd(void);
void update_lcd_Line1(void);
void update_lcd_Line2(void);
void update_lcd_Line3(void);
void update_lcd_Line4(void);

void Timer0_Handling(void);
void Timer3_Int_Handling(void); 
void setting_Mode_Int_Handling(void); 
void Start_Int_Handling(void); 
void Stop_Int_Handling(void);

void reloadTimer0(void)
{  
    TMR0H = (unsigned char) ((STARTVALUE0 >>  8) & 0x00FF);
    TMR0L =  (unsigned char)(STARTVALUE0 & 0x00FF );   
}


void reloadTimer3(void)
{  
    TMR3H = (unsigned char) ((STARTVALUE3 >>  8) & 0x00FF);
    TMR3L =  (unsigned char)(STARTVALUE3 & 0x00FF );   
}

void Timer0_Int_Handling(void)
{
    INTCONbits.TMR0IF = 0;
    if(count_hour == 0 && count_min == 0 && count_sec == 0)
    {
        Buzzer_ON = 1;
        Cooking = 0;
        Heating = 0;
        update_lcd();
        return;
    }
    count_sec--;
    if(count_sec < 0) { 
        count_min--;
        count_sec = 59;
    }
    if(count_min < 0) { 
        count_hour--;
        count_min = 59;
    }
    if(count_hour < 0){
       count_hour = 0;
    }
    update_lcd();
    reloadTimer0();
}

void Heating_Control(void)
{
    if(Int_Heat_Counter > 0 && Int_Heat_Counter <= Percent_Heat_Counter)
        {
            PORTCbits.RC5 = 1;
            Heating = 1;
        }
        else if(Int_Heat_Counter > Percent_Heat_Counter && Int_Heat_Counter <= 10)
        {
            PORTCbits.RC5 = 0;
            Heating = 0;
            if(Int_Heat_Counter == 10)
            {
                Int_Heat_Counter = 0;
                return ;
            }
        }
    Int_Heat_Counter ++;

}


void Timer3_Int_Handling(void)
{
    PIR2bits.TMR3IF = 0;  

    if(Cooking == 0)
    {
        Heating = 0;
        return;
    }
    if(Cooking_Mode == 0)
    {
        Heating_Control();
    }
    else
    {
        unsigned int error = 0;
        if(Cooking_Temp > (SetUp_Point + Hysteresis))
        {
//            Heating = 0;
            PORTCbits.RC5 = 0;
        }
        else {
             error = (int)(SetUp_Point - Cooking_Temp); // round to integer
             if(error > 10) error = 10;// makes Heat percentage to max 100%
             else if( error < 5) error = 5; //makes Heat Percentage to 50% in the PWM 
             Percent_Heat_Counter = error; //Generates the PWM by Timer3 interrupt described
        }   
        Heating_Control();
    }
    reloadTimer3();
}

void setting_Mode_Int_Handling(void)
{
    INTCONbits.INT0F = 0;
        delay_ms(40);
    if(PORTBbits.RB0)
       return; 
        
    if(Setting_Mode == 6)
        Setting_Mode = 0;
    else
        Setting_Mode ++;
}

void Start_Int_Handling(void)
{
    INTCON3bits.INT1F = 0;
        delay_ms(20);
    if(PORTBbits.RB1)
       return; 
    if(count_hour != 0 || count_min != 0 || count_sec != 0)
    {
        Cooking = 1;
        Heating = 1;
        T0CONbits.TMR0ON = 1;
    }
}

void Stop_Int_Handling(void)
{
    INTCON3bits.INT2F = 0;
        delay_ms(20);
    if(PORTBbits.RB2)
       return; 
        
    Cooking = 0;
    Heating = 0;
    T0CONbits.TMR0ON = 0;
}



void __interrupt(high_priority) highIsr (void)
//void interrupt high_priority highIsr(void)
{
    if(INTCONbits.TMR0IF) Timer0_Int_Handling();
    if(PIR2bits.TMR3IF) Timer3_Int_Handling(); 
    if(INTCONbits.INT0F) setting_Mode_Int_Handling(); 
    if(INTCON3bits.INT1F) Start_Int_Handling(); 
    if(INTCON3bits.INT2F) Stop_Int_Handling();
}
void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    TRISB = 0x3F; // all pushbuttons are inputs
    TRISC = 0x00; // RX input , others output
    TRISA = 0x25; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

// This function is needed for measuring speed
void initTimers(void) {
    T0CON = 0b00000011;
    T3CON = 0b00001000;

    reloadTimer3();
    reloadTimer0();
    
    INTCON = 0b11110000;
    INTCON2 = 0b00000000;
    INTCON3 = 0b00011000;
    
    PIE1 = 0b00000100;
    PIR1 = 0b00000000;
    IPR1 = 0b00000000;
    PIE2 = 0b00000010;
    PIR2 = 0b00000000;
    IPR2 = 0b00000000;
}

void set_baoundares(void){
    if(count_sec > 59) { 
        count_min++;
        count_sec = 0;
    }
    if(count_min > 59) { 
        count_hour++;
        count_min = 0;
    }
    if(count_hour > 5){
       count_hour = 5;
    }
    
    if(count_sec <= 0 && !PORTBbits.RB4 && Cooking_Mode != 5 && Cooking_Mode != 6) { 
    count_min--;
    count_sec = 59;
    }
    if(count_min <= 0 && !PORTBbits.RB4 && Cooking_Mode != 5 && Cooking_Mode != 6) { 
        count_hour--;
        count_min = 59;
    }
    if(count_hour <= 0 && !PORTBbits.RB4 && Cooking_Mode != 5 && Cooking_Mode != 6){
       count_hour = 5;
    }
}






void update_lcd_Line1(void){
    unsigned char Buffer[16];
    set_baoundares();
    int ss = count_sec;
    int mm = count_min;
    int hh = count_hour;
    
    lcd_gotoxy(1, 1);
    sprintf(Buffer, "Time: %02d:%02d:%02d  ", hh, mm, ss); // Display Speed
    lcd_puts(Buffer); 
}

void update_lcd_Line2(void){
    unsigned char Buffer[16];
    float CT = Cooking_Temp;
    unsigned char *CK;
    if(!Cooking){
        CK = "OFF";
    }
    else{
        CK = "ON";
    }
    lcd_gotoxy(1, 2);
    sprintf(Buffer, "CT:%03.1fC CK:%3s", CT, CK);
    lcd_puts(Buffer);
}

void update_lcd_Line3(void){
    unsigned char Buffer[16];
    float SP = SetUp_Point;
    unsigned char *HT;
    if(!Heating){
        HT = "OFF";
    }
    else{
        HT = "ON";
    }
    lcd_gotoxy(1, 3);
    sprintf(Buffer, "SP:%03.1fC HT:%3s", SP, HT);
    lcd_puts(Buffer);
}
char* intToString(int number) {
    // Determine the length of the string representation
    int length = sprintf(NULL, 0, "%d", number);
    
    // Allocate memory for the string representation
    char* str = (char*)malloc((length + 1) * sizeof(char));
    
    // Convert the integer to a string
    sprintf(str, "%d", number);
    
    return str;
}
void update_lcd_Line4(void){
    unsigned char Buffer[16];
    int H = Hysteresis;
    unsigned char *MD;
    unsigned char *P;
    
    switch(Setting_Mode)
    {
        case 0:
            MD = "Sec";
            break;
        case 1:
            MD = "10S";
            break;
        case 2:
            MD = "Min";
            break;
        case 3:
            MD = "10M";
            break;
        case 4:
            MD = "HR";
            break;
        case 5:
            MD = "HYS";
            break;
        case 6:
            MD = "HT%";
            break;
    }
    
    if(!Cooking_Mode){
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "MD:%s P:%d  H:%d", MD, Percent_Heat_Counter * 10, H);
        lcd_puts(Buffer);
        //sprintf(P, "%d H:", Percent_Heat_Counter * 10);
        //itoa(Percent_Heat_Counter * 10, P, 10);
        //P = intToString(Percent_Heat_Counter * 10);
//        int length = snprintf( NULL, 0, "%d", Percent_Heat_Counter * 10);
//        char* str = malloc( length + 1 );
//        snprintf( str, length + 1, "%d", Percent_Heat_Counter * 10);
//        P = str;
    }
    else{
        //P = "Aut H:";
        lcd_gotoxy(1, 4);
        sprintf(Buffer, "MD:%s P:Aut H:%d", MD, H);
        lcd_puts(Buffer);
    }

}
void update_lcd(void){
    update_lcd_Line1();
    update_lcd_Line2();
    update_lcd_Line3();
    update_lcd_Line4();
}

void main(void) {
    //ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    char Buffer[32]; // for sprintf
    int raw_val;
    unsigned char channel;
    float voltage;
    
    setupPorts();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    initTimers();
    
    lcd_putc('\f'); //clears the display    
    while (1) {
        CLRWDT(); // no need for this inside the delay below
        delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
        update_lcd();
        set_baoundares();
        for (channel = 0; channel < 3; channel++)
        {
            // read the adc voltage
            switch(channel)
            {
                case 0:
                    voltage = read_adc_voltage((unsigned char) channel);
                    SetUp_Point = (voltage * 300) / 5.0;
                    break;
                case 2:
                    voltage = read_adc_voltage((unsigned char) channel);
                    Cooking_Temp = voltage * 300;
                    break;
                    
            }
        }
    
        if(!PORTBbits.RB5)
        {
            Cooking = 0;
            Heating = 0;
            T0CONbits.TMR0ON = 0;
        }
        
        if(!PORTAbits.RA5)
        {
            if(Cooking_Mode)
                Cooking_Mode = 0;
            else
                Cooking_Mode = 1;
        }
        
        if(Cooking == 0)
        {
            T3CONbits.TMR3ON = 0;
            T0CONbits.TMR0ON = 0;
            Heating = 0;
        }
        else
        {
            T3CONbits.TMR3ON = 1;
            T0CONbits.TMR0ON = 1;
        }
        
        if(PORTCbits.RC5 == 0)
        {
            Heating = 0;
        }
        else
        {
            Heating = 1;
        }
        
        switch(Setting_Mode)
        {
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
                            count_sec += 10;
                        }
                        update_lcd();
                    }
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                            count_sec -= 10;
                        }
                        update_lcd();
                    }
                break;
            case 2:
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
            case 3:
                if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            count_min += 10;
                        }
                        update_lcd();
                    }
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                           count_min -= 10;
                        }
                        update_lcd();
                    }
                break;
            case 4:
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
            case 5:
                if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            if(Hysteresis >= 0 && Hysteresis <= 4)
                                Hysteresis ++;
                        }
                        update_lcd();
                    }
                    
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                            if(Hysteresis >= 0 && Hysteresis <= 4)
                                Hysteresis --;
                        }
                        update_lcd();
                    }
                break;
            case 6:
                    if(!PORTBbits.RB3){
                        delay_ms(250);
                        if(!PORTBbits.RB3)
                        {
                            if(Percent_Heat_Counter >= 0 && Percent_Heat_Counter < 10)
                                Percent_Heat_Counter ++;
                        }
                        update_lcd();
                    }
                    
                    if(!PORTBbits.RB4){
                        delay_ms(250);
                        if(!PORTBbits.RB4)
                        {
                            if(Percent_Heat_Counter > 0 && Percent_Heat_Counter <= 10) 
                                Percent_Heat_Counter --;
                        }
                        update_lcd();
                    }
                break;
        }
    }
}
