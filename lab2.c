/*
 * File:   lab2.c
 * Author: Diana Alvarado
 *
 * Created on 31 de enero de 2022, 02:19 PM
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//----------------------Librerias--------------------------------

#include <xc.h>
#include "lcd.h"
#include "ADC.h"
#include "USART.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <pic16f887.h>

//-------------------------Constantes
#define _XTAL_FREQ 8000000

/*
 * ----------------------------- Variables ----------------------------------
 */
char casos;
uint8_t cont;
char V1;
char V2;
char variable;
float V1con, V2con = 0;
char V1c[5];
char V2c[5];
char V3c[5];
char centena;
char decena;
char unidad;

#define _4MHz  3
#define trans  1
#define recep  1
/*
 * -------------------------- Prototipos de funciones -----------------------
 */

void setup(void); //configuraciones
void convert(void);
void convertV2(char val);
void convertV3(char valor);


/*
 * ------------------------------Interupciones----------------------------
 */
void __interrupt () isr (void)
{
    if (PIR1bits.ADIF)   // Interrupcion ADC
    {  
        if(ADCON0bits.CHS == 0)
            V1 = ADRESH;
        else
            V2 =  ADRESH;
        PIR1bits.ADIF =0;

    }
    if(PIR1bits.RCIF)
    {
        variable = RCREG;          
        PIR1bits.RCIF = 0;
        
    }
    return;
}

/*
 * ----------------------------- MAIN CONFIGURACION --------------------------- 
 */
void main (void)
{
    //setup de ADC, USART y demás
    setup();
    ADC_setup();
    USART_setup(_4MHz,trans,recep);
    __delay_us(50);
    ADCON0bits.GO = 1;
    
    //LCD
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("S1:   S2:   S3:");
    
    
    //cadena("\r 1 para ver V1 y V2 \r");
    //cadena("\r + inc V3 \r");
    //cadena("\r -  dec V3 \r");
        
    while(1)
    {
        
        convert();
        ADC_channel();
        V1con = (V1*0.01961);
        V2con = (V2*0.01961);
        
        Lcd_Set_Cursor(2,1);                //2da fila 
        sprintf(V1c,"%.2fV", V1con);    //Se convierte a string
        Lcd_Write_String(V1c);      //los strings obtenidos
        sprintf(V1c," %.2fV ", V2con);
        Lcd_Write_String(V1c);
        sprintf(V1c, " %d", centena);
        Lcd_Write_String(V1c);
        sprintf(V1c, "%d", decena);
        Lcd_Write_String(V1c);
        sprintf(V1c, "%d ", unidad);
        Lcd_Write_String(V1c);
       
        
        switch(variable)
        {
            case('1'):
                sprintf(V1c,"%.2fV", V1con);
                cadena(" V1: \r");
                cadena(V1c);
                cadena("\r");
                sprintf(V2c," %.2fV ", V2con);
                cadena(" V2: \r");
                cadena(V2c);
                variable = 0;
                break;
            case('+'):
                cont++;
                variable = 0;
                break;
            case('-'):
                cont--;
                variable = 0;
                break;
        }
        
    }
}

/*
 * -------------------------------- Funciones --------------------------------
 */
void setup(void)
{
    //Configuración entradas y salidas
    ANSEL = 0b00000011;
    ANSELH = 0;
    
    TRISA = 0b11;
    TRISB = 0;
    TRISD = 0;
    TRISE = 0;
   
    //Limpiar todos los puertos
    PORTA = 0; 
    PORTB = 0; 
    PORTD = 0; 
    PORTE = 0; 
    
    //Configuracion del oscilador 
    OSCCONbits.IRCF = 0b0110; // 4MHz
    OSCCONbits.SCS = 1;
    
    //------------------ Configuración de las interrupciones  -----------------
    INTCONbits.GIE = 1; //habilitar interrupciones
    INTCONbits.PEIE = 1;        //habilitar int perifericas
    PIE1bits.ADIE = 1;          //habilitiar int analogicas 
    __delay_us(50);
    
    return; 

}
void convert (void){
    centena = (cont/100);
    decena = ((cont-(centena*100))/10);
    unidad = (cont-(centena*100 + decena*10));
    return;
}



