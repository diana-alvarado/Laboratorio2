/*
 * File:   USART.c
 * Author: motad
 *
 * Created on 
 */


#include <xc.h>
#include "USART.h"
#include <stdlib.h>
#include <stdio.h>
#include <pic16f887.h>

#define _XTAL_FREQ 8000000

void USART_setup(int frec, int trans, int rec){
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 1;
    
    switch (frec){
        case 1: //1MHz
            SPBRG = 25;
            SPBRGH = 0;
            break;
        
        case 2: //2MHz
            SPBRG = 51;
            SPBRGH = 0;
            break;
            
        case 3: //4MHz
            SPBRG = 103;
            SPBRGH = 0;
            break;
            
        case 4: //8MHz
            SPBRG = 207;
            SPBRGH = 0;
            break;
    }
    
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    TXSTAbits.TX9 = 0;
    
    if(rec == 1)
        RCSTAbits.CREN = 1;
    else 
        RCSTAbits.CREN = 0;
    
    if(trans == 1)
        TXSTAbits.TXEN = 1;
    else 
        TXSTAbits.TXEN = 0;
    
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
}
void cadena (char *str){
        while(*str != '\0'){
            TX_usart(*str);
            str++;
        }
    }

void TX_usart(char data)
    {
        while(TXSTAbits.TRMT == 0);
        TXREG = data;
    }
char RX_resultado()
    {
        return RCREG;
    }
