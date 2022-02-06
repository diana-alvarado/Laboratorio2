/* 
 * File:   valores.c
 * Author: motad
 *
 * Created on 27 de enero de 2022, 02:30 AM
 */

#include "ADC.h"
#include <stdlib.h>
#include <stdio.h>
#include <pic16f887.h>
#include <xc.h>

#define _XTAL_FREQ 8000000
//Funciones
void ADC_setup(){
    ADCON1bits.ADFM = 0;        //Justificaión izquierda
    ADCON1bits.VCFG0 = 0;       //Vref en VSS Y VDD
    ADCON1bits.VCFG1 = 0;  
   
    ADCON0bits.ADCS = 0b01;     //FOSC/8
    ADCON0bits.CHS = 0;
    ADCON0bits.ADON = 1;
    __delay_us(50);
    
    PIR1bits.ADIF = 0; //bandera ADc
    PIE1bits.ADIE = 1; //generales ADC
    
}
char ADC_channel(void){
    if (ADCON0bits.GO == 0){
        if(ADCON0bits.CHS == 0)
            ADCON0bits.CHS = 1;
        else
            ADCON0bits.CHS = 0;
        __delay_us(50);
        ADCON0bits.GO = 1;
    }
}
