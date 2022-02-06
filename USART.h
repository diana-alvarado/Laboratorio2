/* 
 * File:   Valores del ADC
 * Author: Diana Alvarado
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef USART_H
#define	USART_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>


//Variables

//Prototipos
void USART_setup(int frec, int trans, int rec);
void cadena (char *str);
void TX_usart(char data);
char RX_resultado();

#endif	/* XC_HEADER_TEMPLATE_H */


