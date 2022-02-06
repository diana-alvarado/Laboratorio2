/* 
 * File:   Valores del ADC
 * Author: Diana Alvarado
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ADC_H
#define	ADC_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>


//Variables

//Prototipos
void ADC_setup(void);
char ADC_channel(void);

#endif	/* XC_HEADER_TEMPLATE_H */
