/*
 *  main.c
 *  Author: KG
 */ 

#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>

int main(void){
	unsigned int duty[13]={
		
		20,   // 2%
		102,  // 10%
		184,  // 18%
		266,  // 26%
		348,  // 34%
		430,  // 42%
		512,  // 50%
		594,  // 58%
		676,  // 66%
		758,  // 74%
		840,  // 82%
		922,  // 90%
		1004  // 98%
	};
	
	int DC_VALUE = 6 ;           // 0<=DC_VALUE<=12 	
	unsigned int avg_adc = 0;
	int count=0;                // count for 16 adc
	
	// set TMR1 in fast PWM 10bit mode with non-inverted output
	// COM1A1 Συνδεση με pb1
	// CS 100 CLK/256 = 62500 Hz
	// WGM 0111 = Fast PWM, 10-bit
	// prescale = 8
	TCCR1A = (1<<WGM10) | (1<< WGM11) | (1<<COM1A1);
	TCCR1B = (1<<WGM12) | ( 1<<CS12) ;
	
	//REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC0(pin PC0),
	// ADLAR=0 =>right adjust the ADC result
	ADMUX = 0b010000001 ;
	
    // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    // ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111 ;
	
	
	DDRB = 0xFF;   // B as OUTPUT for PWM
	DDRC = 0x00;   // C as INPUT for ADC
	DDRD = 0x1F;   // D as OUTPUT for PD0-PD4 and INPUT for rest PD7 INCR, PD6 DEC 
	
	
 while(1){	
     // Check if PD7 is pressed increase
     if ((!(PIND & 0b10000000) && DC_VALUE!=12)) DC_VALUE = DC_VALUE + 1;
     
     // Check if PD6 is pressed decrease
     if ((!(PIND & 0b01000000) &&  DC_VALUE!=0)) DC_VALUE = DC_VALUE - 1;
        						
      
      OCR1A = duty[DC_VALUE];  	   
		    
	  // Getting 16 measures and using average
	 
	 // Set ADSC flag of ADCSRA			
	 ADCSRA = ADCSRA | (1<<ADSC);
	  _delay_ms(100); // fixing bounce 
	  
	 while ((ADCSRA & (1<<ADSC))) _delay_ms(1); //Wait until ADSC flag gets 0
	      
	 avg_adc = avg_adc+ADC;			
	  count=count+1;
		    	
			
	// Ανάλογα με την τιμή του ADC, ανάβουμε το κατάλληλο LED
    if (count==16){
		count=0;
		
		avg_adc = avg_adc >> 4; // divide by 16		
    	// PORTD &= ~(0x1F);                  // Σβήνουμε όλα τα LED
    	 if (avg_adc <= 200) {
    	     	PORTD = (1 << 0);           // Ανάβουμε το PD0
    	 } else if (avg_adc <= 400) {
    	     	PORTD = (1 << 1);           // Ανάβουμε το PD1
    	 } else if (avg_adc <= 600) {
    	    	PORTD = (1 << 2);           // Ανάβουμε το PD2
    	 } else if (avg_adc <= 800) {
    	    	PORTD = (1 << 3);           // Ανάβουμε το PD3
    	  } else {
    	    	PORTD = (1 << 4);           // Ανάβουμε το PD4
    	 }	
		avg_adc = 0;			 
    }
		
    };//while
	
}//main
