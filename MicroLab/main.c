/*
 * lab3.3
 * main.c
 *
 * Created: 10/24/2024 10:11:26 AM
 *  Author: pavli
 */ 

/*
 *  main.c
 *  Author: KG
 */ 

#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>

int main(void){
	unsigned int duty[13] = {
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
	int DC_VALUE = 6 ;  // 0<=DC_VALUE<=12 
	int mode = 0 ; //  mode 1 = 0,  mode 2 = 1
	
	unsigned int ADC_VALUE = 0;
	
	// set TMR1 in fast PWM 8bit mode with non-inverted output
	// COM1A1 Συνδεση με pb1
	// CS 100 CLK/256 = 62500 Hz
	// WGM 0101 = Fast PWM, 8-bit
	// prescale = 8
	TCCR1A = (1<<WGM10) |(1 << WGM11)| (1<<COM1A1);
	TCCR1B = (1<<WGM12) | ( 1<<CS12) ;
	
	//REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0000 => select ADC0(pin PC0),
	// ADLAR=0 =>right adjust the ADC result
	ADMUX = 0b01000000 ;
	
    // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
    // ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111 ;
	
	
	DDRB |= 0xFF;         // B as OUTPUT   ?????????
	DDRC |= 0xFE;         // C0 as INPUT for POT1
	DDRD |= 0x00;   // D as INPUT for switches
	
	
    while(1){
		// Check if PD6 is pressed Mode 1
		if (!(PIND & 0b01000000))  mode = 0;
 
		// Check if PD7 is pressed mode 2
		if (!(PIND & 0b10000000))  mode = 1;
        
		_delay_ms(50); // fixing bounce
		
		
        if ( mode == 0 ) { // Mode 1 code ( PD1 αυξανει , PD2 μειωνει
			// Check if PD1 is pressed increase
			if ((!(PIND & 0b00000010) && DC_VALUE!=12)) DC_VALUE = DC_VALUE + 1;

			// Check if PD7 is pressed decrease
			if ((!(PIND & 0b00000100) &&  DC_VALUE!=0)) DC_VALUE = DC_VALUE - 1;
		    						
		    _delay_ms(50); // fixing bounce
		 
		    OCR1A = duty[DC_VALUE];

   			
			   
		} else { // Mode 2 code
			ADC_VALUE = 0; 
		    
			// Getting 16 measures and using average
		    for (int i=0; i<16; i++){
				// Set ADSC flag of ADCSRA			
		    	ADCSRA = ADCSRA | (1<<ADSC);
			
		    	while ((ADCSRA & (1<<ADSC))) _delay_ms(1); //Wait until ADSC flag gets 0
			    
				ADC_VALUE += ADC;			
		    }
		    
			ADC_VALUE = ADC_VALUE >> 4;// divide by 16			
			// ADC_VALUE = ADC_VALUE << 6 // scale
			// instead of dividing by 16 and then mult again to reach 16bits in total
			// i will just use the total sum which in some way is the 16bit average
			
			 OCR1A = ADC_VALUE;						 
        };
		
    };//while
	
}//main
