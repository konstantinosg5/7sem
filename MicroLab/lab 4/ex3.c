#define F_CPU 16000000UL // Set CPU frequency to 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LCD_RS 2
#define LCD_E 3
uint16_t adc_value = 0;
float COcons = 0.0;
float temp=0.0;

void adc_init() {
	// REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC1(pin PC1),
	// ADLAR=0 =>right adjust the ADC result      0b01000010 	
	ADMUX = (1 << REFS0) | (1 << MUX0) ; 
	
	 // Enable ADC, Enable ADC interupts, prescaler = 128
	ADCSRA = (1 << ADEN) | ( 1<< ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void timer1_init() {
	// Set CTC mode and prescaler to 256
	TCCR1B = (1 << WGM12) | (1 << CS12);
	OCR1A = 6249; // Set compare value for 100 ms at 16 MHz with prescaler 256
	TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
}


ISR(ADC_vect) {	
	adc_value = ADC; // Return ADC value
}

ISR(TIMER1_COMPA_vect) {
	// This function is called every 100 ms
	ADCSRA |= (1 << ADSC); // Start ADC conversion
}

//======================================================
 void write_2_nibbles(int data){
uint8_t r25 = PIND & 0x0F; //read PIND

uint8_t high_byte = (data & 0xF0) + r25;
PORTD = high_byte;
PORTD |= (1 << PD3); // Set PD3 high
_delay_us(1);        // Short delay
PORTD &= ~(1 << PD3); // Set PD3 low

 uint8_t low_byte = ((data & 0x0F) << 4) + r25;

 // Output the low byte to PORTD
 PORTD = low_byte;

 // Enable pulse for low nibble
 PORTD |= (1 << PD3); // Set PD3 high
 _delay_us(1);        // Short delay
 PORTD &= ~(1 << PD3); // Set PD3 low
}



//=======================================================
void lcd_command(unsigned char code){
	PORTD = ~( 1<<2 ); //LCD_RS=0(PD2=0), Instruction
	write_2_nibbles(code); // send Instruction
	_delay_us(250); // Wait 250uSec
}

//=======================================================
void lcd_data(unsigned char data){
	PORTD |= (1 << 2) ; //LCD_RS=1(PD2=1), Data
	write_2_nibbles(data) ; //send data
	_delay_us(250); // Wait 250uSec
}

//=======================================================
void lcd_clear_display(){
	lcd_command (0x01) ; //clear display command
	_delay_ms(5) ; // Wait 5 mSec
}

//===========================================================
void lcd_init(){
/*;Αρχικοποίηση και ρυθμίσεις της οθόνης LCD όπως παρουσιάζεται παρακάτω:
;DL = 0 4 bit mode
;N = 1 2 lines
;F = 0 5×8 dots
;D = 1 display on
;C = 0 cursor off
;B = 0 blinking off
;I/D = 1 DD-RAM address auto increment
;SH = 0 shift of entire display off
*/
_delay_ms(200);
PORTD=0x30 ;      // command to switch to 8 bit mode
PORTD |= (1<< 3) ;// Enable Pulse
_delay_us(1);
PORTD = 0x00;
_delay_us(250);   //Wait 250uSec

_delay_ms(200);
PORTD=0x30 ;      // command to switch to 8 bit mode
PORTD |= (1<< 3) ;// Enable Pulse
_delay_us(1);
PORTD = 0x00;
_delay_us(250);   //Wait 250uSec

_delay_ms(200);
PORTD=0x30 ;      // command to switch to 8 bit mode
PORTD |= (1<< 3); // Enable Pulse
_delay_us(1);
PORTD = 0x00;
_delay_us(250);   // Wait 250uSec

PORTD = 0x20;     // command to switch to 4 bit mode
PORTD |= (1<< 3); // Enable Pulse
_delay_us(1);
PORTD = 0x00;
_delay_us(250);

lcd_command(0x28); // 5x8 dots, 2 lines

lcd_command(0x0c) ; // dislay on, cursor off
lcd_clear_display();

lcd_command(0x06) ; //Increase address, no display shift ;

}




void LCD_GasDetected(){
	lcd_data('G');
	lcd_data('A');
	lcd_data('S');
	lcd_data(' ');
	lcd_data('D');
	lcd_data('E');
	lcd_data('T');
	lcd_data('E');
	lcd_data('C');
	lcd_data('T');
	lcd_data('E');
	lcd_data('D');
}

void LCD_COclear(){
	lcd_data('C');
	lcd_data('L');
	lcd_data('E');
	lcd_data('A');
	lcd_data('R');
}

int main(void) {
DDRD=0xFF;
DDRB=0xFF;
DDRC=0x00;

lcd_init();    // Initialize LCD
adc_init();    // Initialize ADC
timer1_init(); // Initialize Timer1

sei(); // Enable global interrupts

while (1) {
	temp = ((adc_value * 3)/1024.0) - 0.1;
	
	COcons=((adc_value*3)/1024.0-0.1)/0.013;
	
	if(COcons<37){
	    PORTB=0x01;
		LCD_COclear();
		_delay_ms(1500);
	}
	else if(COcons<70){
	    PORTB=0x03;
		LCD_COclear();
		_delay_ms(1500);
		
	}
	else if(COcons<107){
	    PORTB=0x07;
		LCD_GasDetected();
		_delay_ms(750);
		PORTB=0x00;		
		_delay_ms(750);
		
    }else if(COcons<150){
        PORTB=0b00001111;
		LCD_GasDetected();
		_delay_ms(750);
		PORTB=0x00;
		_delay_ms(750);
		
    }else if(COcons<187) {
        PORTB=0b00011111;
		LCD_GasDetected();
		_delay_ms(750);
		PORTB=0x00;
		_delay_ms(750);
		
    }else {
        PORTB=0b00111111;
		LCD_GasDetected();
		_delay_ms(750);
		PORTB=0x00;
		_delay_ms(750);
    }
	
    lcd_clear_display();	


}

return 0;
}
