#define F_CPU 16000000UL  // Συχνότητα ρολογιού 16 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>


#define VREF 5.0  // Τάση αναφοράς 5V
#define LCD_RS 2 
#define LCD_E 3

void adc_init() {
	ADMUX = (1 << REFS0) |(1 << MUX0) ;  // Επιλέγω AVCC ως τάσης αναφοράς και ADC1 ως είσοδο
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // διαιρέτης 128 (125kHz)
}

uint16_t adc_read() {
	ADCSRA |= (1 << ADSC);  // Ξεκινάω μετατροπή
	while (ADCSRA & (1 << ADSC));  // Περιμένω μέχρι να ολοκληρωθεί η μετατροπή
	return ADC;
}

void display_voltage(uint16_t adc_value) {
	float voltage = (adc_value / 1024.0) * VREF;  // Υπολογισμός τάσης με βάση την τιμή ADC και το VREF
	unsigned char buffer[16];
	snprintf(buffer, sizeof(buffer), "VOLTAGE : %.2f V", voltage);
	lcd_command(0x80);  // Θέση στην αρχή της οθόνης
	lcd_print(buffer);
}

/*void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++);
	}
	//_delay_ms(500);
}
*/

/*================= Ρουτίνες ===================*/
 void write_2_nibbles (unsigned char data) {
	 // Μεταφέρω 4 υψηλά bit //
	 PORTD = (PIND & 0x0F ) | ( data & 0xF0 );
	 PORTD |= (1 << LCD_E);
	 _delay_us(1);
	 PORTD &= ~(1 << LCD_E);
	 
	 // Μεταφέρω 4 χαμηλά bit //
	 PORTD = (PIND & 0X0F) | (data << 4);
	 PORTD |= (1 << LCD_E);
	 _delay_us(1);
	 PORTD &= ~(1 << LCD_E);
	 
 }
 
 void lcd_data(unsigned char data){
	 PORTD |= (1 << LCD_RS); // RS=1 Δεδομένα
	 write_2_nibbles(data);
	 _delay_us(250);
 }
 
void lcd_command(unsigned char cmd){
	PORTD = ~(1 << LCD_RS); // RS=1 Εντολή
	write_2_nibbles(cmd);
	_delay_us(250);
}

void lcd_clear_display(){
	lcd_command(0x01); //Εντολή καθαρισμού οθόνης
	_delay_ms(3);
}

void lcd_init(){
	_delay_ms(200);
	lcd_command(0x30);
	PORTD |= (1 << LCD_E); // Enable Pulse
	_delay_us(250);
	
	lcd_command(0x30);
	PORTD |= (1 << LCD_E); // Enable Pulse
	_delay_us(250);
	
	lcd_command(0x30);
	PORTD |= (1 << LCD_E); // Enable Pulse
	_delay_us(250);
	
	lcd_command(0x20);
	PORTD |= (1 << LCD_E); // Enable Pulse
	_delay_us(250);
	
	//5x8 κουκίδες χαρακτήρων, 2 γραμμές
	lcd_command(0x28);
	
	//Display on , cursor off
	lcd_command(0x0C);
	
	//Καθαρισμός οθόνης
	lcd_clear_display();
	
	lcd_command(0x06);
	
}

void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++);
	}
	//_delay_ms(500);
}


int main() {
	DDRD = 0xFF;
	DDRC = 0x00;
	lcd_init();
	adc_init();

	while (1) {
		uint16_t adc_value = adc_read();  // Ανάγνωση από τον ADC
		display_voltage(adc_value);  // Εμφάνιση τάσης στην οθόνη
		_delay_ms(1000);  // Αναμονή 1 sec
	}
}
