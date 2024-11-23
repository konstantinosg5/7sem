
/*
 * main.c
 *
 *  Author: KG
 */ 

#define F_CPU 16000000UL

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define LCD_RS 2
#define LCD_E 3

#define PCA9555_0_ADDRESS 0x40 //A0=A1=A2=0 by hardware
#define TWI_READ 1 // reading from twi device
#define TWI_WRITE 0 // writing to twi device
#define SCL_CLOCK 100000L // twi clock in Hz

//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2

// PCA9555 REGISTERS
typedef enum {
	REG_INPUT_0 = 0,
	REG_INPUT_1 = 1,
	REG_OUTPUT_0 = 2,
	REG_OUTPUT_1 = 3,
	REG_POLARITY_INV_0 = 4,
	REG_POLARITY_INV_1 = 5,
	REG_CONFIGURATION_0 = 6,
	REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;

//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58

#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)


//initialize TWI clock
void twi_init(void){
	TWSR0 = 0; // PRESCALER_VALUE=1
	TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}


// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void){
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}

//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void){
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}
// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address){
	uint8_t twi_status;
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));
	
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) )
	{
		return 1;
	}
	return 0;
}


// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address){
	uint8_t twi_status;
	while ( 1 ){
		// send START condition
		TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// wait until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
		// send device address
		TWDR0 = address;
		TWCR0 = (1<<TWINT) | (1<<TWEN);
		// wail until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) )
		{
			/* device busy, send stop condition to terminate write operation */
			TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			// wait until stop condition is executed and bus released
			while(TWCR0 & (1<<TWSTO));
			continue;
		}
		break;
	}
}



// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data ){
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}


// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address){
	return twi_start( address );
}


// Terminates the data transfer and releases the twi bus
void twi_stop(void){
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}


void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value){
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}


uint8_t PCA9555_0_read(PCA9555_REGISTERS reg){
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	
	return ret_val;
}


//================================================================== FOR LCD ================
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
// ==============================================================================================
// ==============================  EVERYTHING ABOVE WAS GIVEN  ==================================
// ==============================================================================================

// Το port1_0 εχει τιμη 0, ειναι δηλαδη pulled downed , οταν ενωθει με ενα απο τα port1_4-7 (που ειναι
// pulled up) θα τα κανει απο 1  να δειξουν 0. Συνεπως, η εισοδος ειναι αντιστροφης λογικης.
uint8_t scan_row(uint8_t row){ // row should be int from 1 to 4 
	uint8_t input;
	row = ~( 1<<(row-1) );  // it leaves 0 only to the line that should be read 
	
	PCA9555_0_write(REG_OUTPUT_1, row); //Set EXT_PORT1_0 as output and rest input
	
	//PCA9555_0_write(REG_INPUT_1,0x00);
	input = PCA9555_0_read(REG_INPUT_1);
	input = ( (~input & 0xF0) >> 4);
	
	return input;  //it return the pressed keys in the lsbs
	
}


uint16_t scan_keyboard (){
	uint16_t keyboard = 0;
	for (uint8_t i=1; i<=4; i++){
		keyboard |= ( scan_row(i)<<( 4*(i-1) ) );
	}
	
	return keyboard;
}



uint16_t pressed_keys; // Moναδα στο αντιστοιχο bit αντιστοιχει σε πιεσμενο πληκτρο, 
                       // μετραμε απο κατω προς τα πανω και απο δεξια προς τα αριστερα
					   // Δηλαδη το bit0 αντιστοιχει στο '*', το bit1 στο '0'κτλ
void scan_keypad_rising_edge(){
	uint16_t pressed_keys_tempo;
	
	pressed_keys_tempo = scan_keyboard();
	_delay_ms(20);
	
	pressed_keys_tempo &= scan_keyboard();
	
	pressed_keys=pressed_keys_tempo;
	
}


uint8_t keyboard_to_ascii(){
	scan_keypad_rising_edge();
	if (pressed_keys==0) return 0;
	if (pressed_keys<0x000F){
		if (pressed_keys == 0x0001) return '*' ;
		if (pressed_keys == 0x0002) return '0' ;
		if (pressed_keys == 0x0004) return '#' ;
		if (pressed_keys == 0x0008) return 'D' ;
		
	}else if (pressed_keys<0x00100){
		if (pressed_keys == 0x0010) return '7' ;
		if (pressed_keys == 0x0020) return '8' ;
		if (pressed_keys == 0x0040) return '9';
		if (pressed_keys == 0x0080) return 'C';
		
	}else if (pressed_keys<0x1000){
		if (pressed_keys == 0x0100) return '4';
		if (pressed_keys == 0x0200) return '5';
		if (pressed_keys == 0x0400) return '6';
		if (pressed_keys == 0x0800) return 'B';
	
	}else {
		if (pressed_keys == 0x1000) return '1';
		if (pressed_keys == 0x2000) return '2';
		if (pressed_keys == 0x4000) return '3';
		if (pressed_keys == 0x8000) return 'A';
	}
	return 0; // if more than one pressed
}




int main(void){
	twi_init();
	DDRD = 0xFF; // output
    
	lcd_init();
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00); //Set EXT_PORT0 as output LEDS    ΑΝΤΙΣΤΡΟΦΗ ΛΟΓΙΚΗ
	PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); 
	
	uint8_t charPressed;
    while(1){		
        charPressed = keyboard_to_ascii();
		if ( charPressed!=0 ) lcd_data(charPressed);
		 		 
		lcd_command(0x80); // cursor to first line
    }
}
