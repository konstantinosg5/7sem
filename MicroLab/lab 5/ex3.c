/*
 * main.c
 *
 * Created: 11/5/2024 8:41:07 PM
 *  Author: pavli
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
// ==============================================================================================
// ==============================  EVERYTHING ABOVE WAS GIVEN  ==================================
// ==============================================================================================

// Send command to LCD via PCA9555
void lcd_command(uint8_t cmd){
	// Send higher nibble
	uint8_t high_nibble = (cmd & 0xF0) >> 2; // Shift to align with P0_2-P0_5
	// Clear RS and DB lines
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (0 << LCD_E) | (high_nibble));
	// Pulse E
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (1 << LCD_E) | (high_nibble));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (0 << LCD_E) | (high_nibble));
	_delay_us(200);
	
	// Send lower nibble
	uint8_t low_nibble = ((cmd & 0x0F) << 2);
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (0 << LCD_E) | (low_nibble));
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (1 << LCD_E) | (low_nibble));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0, (0 << LCD_RS) | (0 << LCD_E) | (low_nibble));
	_delay_ms(2);
}

// Send data to LCD via PCA9555
void lcd_data(uint8_t data){
	// Send higher nibble
	uint8_t high_nibble = (data & 0xF0) >> 2; // Shift to align with P0_2-P0_5
	// Set RS=1 for data
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (0 << LCD_E) | (high_nibble));
	// Pulse E
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (1 << LCD_E) | (high_nibble));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (0 << LCD_E) | (high_nibble));
	_delay_us(200);
	
	// Send lower nibble
	uint8_t low_nibble = ((data & 0x0F) << 2);
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (0 << LCD_E) | (low_nibble));
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (1 << LCD_E) | (low_nibble));
	_delay_us(1);
	PCA9555_0_write(REG_OUTPUT_0, (1 << LCD_RS) | (0 << LCD_E) | (low_nibble));
	_delay_ms(2);
}

// Initialize LCD
void lcd_init(void){
	_delay_ms(200); // Wait for LCD to power up
	lcd_command(0x33); // Initialize
	_delay_ms(50);
	lcd_command(0x32); // Set to 4-bit mode
	_delay_ms(50);
	lcd_command(0x28); // 2 line, 5x8 dots
	_delay_ms(5);
	lcd_command(0x0C); // Display on, cursor off
	_delay_ms(5);
	lcd_command(0x06); // Entry mode set
	_delay_ms(5);
	lcd_command(0x01); // Clear display
	_delay_ms(5);
}

// Print string to LCD
void lcd_print(const char *str){
	while(*str){
		lcd_data(*str++);
	}
}

int main(void){
	// Initialize TWI
	twi_init();
	
	// Configure PCA9555: Set REG_OUTPUT_0 as output (all 8 bits)
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00); // All P0 as outputs
	// Configure REG_OUTPUT_1 as needed (not used for LCD in this example)
	PCA9555_0_write(REG_CONFIGURATION_1, 0xFF); // All P1 as inputs
	
	// Initialize LCD
	lcd_init();
	
	// Print Name and Surname
	lcd_command(0x80); // Set cursor to first line
	lcd_print("STYLIANI");
	
	lcd_command(0xC0); // Set cursor to second line
	lcd_print("PAVLIDOU");
	
	while(1){
		// Άπειρος βρόχος
	}
	
	return 0;
}
