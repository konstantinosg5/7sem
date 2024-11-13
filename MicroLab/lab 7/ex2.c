/*
 * main.c
 * Author: KG
 *  
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define PCA9555_0_ADDRESS 0x40
#define TWI_READ 1
#define TWI_WRITE 0
#define SCL_CLOCK 100000L
//A0=A1=A2=0 by hardware
// reading from twi device
// writing to twi device
// twi clock in Hz
//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2
// PCA9555 REGISTERS
typedef enum {
	REG_INPUT_0 =0,
	REG_INPUT_1 =1,
	REG_OUTPUT_0 =2,
	REG_OUTPUT_1 =3,
	REG_POLARITY_INV_0 =4,
	REG_POLARITY_INV_1 =5,
	REG_CONFIGURATION_0 =6,
	REG_CONFIGURATION_1 =7
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

//========================================================  Functions for TWI protocol
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

	uint8_t twi_status; // send START condition
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

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));

	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) ) return 1;
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
		if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) ){
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
//        1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
	return twi_start( address );
}

// Terminates the data transfer and releases the twi bus
void twi_stop(void){

	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}
//================================================================= functions for TWI /\
//==================================================================================
//================================================================= functions for PCA9555 \/
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value){

	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();

}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg){
	uint8_t val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	val = twi_readNak();
	twi_stop();
	return val;
}

//  ====================================================== Functions for the PCA9555 /\
//  ====================================================================
//  ====================================================== Functions for LCD Screen \/
void write_2_nibbles(uint8_t data){

	uint8_t temp=data;
	PCA9555_0_write(REG_OUTPUT_0, (PCA9555_0_read(REG_INPUT_0) & 0x0f) + (temp & 0xf0)); // transfer high byte in pca9555_0 reg output
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) | 0x08); // Enable Pulse
	// small delay
	asm("nop");
	asm("nop");
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) & (~0x08)); // Disable Pulse
	
	data=(data<<4)|(data>>4);
	PCA9555_0_write(REG_OUTPUT_0, (PCA9555_0_read(REG_INPUT_0) & 0x0f) + (data & 0xf0)); // transfer low byte in pca9555_0 reg output
	
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) | 0x08); // Enable Pulse
	// small delay
	asm("nop");
	asm("nop");
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) & (~0x08)); // Disable Pulse
	
}



void lcd_command(uint8_t cmd){
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) & (~0x04)); //LCD_RS=0 (Instruction)
	write_2_nibbles(cmd);
	_delay_us(250);
}

void lcd_data(uint8_t  data){
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) | 0x04); //LCD_RS=1 (Data)
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_clear_display(){
	lcd_command(0x01);  // command for clearing the screen
	_delay_ms(5);
}

void lcd_init(){
	_delay_ms(200);
	int count=0;
	while(count<3){      //command to switch to 8 bit mode
		PCA9555_0_write(REG_OUTPUT_0, 0x030);
		PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) | 0x08); // Enable Pulse
		// small delay
		asm("nop");
		asm("nop");
		PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) & (~0x08));
		_delay_us(250);
		++count;
	}
	
	PCA9555_0_write(REG_OUTPUT_0, 0x20); //command to switch to 4 bit mode
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) | 0x08); // Enable Pulse
	_delay_us(2);
	PCA9555_0_write(REG_OUTPUT_0, PCA9555_0_read(REG_INPUT_0) & (~0x08));

	_delay_us(250);
	
	lcd_command(0x28); //5*8 dots, 2 lines
	lcd_command(0x0c); //display on, cursor off
	
	lcd_clear_display();
	
}

void lcd_print(char *str){
	int i;
	for(i=0; str[i]!=0; i++) lcd_data(str[i]);
}


//=========================================================================
//========== ABOVE IS GIVEN CODE FOR THE LCD, TWI AND THE PCA9555 ==============
//=========================================================================

uint8_t one_wire_reset(){
	DDRD |= 0x10;   // set PD4 as output
	
	PORTD &= 0xEF; // 480us reset pulse
	_delay_us(480);
	
	DDRD &= 0xEF; // set PD4 as input
	PORTD &= 0xEF; // disable pull up
	
	_delay_us(100); // wait for someone to connect
	
	 //if device connects PD4 becomes 0 
	if (PIND & 0x10 == 0x10) return 0; // no device 
	else return 1;  // device connected
}


uint8_t one_wire_receive_bit(){
	DDRD |= 0x10; // set PD4 as output
	PORTD &= 0xEF;
	_delay_us(2);
	
	DDRD &= 0xEF; // set PD4 as input
	PORTD &= 0xEF;  // disable pull up
	_delay_us(10);
	
	uint8_t output = (PIND & 0x10)>>4;;
	_delay_us(49); // delay to meet the standard
	
	return output;	
}

void one_wire_transmit_bit(uint8_t value){
	DDRD |= 0x10; // set PD4 as output
	PORTD &= 0xEF;
	_delay_us(2);
	
	value &= 0x01;
	
	if (value == 0x01) PORTD |= 0x10;  // used 'if' so it changes only PD4 
	else PORTD &= 0xEF;
	
	_delay_us(58);
	
	DDRD &= 0xEF; // set PD4 as input
	PORTD &= 0xEF;  // disable pull up
	_delay_us(1);
}


uint8_t one_wire_receive_byte(){
	uint8_t output;
	
	for (int i=0; i<8; i++){
		output |= (one_wire_receive_bit()<<i);
	}
	
	return output;
}


void one_wire_transmit_byte(uint8_t value){
	for (int i=0; i<8; i++){
		one_wire_transmit_byte(value>>i);
	}	
}

uint16_t thermometer_routine(){
	// 1. initialization and check for connected devices
	if(one_wire_reset()==0) return 0x8000; //if no device connected
	
	// 2. send command 0xCC
	one_wire_transmit_byte(0xCC);
	
	// 3.Send command 0x44 and read temp as a check?
	one_wire_transmit_byte(0x44);
	while (one_wire_receive_bit()==0) _delay_ms(250); //polling
	
	
	// 5. Reinitialization and check for connected devices
	if(one_wire_reset()==0) return 0x8000; //if no device connected
	
	// 6. send command 0xCC
	one_wire_transmit_byte(0xCC);
	
	// 7. Send command 0xBE for reading
	one_wire_transmit_byte(0xBE);
	
	// 8. Receive the 2 bytes
	uint16_t temp;
	temp = one_wire_receive_byte();  // low byte
	temp |= ( one_wire_receive_byte()<<8 );  // high byte
	
	return temp;	
}

void display_temp(uint16_t temp){
	lcd_clear_display();
	
	char display[]="TEMP:     °C";
	
	if (temp>0xF000){ // temp negative get 2 compliment
		temp=~temp;
		temp+=1;
		display[5]='-';
	}
	
	temp = temp>>1; // don't need decimal part 
	                // DS1820  set to 1
	                // DS18B20 set to 4

	// Isolate each digit
	uint8_t hunt = temp/100;
	temp = temp % 100;
	
	uint8_t tens = temp/10;
	temp = temp % 10;
	
	uint8_t ones = temp;

	// Make digits ASCII
	hunt='0'+hunt;
	tens='0'+tens;
	ones='0'+ones;
	
	if (hunt=='0'){
		 display[6]==tens;
		 display[7]==ones;
	}
	else {
		display[6]=hunt;
		display[7]=tens;
		display[8]=ones;
	}	
	
	lcd_print(display); // i load that only once and i will change only the numbers
}

int main(){
	twi_init();
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00); //Set EXT_PORT0 as output LEDS   INVERSE LOGIC	
	lcd_init();
	
	uint16_t temperature;
	
	
    while(1){
		temperature = thermometer_routine();
		
		if(temperature == 0x8000){ 
			lcd_clear_display();
			char display[] = "NO DEVICE";
			lcd_print(display);
		}else{
			display_temp(temperature);			
		}	
    }
}
