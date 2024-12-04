/*
 * main.c
 * Author: KG
 *  
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h> // for sprintf 

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
//  =======================================================================================
//  ===================================================== Functions for keyboard \/
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
//  ====================================================== Functions for keyboard /\
//  ===================================================================================
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

void lcd_print(const char *str){
	int i;
	for(i=0; str[i]!=0; i++) lcd_data(str[i]);
}

//  ======================================================  Functions for LCD Screen /\
//  ====================================================================
//  ======================================================  Functions for OneWire/TempRoutine \/
uint8_t one_wire_reset(){
	DDRD |= 0x10;   // set PD4 as output
	
	PORTD &= 0xEF; // 480us reset pulse
	_delay_us(480);
	
	DDRD &= 0xEF; // set PD4 as input
	PORTD &= 0xEF; // disable pull up
	
	_delay_us(100); // wait for someone to connect
	uint8_t input = PIND;
	_delay_us(380);
	 //if device connects PD4 becomes 0 
	if ((input & 0x10 )== 0x10) return 0; // no device 
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
	uint8_t output=0;
	
	for (int i=0; i<8; i++){
		output |= (one_wire_receive_bit()<<i);
	}
	
    asm("nop");	
	return output;
}


void one_wire_transmit_byte(uint8_t value){
	for (int i=0; i<8; i++){
		one_wire_transmit_bit(value>>i);
	}
	asm("nop");	
}

uint16_t thermometer_routine(){
	// 1. initialization and check for connected devices
	uint8_t temp1=one_wire_reset();
	if(temp1 == 0) return 0x8000; //if no device connected
	
	// 2. send command 0xCC
	one_wire_transmit_byte(0xCC);
	
	// 3.Send command 0x44 and read temp as a check?
	one_wire_transmit_byte(0x44);
	while (one_wire_receive_bit()==0) ; //polling
	
	
	// 5. Reinitialization and check for connected devices
	if(one_wire_reset()==0) return 0x8000; //if no device connected
	
	// 6. send command 0xCC
	one_wire_transmit_byte(0xCC);
	
	// 7. Send command 0xBE for reading
	one_wire_transmit_byte(0xBE);
	
	// 8. Receive the 2 bytes
	uint16_t temp = 0;
	temp = one_wire_receive_byte();  // low byte
	temp |= ( one_wire_receive_byte()<<8 );  // high byte
	
	return temp;	
}


//  ================================================= Functions for OneWire/TempRoutine /\
//============================================================================================
//  ===============================================================   ADC Functions  \/
void adc_init() {
	// REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC0(pin PC0),
	// ADLAR=0 =>right adjust the ADC result      0b01000100
	ADMUX = (1 << REFS0) ;
	
	// Enable ADC prescaler = 128  
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}




//  ========================================================== ADC Functions  /\
//  =================================================================================
//  ======================================================     Functions for UART \/
/* Routine: usart_init
Description:This routine initializes the usart as shown below.
------- INITIALIZATIONS -------
Baud rate: 9600 (Fck= 8MH)
Asynchronous mode
Transmitter on
Reciever on
Communication parameters: 8 Data ,1 Stop, no Parity
--------------------------------
parameters: ubrr to control the BAUD.
return value: None.*/
void usart_init(unsigned int ubrr){
	UCSR0A=0;
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)ubrr;
	UCSR0C=(3 << UCSZ00);
	return;
}

/* Routine: usart_transmit
Description:This routine sends a byte of data using usart.
parameters:
data: the byte to be transmitted
return value: None. */
void usart_transmit(uint8_t data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=data;
}

/* Routine: usart_receive
Description:This routine receives a byte of data from usart.
parameters: None.
return value: the received byte */
uint8_t usart_receive(){
	while(!(UCSR0A&(1<<RXC0)));
	return UDR0;
}
//  ======================================================     Functions for UART /\


//===========================================================================================
//===== ABOVE IS GIVEN CODE FOR THE ADC, LCD, TWI, PCA9555, OneWire/Thermometer and UART ==========
//===========================================================================================
uint16_t adc_read_pressure() {
	ADCSRA |= (1 << ADSC);  // Start Conversion
	while (ADCSRA & (1 << ADSC));  // Polling
	return (ADC*20)/1024*10; // calculate pressure;
}

void lcd_display(uint16_t temperature, uint16_t pressure, const char* status ){
	lcd_clear_display();
	float temp,pres;
	
	temp = (temperature>>4); // Integer part of temp
	temp += (temperature & 0x000F)/16.0; // decimal part
	temp += 12;
	pres = pressure /10.0; // pressure was given multiplied by 10
	
	// creating json
	static char display[16];
	
	sprintf(display, "T: %.1foC P:%.1f", temp, pres);
	
	lcd_print(display);
	
	lcd_command(0xC0); //change line
	lcd_print(status);
	

}

const char* esp_read(){
	uint8_t c, i=0;
	char readBuffer[100];  // As big as the LCD
	*readBuffer=NULL;
	while((c = usart_receive())!= '\n'){// As long as its not change line
		readBuffer[i++]=c;		
	}
	
	return readBuffer;
}

void esp_write(const char *str){
	int i=0;
	
	while( str[i]!='\n' ){
		usart_transmit(str[i]);
		i++;
		asm("nop");
	} 
	usart_transmit('\n');
}

void loadingAnimation(uint16_t totalTime){ // it displays a loading animation to second line of lcd
	int step = totalTime/16;
	
	lcd_command(0xC0); // second line
	
	for(int i=0; i<16; i++){
		lcd_data('#');
		for(int j=0; j<step; j++){
			_delay_ms(1);
		}
	}
	
}

const char* status;
void CheckStatus(uint16_t temperature, uint16_t pressure, char button ){
	static int flagNurseCalled=0;
	temperature=temperature>>4;
	temperature+=12;
	if (button == '5') {
		flagNurseCalled=1;
		status = "NURSE CALL";
		return;
	}
	else if (button == '#') flagNurseCalled=0; 
	
	if(flagNurseCalled==0){ // If nurse isn't being called, you can change status
		if( pressure<40 || pressure>120) status = "CHECK PRESSURE";
		else if (temperature<34 || temperature>37) status = "CHECK TEMP";
		else status = "OK";
	}
	
}

void esp_transmit_payload(uint16_t temperature, uint16_t pressure, const char* status){
	float temp,pres;
	
	temp = (temperature>>4); // Integer part of temp
	temp += (temperature & 0x000F)/16.0; // decimal part
	
	pres = pressure /10.0; // pressure was given multiplied by 10
	
	// creating json
	static char json[200];
	
	sprintf(json,"ESP:payload:[{\"name\": \"temperature\",\"value\": \"%.1f\"},{\"name\": \"pressure\",\"value\": \"%.1f\"},{\"name\": \"team\",\"value\": \"5\"},{\"name\": \"status\",\"value\": \"%s\"}]\n",temp, pres, status);
	
	esp_write(json);
	
}


int main(){
	DDRD=0xFF;  // Set for the thermometer
	DDRC=0x00;  // input for pressure sensor POT0
	twi_init();
	adc_init(); // for POT0 to take pressure
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00); // Set EXT_PORT0 as output for LCD
	PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); // Set EXT_PORT1 for 4x4 keyboard
	lcd_init();
	usart_init(103); // UBRR =(fosc /16 * BAUD) -1
	
	uint8_t button;
	uint16_t temperature, pressure;
	//const char* buffer;  // read buffer
	//buffer=" ";
    while(1){
		loadingAnimation(500);
		lcd_clear_display();
		esp_write("ESP:restart");
		// ==========    STEP 1: Connect to wifi   ===================
		while(1){
			esp_write("ESP:connect\n");
						
			//buffer=esp_read();
			//esp_write(buffer);
			if (strcmp(esp_read(),"Success")){//	if it return "\"Success\""	
				lcd_clear_display();		
				lcd_print("1. SUCCESS");
				break;
				
			} else{
				lcd_clear_display();
				lcd_print("1. FAIL");
				loadingAnimation(20000);// if fail wait 20sec
			}					
		}
		
		loadingAnimation(1000); // wait 2 sec in between steps
		 
		 
		// ==========    STEP 2: Connect to URL   ===================
		while(1){
			esp_write("ESP:url:\"http://192.168.1.250:5000/data\"\n");
					
			//buffer=esp_read();
			if (strcmp(esp_read(),"Success")){//	if it return "\"Success\""
				lcd_clear_display();
				lcd_print("2. SUCCESS");
				break;
				
				} else{
				lcd_clear_display();
				lcd_print("2. FAIL");
				loadingAnimation(20000);// if fail wait 20sec
			}
		}
		
		loadingAnimation(1000); // wait 2 sec in between steps
		
		
		// ==========    STEP 3: PAYLOAD LCD   ===================
		temperature = thermometer_routine();
		
		if(temperature == 0x8000){ 
			lcd_clear_display();
			char display[] = "THERMOMETER PLZ!";
			lcd_print(display);
			temperature=-1; // Value to -1 to indicate malfunction
		}
		
		pressure = adc_read_pressure();
		button = keyboard_to_ascii();
		
		CheckStatus(temperature, (uint16_t)(pressure),button);
		
		lcd_display(temperature, pressure, status);
		
		_delay_ms(5000);
		
		
		// ==========    STEP 3: PAYLOAD ESP   ===================		
		while(1){
			esp_transmit_payload(temperature, pressure, status);
			
			if (strcmp(esp_read(),"Success")){//	if it return "\"Success\""
				lcd_clear_display();
				lcd_print("3. SUCCESS");
				break;
				
				} else{
				lcd_clear_display();
				lcd_print("3. FAIL");
				loadingAnimation(20000);// if fail wait 20sec
			}
		}
		
		loadingAnimation(1000);
		
		
		// ==========    STEP 4: SERVER RESPONSE   ===================
		esp_write("ESP:transmit\n");
		                  
						 
		//buffer=esp_read();
		
		lcd_clear_display();
		lcd_print("4. ");
		lcd_print(esp_read());		
		
		loadingAnimation(2000);
		
		
		
    }//main while
}//main
