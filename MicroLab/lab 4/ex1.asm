.include "m328PBdef.inc" 
.org 0x00
rjmp reset

.org 0x1A
rjmp TMR1_ISR

.org 0x2A ;ADC Conversion Complete Interrupt
rjmp ADC_ISR

; Define constants for TMR1
.equ PRESCALER = 256
.equ TIMER1_COMPARE_VALUE = 62499
.equ PD2=2
.equ PD3=3
; Register definitions
.def resultL = r26           ; Lower byte of the result (in mV)
.def resultH = r27           ; Higher byte of the result (in mV)

.def temp = r16
.def ADC_L = r21
.def ADC_H = r22

.def hundreds=r29
.def tens=r30
.def units=r31

reset:
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPH, r24 ; init stack pointer

ser r24
out DDRD, r24 ; set PORTD as output
clr r24
out DDRC, r24; set PINC as input

rcall lcd_init
ldi r24, low(100)
ldi r25, high(100) ; delay 100 mS
rcall wait_msec

rcall init_adc
rcall init_tmr1

sei


main:
nop
ldi r24, low(100)
ldi r25, high(100) ; delay 100 mS
rcall wait_msec 
jmp main



;=======================================================
calc_voltage:
; Mπορω να θυσιασω 1 bit  του adc, μιας και 10bit δινουν ακριβεια 1/1024 * 5 = 0,004
; που δεν μας χρειαζετε αφου στρογγυλοποιουμε στο δευτερο δεκαδικο και με μονο 9bit
; εχουμε ακριβεια 1/512 * 5 = 0,009. Συνεπως,
; Vin= ADC/512 * 500 = ADC/128 * 125 με 0<=ADC<=512 (125 * 512 = 64000 <65535 αρα το πολυ 16bit)

; Step 1: Load 125 into temp registers
ldi temp, low(125)     ; Load lower byte 

clc
ror ADC_H
ROR ADC_L
clc
; Step 2: Multiply ADC * 125 με 0<=ADC<=512 (125 * 512 =64000 <65535 αρα το πολυ 16bit)
mul ADC_L, temp          ; Multiply lower bytes, result in r0:r1
mov resultL, r0          ; Move low byte to resultL
mov resultH, r1          ; Move high byte to resultH

mul ADC_H, temp          ; Multiply ADC_H by tempL
add resultH, r0          ; Add to result high byte
clr r0                   ; Clear r0 after use

; Step 3: Divide the result by 128 (shift 7 to right)
ldi temp,7
div: 
clc                        ; clear carry
ror  resultH               ; Shift (with carry)
ror  resultL               ; Shift 
dec temp
brne div

clr hundreds
clr tens
clr units

; find hundrends digit
calcHundreds:
cpi resultL, 100
sbrs resultH,0
brlo calcTens
sbiw resultL, 63
sbiw resultL, 37
inc hundreds
rjmp calcHundreds

calcTens:       ; at this point the value is only in resultL
cpi resultL,10
brlo calcUnits
subi resultL,10
inc tens
rjmp calcTens

calcUnits:
mov units,resultL

ret                     ; Return with result in resultL:resultH 


;========================================================
init_adc:
;//REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0000 => select ADC1(pin PC1),
;// ADLAR=0 =>right adjust the ADC result
ldi temp, 0b01000001 ;
sts ADMUX, temp
;// ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
;// ADIE=1 => Enable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
ldi temp, 0b10001111
sts ADCSRA, temp
ret

;=======================================================
init_tmr1:
; Set up Timer1
; Set CTC mode (WGM12 = 1), Set prescaler to 256 (CS12 = 1) 
ldi r16, (1 << WGM12) | (1 << CS12) 
sts TCCR1B, r16

; Set the compare value
ldi r16, low(TIMER1_COMPARE_VALUE)
sts OCR1AL, r16       ; Set lower byte
ldi r16, high(TIMER1_COMPARE_VALUE)
sts OCR1AH, r16       ; Set upper byte


; Enable Timer1 Compare Match Interrupt
ldi r16, (1 << OCIE1A) ; so tha it triggers when it reaches the value stored in OCR1A
sts TIMSK1, r16        ; Enable Timer1 compare match interrupt

ret

;=======================================================
TMR1_ISR:
lds temp, ADCSRA    ;
ori temp, (1<<ADSC) ; Set ADSC flag of ADCSRA
sts ADCSRA, temp    ; it start ADC every 1sec  
reti


;=======================================================
ADC_ISR:
lds ADC_L,ADCL ; Read ADC result
lds ADC_H,ADCH ;

rcall lcd_clear_display
rcall calc_voltage

ldi r24,'V'
call lcd_data
ldi r24,'O'
call lcd_data
ldi r24,'L'
call lcd_data
ldi r24,'T'
call lcd_data
ldi r24,'A'
call lcd_data
ldi r24,'G'
call lcd_data
ldi r24,'E'
call lcd_data
ldi r24,':'
call lcd_data
ldi r24,' '
call lcd_data

ldi r24,'0'
add r24, hundreds
call lcd_data

ldi r24,','
call lcd_data

ldi r24,'0'
add r24, tens
call lcd_data

ldi r24,'0'
add r24, units
call lcd_data

ldi r24,'V'
call lcd_data

ldi r24, low(200)
ldi r25, high(200) ; delay 100 mS
rcall wait_msec 

reti

;======================================================

;======================================================================
;=====================  COPY απο διαφανειες  ============================
;======================================================================
;Αρχικοποίηση και ρυθμίσεις της οθόνης LCD όπως παρουσιάζεται παρακάτω:
;DL = 0 4 bit mode
;N = 1 2 lines
;F = 0 5×8 dots
;D = 1 display on
;C = 0 cursor off
;B = 0 blinking off
;I/D = 1 DD-RAM address auto increment
;SH = 0 shift of entire display off

lcd_init:
ldi r24 ,low(200) ;
ldi r25 ,high(200) ; Wait 200 mSec
rcall wait_msec ;
ldi r24 ,0x30 ; command to switch to 8 bit mode
out PORTD ,r24 ;
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec ;
ldi r24 ,0x30 ; command to switch to 8 bit mode
out PORTD ,r24 ;
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec ;
ldi r24 ,0x30 ; command to switch to 8 bit mode
out PORTD ,r24 ;
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec
ldi r24 ,0x20 ; command to switch to 4 bit mode
out PORTD ,r24
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec
ldi r24 ,0x28 ; 5x8 dots, 2 lines
rcall lcd_command
ldi r24 ,0x0c ; dislay on, cursor off
rcall lcd_command
rcall lcd_clear_display
ldi r24 ,0x06 ; Increase address, no display shift
rcall lcd_command ;
ret

;=======================================================
lcd_clear_display:
ldi r24 ,0x01 ; clear display command
rcall lcd_command
ldi r24 ,low(5) ;
ldi r25 ,high(5) ; Wait 5 mSec
rcall wait_msec ;
ret	


;=======================================================
lcd_command:
cbi PORTD ,PD2 ; LCD_RS=0(PD2=0), Instruction
rcall write_2_nibbles ; send Instruction
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec
ret

;=======================================================
lcd_data:
sbi PORTD ,PD2 ; LCD_RS=1(PD2=1), Data
rcall write_2_nibbles ; send data
ldi r24 ,250 ;
ldi r25 ,0 ; Wait 250uSec
rcall wait_usec
ret

;======================================================
write_2_nibbles:
push r24 ; save r24(LCD_Data)
in r25 ,PIND ; read PIND
andi r25 ,0x0f ;
andi r24 ,0xf0 ; r24[3:0] Holds previus PORTD[3:0]
add r24 ,r25 ; r24[7:4] <-- LCD_Data_High_Byte
out PORTD ,r24 ;
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
pop r24 ; Recover r24(LCD_Data)
swap r24 ;
andi r24 ,0xf0 ; r24[3:0] Holds previus PORTD[3:0]
add r24 ,r25 ; r24[7:4] <-- LCD_Data_Low_Byte
out PORTD ,r24
sbi PORTD ,PD3 ; Enable Pulse
nop
nop
cbi PORTD ,PD3
ret

;===============================================
wait_msec:
push r24 ; 2 cycles
push r25 ; 2 cycles
ldi r24 , low(999) ; 1 cycle
ldi r25 , high(999) ; 1 cycle
rcall wait_usec ; 998.375 usec
pop r25 ; 2 cycles
pop r24 ; 2 cycles
nop ; 1 cycle
nop ; 1 cycle
sbiw r24 , 1 ; 2 cycles
brne wait_msec ; 1 or 2 cycles
ret ; 4 cycles


wait_usec:
sbiw r24 ,1 ; 2 cycles (2/16 usec)
call delay_8cycles ; 4+8=12 cycles
brne wait_usec ; 1 or 2 cycles
ret

delay_8cycles:
nop
nop
nop
ret
