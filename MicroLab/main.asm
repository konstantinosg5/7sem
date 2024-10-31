;
; Omada3_1.asm
;
; Created: 23/10/2024 10:19:13 πμ
; Author : pavli
;

;
; lab3 - ex1.asm
;
; Author : KG
;

;ΣΗΜΕΙΩΣΕΙΣ:> Εχω χρησιμοποισει τους καταχωρητες ρ16-ρ20 και τους ρ27,28(Χ) και ρ30,31(Ζ)
;           > πιθανον να υπαρχει προβλημα με τους σπινθηρισμους
;           > εβαλα το Fast PWM, 8-bit ελπιζω αυτο να ζηταει

.include "m328PBdef.inc"
.equ MU=150
.org 0x0
rjmp reset

.def DC_VALUE=r16
.def offset=r17

;======================================== RESET / INITIALIZE
reset:
ldi r24, LOW(RAMEND)
out SPL, r24
ldi r24, HIGH(RAMEND)
out SPH, r24

ldi DC_VALUE,0x06; starting value 50% , οπου για να βρω το ποσοστο κανω 
                 ;  DC_VALUE * 8 +2 και αρα παιρει τιμες απο 0 - 12

SER r24
out DDRB,r24 ; Init PORTB as output

clr r24
out DDRD,r24  ; PIND as input            USE PIND FOR INPUT!!!!!!!!


;PWM set up
ldi r24, (1<<WGM10)| (1 << WGM11) | (1<<COM1A1) ; COM1A1 Συνδεση με pb1
sts TCCR1A, r24                   ; CS 100 CLK/256 = 62500 Hz
ldi r24, (1<<WGM12) | (1<<CS12)   ; WGM 0101 = Fast PWM, 8-bit 
sts TCCR1B, r24 
 

; registers X hold the start of the array with the precalc values for OCR1A
ldi XL, low(2*array)          ; Z register low byte (index)
ldi XH, high(2*array)         ; Z register high byte (address base)


;============================================ MAIN 
main:
sbis PIND,3
inc DC_VALUE

sbis PIND,4
dec DC_VALUE

rcall wait_x_msec 

movw Z , X
; Add the offset to ZL 
mov offset, DC_VALUE
lsl offset ; offset * 2
add ZL, offset   
brcc no_carry     ; Branch if no carry (addition didn't overflow)

; If there was a carry, increment ZH 
inc ZH

no_carry:
lpm r18,Z+  ;low byte
lpm r19,Z   ;high byte

sts OCR1AL, R18        ; βαζω στον ocr1a το DC σε 16bit μορφη
sts OCR1AH, R19

cpi DC_VALUE, 0x0C    ;ελεγξε οτι το DC εχει φτασει στο MAX  (12dec)         
breq maxed

cpi DC_VALUE, 0x00    ; ελεγξε οτι το DC εχει φτασει στο MΙΝ
breq minim

rjmp main


;=============================
; Aν ειμαστε στο μεγιστο ή στο ελαχιστο
maxed:
sbis PIND,4
rjmp endmax

rjmp maxed

endmax:
dec DC_VALUE
rjmp main

;====
minim:
sbis PIND,3
rjmp endmin

rjmp minim

endmin:
inc DC_VALUE
rjmp main

wait_x_msec:
push r23

in r23, SREG
push r23 ; save 23, SREG to stack
push r22

 ldi r24, low(MU)
 ldi r25, high(MU)
;==============

delay1:
ldi r22, 0x10 ;r22=16

delay3:
ldi r23, 235 ; (1 cycle)

;== ~1ms delay
delay2:
dec r23 ; 1 cycle
nop ; 1 cycle
brne delay2 ; 1 or 2 cycles
;==

dec r22
brne delay3

sbiw r24, 1 ; 2 cycles
brne delay1 ; 1 or 2 cycles


;=============


; Retrieve r23, SREG from stack
pop r22
pop r23
out SREG, r23
pop r23 

ret ; Return from call


;=============================================
; Define the matrix with 16-bit values
array:
;       2%     10%     18%    26%      34%     42%      50%      58%     66%     74%     82%     90%     98%
; .dw	20,   102,    184,   266,      348,    430,    512,      594,    676,    758,    840,    922,   1004  
.dw   0x0014, 0x0066, 0x00b8, 0x010a, 0x015c, 0x01ae,  0x0200, 0x0252,  0x02a4, 0x02f6, 0x0348, 0x039a, 0x03ec

 ;  2%     10%     18%    26%      34%     42%      50%       58%     66%     74%     82%     90%     98%
; .dw 0x051F, 0x022A, 0x2E14, 0x428F, 0x570A, 0x6B85, 0x8000, 0x947A,	0xA8F6, 0xBD71, 0xD1EB, 0xE666, 0xFAE1
	  
		
