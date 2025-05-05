// PROJECT  :ADC, USART, and Double Dabble algorithm
// PURPOSE  :Serial.println(analogRead(PIN)) in assembly
// COURSE   :ICS4U-E
// AUTHOR   :R. Jamal
// DATE     :Created 2025 04 30
// MCU      :328P
// STATUS   :Working
// REFERENCE:http://darcy.rsgc.on.ca/ACES/TEI4M/Assembly/images/InterruptVectorTable.png
#include <avr/io.h>             //required to reference ports by 
#include  "Timer1prescalers.h"   //local Timer defines
#include  "ADCprescalers.h"     //local ADC defines
.global main;           bypass Arduino C requirement of setup & loop
.global TIMER1_OVF_vect; sufficient for handling this interrupt
.global ADC_vect;       sufficient for handling this interrupt
.equ LOW_MASK, 0x0F;
.equ HIGH_MASK, 0xF0;
.equ THRESH_LOW, 5;
.equ THRESH_HIGH, (5 << 4);
.equ BYTES, 16;
.equ BAUD_PRESCALER, 103    ; precomputed for 16MHz OSC & 9600 BAUD
.equ ASYNCHRONOUS, 0 << UMSEL01 | 0 << UMSEL00 ;
.equ SYNCHRONOUS,  0 << UMSEL01 | 1 << UMSEL00 ;
.equ PARITY_NONE,  0 << UPM01   | 0 << UPM00 ;
.equ PARITY_EVEN,  1 << UPM01   | 0 << UPM00 ;
.equ PARITY_ODD,   1 << UPM01   | 1 << UPM00 ;
.equ CHAR_SIZE8,   0 << UCSZ02  | 1 << UCSZ01 | 1 << UCSZ00 ; 8 - bit
.equ STOP_BITS1,   0 << USBS0 ;
.equ STOP_BITS2,   1 << USBS0 ;
count = 25            ; Register for iterations
util = 16             ; Utility register (low nibble)
util2 = 17            ; Utility register (high nibble)
bin0 = 18             ; Low byte for ADC reading
bin1 = 19             ; High byte for ADC reading
BCD01 = 20            ; BCD scratch space 0 / 1
BCD23 = 21            ; BCD scratch space 2 / 3
BCD4 = 22             ; BCD scratch space 4
offset  = 23          ; Register for ASCII offset
main :                ; Label (Main)
rcall TIMER1Setup     ; Timer 1 Mode 0 with preload
rcall ADCSetup        ; Configure ADC peripheral
rcall init_USART      ; Configure USART peripheral
sei                   ; Enable global interrupts
hold:
rjmp    hold          ; Wait for interrupts
ret                   ; End of main function
; PreCondition: Binary number is in bin0 and bin1
; PostCondition: BCD conversion of bin0 and bin1 are in BCD01-4
doubDabb:
ldi count, BYTES      ; Load # of iterations
clr BCD01             ; Clear BCD scratch space from last iteration
clr BCD23             ; Clear BCD scratch space from last iteration
clr BCD4              ; Clear BCD scratch space from last iteration
next:
lsl bin0              ; Shift 1st space left
rol bin1              ; Shift 2nd byte left (grab low byte from carry)
rol BCD01             ; Shift 3rd byte left (grab low byte from carry)
rol BCD23             ; Shift 4th byte left (grab low byte from carry)
rol BCD4              ; Shift 5th byte left (grab low byte from carry)
cpi count, 1          ; Check for last iteration
breq last             ; End the program and skip add three on last iteration
mov util, BCD01       ; Copy BCD01 to the util register
mov util2, BCD01      ; Copy BCD01 to the util2 register
rcall doLow           ; Add 3 to low nibble (if above 5);
rcall doHigh          ; Add 3 to high nibble (if above 5), combine nibbles;
mov BCD01, util       ; Update BCD01
mov util, BCD23       ; Copy BCD23 to the util register
mov util2, BCD23      ; Copy BCD23 to the util2 register
rcall doLow           ; Add 3 to low nibble (if above 5);
rcall doHigh          ; Add 3 to high nibble (if above 5), combine nibbles;
mov BCD23, util       ; Move back to BCD23
mov util, BCD4        ; Copy BCD4 to the util register
rcall doLow           ; Add 3 to low nibble (if above 5);
mov BCD4, util        ; Update BCD4
dec count             ; Decrement the iteration count
rjmp next             ; Go to next iteration
last:                 ; Label for end of program
ret                   ; End of function
doLow:                ; Function (adds 3 to high nibble when >= 5)
andi util, LOW_MASK   ; Clear high bits
cpi util, THRESH_LOW  ; Compare with 5
brlo skipLow          ; Skip next line if lower than 5
subi util, -3         ; Add 3 (when >= to 5)
skipLow:              ; Label
ret;                  ; End function
doHigh:               ; Function (adds 3 to high when >= 5, combines nibbles)
andi util2, HIGH_MASK ; Clear low bits
cpi util2, THRESH_HIGH; Compare with (5 << 4)
brlo skipHigh         ; Skip next line when less than (5 << 4)
subi util2, -(3 << 4) ; Add 3 (when >= to 5)
skipHigh:             ; Label
or util, util2        ; Combine high and low nibbles (from doLow)
ret                   ; End function
; PreCondition: OR flags for ADC initialization are defined
; PostCondition: ADC is ready For service
ADCSetup:
ser   util              ; r16 < - 0xFF
sts   DIDR0, util       ; Disable digital pins on PORTC when using ADC
ldi   util, ADCAVCC     ; AVCC as voltage reference (0.1uF on AREF)
ori   util, ADC5        ; Choose (arbitrarily) analog pin A5
sts   ADMUX, util       ; Do it
ldi   util, 1 << ADEN   ; Enable the ADC utility
ori   util, 1 << ADSC   ; Start a first dummy conversion
ori   util, ADCps128    ; Recommended prescaler to get to 125kHz
sts   ADCSRA, util      ; Do it
dummy:
lds   util, ADCSRA      ; Wait for it to complete
sbrs  util, ADIF        ; Is the first 25 - cycle conversion complete ?
rjmp  dummy             ; Keep waiting....
lds   util, ADCSRA      ; Prepare to clear the ADIF flag
ori   util, 1 << ADIF   ; ADIF flag is cleared by writing a 1 to it
ori   util, 1 << ADATE  ; Enable external trigger to start a conversion
sts   ADCSRA, util      ; Apply the OR flags
ldi   util, ADCtsT1OVF  ; select Timer1 Overflow as Trigger Source
sts   ADCSRB, util      ; Do it
lds   util, ADCSRA      ; Need to enable ADC Complete Interrupt...
ori   util, 1 << ADIE   ; Set the interrupt Enable bit...
sts   ADCSRA, util      ; Do it
ret                     ; End of function
; PreCondition: OR flags for Timer1 initialization are defined
; PostCondition: Timer1 is ready For service
TIMER1Setup:
clr util                ; Prepare for Normal Mode
sts TCCR1A, util        ; Set Normal Mode, now configure the prescaler...
ldi util, T1ps64        ; T1: 2 ^ 24 / 2 ^ 8 / 2 ^ 16  (prescale) > 1 ovf / s > 0.5Hz
sts TCCR1B, util        ; Timer1 clock is now system clock / prescale
ldi util, 0x00          ; Load TCNT1
sts TCNT1H, util        ; Put 0 into TCNT1H
sts TCNT1L, util        ; At 60Hz there appears to be some flicker
ldi util, 1 << TOIE1    ; Enable Timer / Counter1 Overflow Interrupt
sts TIMSK1, util        ; Enable Timer1 Overflow interrupts
ret                     ; Exit function
TIMER1_OVF_vect:        ; Destination of timer1 overflow
ldi util, 0x00          ; Put 0 into util
sts TCNT1H, util;       ; Clear timer1
sts TCNT1L, util        ; Clear timer1
reti                    ; Exit Interrupt
ADC_vect:               ; ISR handler of ADC Complete
lds bin0, ADCL          ; Grab low byte of ADC readout
lds bin1, ADCH          ; Grab high byte of ADC readout
rcall doubDabb;         ; Convert to BCD
rcall display           ; Send on USART
reti                    ; Exit interrupt
; PreCondition: OR flags for UART initialization are defined
; PostCondition: UART is ready for service
init_USART :            ; Label for function
ldi   util, hi8(BAUD_PRESCALER)   ; set the baud rate
sts   UBRR0H, util                ; Same
ldi   util, lo8(BAUD_PRESCALER)   ; Same
sts   UBRR0L, util                ; Same
ldi   util, (1 << RXEN0) | (1 << TXEN0)   ; Enable receive and transmit
sts   UCSR0B, util                        ; Same
ldi   util, ASYNCHRONOUS | CHAR_SIZE8 | PARITY_NONE | STOP_BITS1
sts   UCSR0C, util                        ; Set Comm. Mode  Default: 8N1
ret                                       ;
; PreCondition: the 3 DD working BCD registers are loaded with the conversion
; PostCondition: the 5 BCD values are transmitted through the UART
display:
ldi   offset, '0'         ; BCD values need an ASCII '0' (48) offset
cpi BCD4, 0               ; Check if BCD4 is 0
breq skips                ; Skip when leading 0
mov   r24, BCD4           ; Prepare the most significant BCD digit
andi  r24, 0x0F           ; Mask off the high nibble
add   r24, offset         ; Upgrade to ASCII character value
rcall TXCHAR              ; Echo (transmit) it
skips:                    ; Skip Case (see above)
mov   r24, BCD23          ; Prepare the BCD 1000s digit
swap  r24                 ; Exchange the high and low nibbles
andi  r24, 0x0F           ; Mask off the high nibble
add   r24, offset         ; Upgrade to ASCII character value
rcall TXCHAR              ; Echo (transmit) it
mov   r24, BCD23          ; Prepare the BCD 100s digit
andi  r24, 0x0F           ; Mask off the high nibble
add   r24, offset         ; Upgrade to ASCII character value
rcall TXCHAR              ; Echo (transmit) it
mov   r24, BCD01          ; Prepare the BCD 10s digit
swap  r24                 ; Exchange the high and low nibbles
andi  r24, 0x0F           ; Mask off the high nibble
add   r24, offset         ; Upgrade to ASCII character value
rcall TXCHAR              ; Echo (transmit) it
mov   r24, BCD01          ; Prepare the BCD 1s (units) digit
andi  r24, 0x0F           ; Mask off the high nibble
add   r24, offset         ; Upgrade to ASCII character value
rcall TXCHAR              ; Echo (transmit) it
ldi r24, '\r'             ; Load Return character
rcall TXCHAR              ; Send it
ldi r24, '\n'             ; Load next carriage
rcall TXCHAR              ; Send it
ret                       ; Exit function
; PreCondition:  Character data (1 byte) to be transmitted is in R24
; PostCondition: Contents of R24 are transmitted via the USART (should appear on the Serial Monitor)
TXCHAR:
lds   util, UCSR0A        ; Determine if the transmit buffer is empty
sbrs  util, UDRE0         ; If the flag is set, exit the loop
rjmp  . - 8               ; Loop back to the previous statement
sts   UDR0, r24           ; Transmit data in r24
ret                       ; Exit function
