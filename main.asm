.include "../m328Pdef.inc"

; --- Definitions ---
.def mpr = r16                  ; multi-purpose register

.equ tim0crla = 0b00000010      ; cfg timer0 for CTC operation
.equ tim0crlb = 0b00000010
.equ tim0inten = 0b00000010     ; cfg timer0 for compare interrupts
.equ timer0max = 49
; when F_CPU == 16Mhz &&  timer0 prescale = F_CPU/8, this maximum 
; makes the timer0 interrupt be called 40,000 times per second,
; just what we want to loop through the wavetable.

.equ note_num = 0               ; a test for controlling which note is played

; --- SRAM map ---
.dseg

; the current 24-bit pointer a spot in the wavetable
INDEX_1:        .byte 1
INDEX_2:        .byte 1
INDEX_3:        .byte 1

; the delta that corresponds to the current note frequency.
DELTA_1:        .byte 1
DELTA_2:        .byte 1
DELTA_3:        .byte 1

LFO_POT_1:      .byte 1

LFO_INDEX_1:    .byte 1
LFO_INDEX_2:    .byte 1
LFO_INDEX_3:    .byte 1

LFO_DELTA_1:    .byte 1
LFO_DELTA_2:    .byte 1
LFO_DELTA_3:    .byte 1

CURRENT_NOTE:   .byte 1

; --- Code start ---
.cseg
; interrupt table
.org 0x0000 ; system reset
    rjmp INIT
    
.org 0x001C
    rjmp TIM0_COMPA

.org 0x0034
INIT:
    ldi mpr, high(RAMEND)       ; set up stack pointer
    out SPH, mpr
    ldi mpr, low(RAMEND)
    out SPL, mpr

    ldi mpr, tim0crla
    out TCCR0A, mpr
    ldi mpr, tim0crlb
    out TCCR0B, mpr
    ldi mpr, tim0inten          ; enable interrupts when timer0 matches OCR0A
    sts TIMSK0, mpr
    ldi mpr, timer0max          ; reset timer0 when it reaches timerMax
    out OCR0A, mpr

    ldi mpr, 0b00010000         ; set up USART to recieve MIDI commands
    sts UCSR0B, mpr
    lid mpr, 0b00000110
    sts UCSR0C, mpr
    ldi mpr, 0x00               ; baud rate at 9600
    sts UBRROH, mpr
    ldi mpr, 0x67
    sts UBRR0L, mpr

    ldi mpr, 0b11111111
    out DDRD, mpr               ; set up PORTD pins 2-7 for output
    ldi mpr, 0b11111111
    out DDRB, mpr               ; set up PORTB pins 0-2 for output

    ; r28:r29:r30 are the current index of the wavetable (24 bit number)
    ; r24:r25:r26 are the index delta of the desired frequency.
    ;   The frequency of the sine wave is determined by how quickly
    ;   the wavetable index is incremented, i.e. by the value of the
    ;   index delta.
    ;
    ; r2:r3:r4 are the current index of the LFO
    ; r5:r6:r7 are the current index delta of the LF0
    
    ldi ZH, high(sawtooth<<1)   ; set up Z register for output waveform
    ldi ZL, low(sawtootth<<1)
    ldi YH, high(sine<<1)       ; set up Y register for LFO waveform
    ldi YL, low(sine<<1)
    sts INDEX_3, ZL
    ldi r29, 0
    sts INDEX_2, r29
    ldi r28, 0
    sts INDEX_1, r28

    sts LFO_INDEX_3, YL
    ldi r16, 0
    sts LFO_INDEX_2, r16
    ldi r17, 0
    sts LFO_INDEX_1, r17
    ldi r18, 0x62               ; set up LFO index delta for desired LFO freq
    sts LFO_DELTA_1, r18
    ldi r19, 0x10
    sts LFO_DELTA_2, r19
    ldi r20, 0x00
    sts LFO_DELTA_3, r20

    ldi mpr, 0
    sts CURRENT_NOTE, mpr

    sei                         ; enable interrupts

; --- Main loop ---
; (1) scans petentiometer values
; (2) TODO: recieves MIDI commands
MAIN:
    .include "adc.asm"          ; potentiometer logic and reading
    .include "usart.asm"        ; MIDI logic and reading

    rjmp MAIN

; --- Interrupt routines ---

TIM0_COMPA:
    ; Sound generation interrupt
    ; register description:
    ;   r30:r29:r28 --> contain the current index of the audio waveform,
    ;                   from MSB to LSB.
    ;   r26:r25:r24 --> contain the index delta for the desire frequency,
    ;                   from MSB to LSB.
    ;   r4:r3:r2    --> contain the current index of the LFO waveform,
    ;                   from MSB to LSB.
    ;   r7:r6:r5    --> contain the idex delta for the desired LFO frequency,
    ;                   from MSB to LSB.
    ;   r1:r0       --> reserved for the hardware multiplier.
    ;   r16 (a_reg) --> reserved to hold the current val of the audio waveform.
    ;   r17 (lfo_reg) --> reserved to hold the current val of the LFO waveform.

    push mpr                    ; save the registers from the main loop.
    in mpr, SREG
    push mpr

    .include "usart.asm"

    ; check if a pad is pressed. If not, set r1 to 0 and skip to OUTPUT.
    lds mpr, CURRENT_NOTE
    cpi mpr, 0
    brne COMPUTE_WAVEFORM
    rjmp NO_NOTE

; the important stuff
; [1] increment the position in the wavetable for the actual audio waveform.
COMPUTE_WAVEFORM:
    lds r28, INDEX_1            ; load the current audio wave index.
    lds r29, INDEX_2
    lds ZL, INDEX_3
    lds r24, DELTA_1            ; load the current audio wave delta.
    lds r25, DELTA_2
    lds r26, DELTA_3
    add r28, r24                ; increment the index by the index delta.
    adc r29, r25
    adc ZL, r26
    sts INDEX_3, ZL             ; store the new index value for the audio.
    sts INDEX_2, r29
    sts INDEX_1, r28
    lpm mpr, Z                  ; load the value pointed to by Z into r16

; [2] the LFO waveform
COMPUTE_LFO:
    ; turn LFO off if LFO_POT_1 is less than 5. This prevents the LFO form
    ; stagnating on a random value with the wavetable.
    lds r18, LFO_POT_1
    cpi r18, 10
    brlo LFO_OFF
    ; otherwise convert the LFO_POT_1 value into a 16-bit integer scaled
    ; for use in the LFO index delta registers.
    ldi r19, 255
    mul r18, r19                ; gives 16-bit value in r1:r0
    sts LFO_DELTA_1, r0
    sts LFO_DELTA_2, r1

    lds r2, LFO_INDEX_1
    lds r3, LFO_INDEX_2
    lds YL, LFO_INDEX_3
    lds r5, LFO_DELTA_1
    lds r6, LFO_DELTA_2
    lds r7, LFO_DELTA_3
    add r2, r5
    adc r3, r6
    adc YL, r7
    sts LFO_INDEX_1, r2
    sts LFO_INDEX_2, r3
    sts LFO_INDEX_3, YL
    push ZH
    push ZL
    mov ZL, YL
    lpm r17, Z
    pop ZL
    pop ZH

; [3] Modulate the audio value according the the LFO,
; and possibly (later) a filter.
MODULATE:
    ; TODO: add branch instrcutions depending on what the LFO should modify.
    ; TODO: see why amplitude is reduced by half when the values are made signed.
    subi r16, 128
    subi r17, 128               ; make LFO val signed.
    muls r17, mpr               ; modulate the audio amplitude by the LFO val
    ldi r17, mpr
    add r1, r18
OUTPUT:
    mov mpr, r1
    mov r17, r1
    andi mpr, 0b11111100
    andi r17, 0b00000011
    lsl r17
    out PORTD, mpr              ; output the current wavetable val
    out PORTB, r17              ; toggle WR pin on DAC
    sbi PORTB, 0
    nop                         ; TODO: why is this here?
    cbi PORTB, 0

    ; end of interrupt code

    pop mpr
    out SREG, mpr
    pop mpr

    reti

NO_NOTE:
    ldi mpr, 0
    mov r1, mpr
    rjmp OUTPUT

; Subroutine to turn off the LFO
LFO_OFF:
    ldi r17, 255
    rjmp MODULATE

; TODO: copy wavetables and index deltas.
