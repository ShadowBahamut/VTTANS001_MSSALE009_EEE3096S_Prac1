/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:
	@ Initialize counter
    MOVS R2, #0


increment_loop:
    @ Check if SW3 (PA3) is pressed to freeze
    LDR R3, GPIOA_BASE   @ Load GPIOA base
    LDR R3, [R3, #0x10]  @ Read input data register (IDR)
    MOVS R4, #8          @ Mask for PA3 (0x8)
    ANDS R4, R3          @ Check PA3 state
    BEQ freeze           @ Freeze if SW3 is pressed

    @ Check if SW2 (PA2) is pressed
    MOVS R4, #4          @ Mask for PA2 (0x4)
    ANDS R4, R3          @ Check PA2 state
    BEQ hold_pattern     @ Hold pattern if SW2 is pressed

    @ Write current LED value
    STR R2, [R1, #0x14]  @ Update LEDs

    @ Check SW1 (PA1) for delay selection
    MOVS R4, #2          @ Mask for PA1 (0x2)
    ANDS R4, R3          @ Check PA1 state
    BNE long_delay       @ Use long delay if SW1 not pressed

    @ SW1 pressed, use short delay
    LDR R3, SHORT_DELAY_CNT
    B start_delay


long_delay:
    @ SW1 is not pressed, use long delay (0.7 seconds)
    LDR R3, LONG_DELAY_CNT

start_delay:
delay_loop:
    SUBS R3, #1         @ Decrement delay counter
    BNE delay_loop      @ Loop until counter is 0

    @ Check if SW0 (PA0) is pressed
    LDR R3, GPIOA_BASE   @ Load GPIOA base
    LDR R3, [R3, #0x10]  @ Read input data register (IDR)
    MOVS R4, #1          @ Mask for PA0 (0x1)
    ANDS R4, R3          @ Check PA0 state
    BNE next_increment   @ If PA0 is high (not pressed), skip increment

    @ if SW0 pressed, increment by 2
    ADDS R2, #2          @ Increment by 2
    B cont_looping       @ Continue looping

next_increment:
    @ if SW0 is not pressed, increment by 1
    ADDS R2, #1

cont_looping:
    @ If counter overflows, it will automatically wrap to 0
    B increment_loop

hold_pattern:
    @ SW2 is pressed, set LED pattern to 0xAA
    MOVS R2, #0xAA
    STR R2, [R1, #0x14]  @ Write to LEDs
    B SW2

SW2:
   @ Wait for SW2 release (PA2)
    LDR R3, GPIOA_BASE   @ Load GPIOA base
    LDR R3, [R3, #0x10]  @ Read input data register (IDR)
    MOVS R4, #4          @ Mask for PA2 (0x4)
    ANDS R4, R3          @ Check PA2 state
    BEQ SW2 @ Loop if still pressed

    @ SW2 released, resume counting from 0xAA
    B increment_loop

freeze:
    @ SW3 is pressed, freeze the pattern
    STR R2, [R1, #0x14]  @ Write current LED value

SW3:
    @ Wait for SW3 release (PA3)
    LDR R3, GPIOA_BASE   @ Load GPIOA base
    LDR R3, [R3, #0x10]  @ Read IDR
    MOVS R4, #8          @ Mask for PA3 (0x8)
    ANDS R4, R3          @ Check PA3 state
    BEQ SW3 @ Loop if still pressed

    @ SW3 released, resume counting
    B increment_loop


write_leds:
    STR R2, [R1, #0x14]
    B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 1866667  @ 0.7 seconds delay
SHORT_DELAY_CNT: 	.word 800000   @ 0.3 seconds delay
