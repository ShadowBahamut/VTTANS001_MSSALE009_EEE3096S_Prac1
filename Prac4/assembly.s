/*
 * assembly.s - Button-Controlled LED Pattern
 * 
 * This program configures GPIO pins on a microcontroller to control LEDs 
 * based on user inputs from push buttons. It uses four buttons (SW0-SW3) 
 * to modify the LED output and control timing.
 */

    .syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ Vectors: Initial stack pointer and reset vector
vectors:
    .word 0x20002000               @ Initial stack pointer
    .word ASM_Main + 1             @ Reset vector points to ASM_Main

@ Main entry point
ASM_Main:

    @ --- Enable GPIO Clocks for GPIOA and GPIOB ---
    LDR R0, RCC_BASE               @ Load base address of RCC
    LDR R1, [R0, #0x14]            @ Read current AHBENR register
    LDR R2, AHBENR_GPIOAB          @ Prepare mask for GPIOA and GPIOB enable
    ORRS R1, R1, R2                @ Set bits 17 and 18 to enable GPIOA and GPIOB
    STR R1, [R0, #0x14]            @ Write updated value back to AHBENR

    @ --- Configure GPIOA pull-up resistors for buttons ---
    LDR R0, GPIOA_BASE             @ Load GPIOA base address
    MOVS R1, #0b01010101           @ Pull-up configuration for PA0 to PA3 (buttons)
    STR R1, [R0, #0x0C]            @ Write to GPIOA_PUPDR register

    @ --- Configure GPIOB for LED output ---
    LDR R1, GPIOB_BASE             @ Load GPIOB base address
    LDR R2, MODER_OUTPUT           @ Output mode configuration for GPIOB pins
    STR R2, [R1, #0]               @ Write to GPIOB_MODER register
    MOVS R2, #0                    @ R2 will hold the current value for the LEDs

main_loop:
    MOVS R2, #0                    @ Reset counter to 0 at the start of the loop

@ --- Increment Loop ---
increment_loop:
    LDR R3, GPIOA_BASE             @ Load GPIOA base address for button reading
    LDR R3, [R3, #0x10]            @ Read input data register (IDR)
    
    @ --- Check SW3 (PA3) for freeze ---
    MOVS R4, #8                    @ Mask for PA3 (SW3)
    ANDS R4, R3                    @ Isolate PA3 state
    BEQ freeze                     @ If SW3 is pressed, freeze pattern

    @ --- Check SW2 (PA2) for hold pattern ---
    MOVS R4, #4                    @ Mask for PA2 (SW2)
    ANDS R4, R3                    @ Isolate PA2 state
    BEQ hold_pattern               @ If SW2 is pressed, hold pattern

    @ --- Update LEDs ---
    STR R2, [R1, #0x14]            @ Write current value of R2 to GPIOB (LEDs)

    @ --- Check SW1 (PA1) for delay selection ---
    MOVS R4, #2                    @ Mask for PA1 (SW1)
    ANDS R4, R3                    @ Isolate PA1 state
    BNE long_delay                 @ If SW1 not pressed, use long delay

    @ --- Short delay ---
    LDR R3, SHORT_DELAY_CNT        @ Load short delay count
    B start_delay

long_delay:
    @ --- Long delay ---
    LDR R3, LONG_DELAY_CNT         @ Load long delay count

start_delay:
delay_loop:
    SUBS R3, #1                    @ Decrement delay counter
    BNE delay_loop                 @ Loop until delay counter reaches zero

    @ --- Check SW0 (PA0) for increment type ---
    LDR R3, GPIOA_BASE             @ Reload GPIOA base for button reading
    LDR R3, [R3, #0x10]            @ Read input data register (IDR)
    MOVS R4, #1                    @ Mask for PA0 (SW0)
    ANDS R4, R3                    @ Isolate PA0 state
    BNE next_increment             @ If SW0 not pressed, increment by 1

    @ --- SW0 pressed, increment by 2 ---
    ADDS R2, #2                    @ Increment counter by 2
    B cont_looping                 @ Continue loop

next_increment:
    @ --- Increment by 1 ---
    ADDS R2, #1                    @ Increment counter by 1

cont_looping:
    @ --- Check for overflow ---
    B increment_loop               @ Return to the start of the increment loop

@ --- Hold LED Pattern (SW2) ---
hold_pattern:
    MOVS R2, #0xAA                 @ Set LED pattern to 0xAA (10101010)
    STR R2, [R1, #0x14]            @ Write to LEDs
    B SW2_release

SW2_release:
    @ --- Wait for SW2 release ---
    LDR R3, GPIOA_BASE             @ Reload GPIOA base for button reading
    LDR R3, [R3, #0x10]            @ Read input data register (IDR)
    MOVS R4, #4                    @ Mask for PA2 (SW2)
    ANDS R4, R3                    @ Check if SW2 is still pressed
    BEQ SW2_release                @ If still pressed, keep waiting

    @ --- SW2 released, resume increment ---
    B increment_loop

@ --- Freeze LED Pattern (SW3) ---
freeze:
    STR R2, [R1, #0x14]            @ Write current LED value to freeze
    B SW3_release

SW3_release:
    @ --- Wait for SW3 release ---
    LDR R3, GPIOA_BASE             @ Reload GPIOA base for button reading
    LDR R3, [R3, #0x10]            @ Read input data register (IDR)
    MOVS R4, #8                    @ Mask for PA3 (SW3)
    ANDS R4, R3                    @ Check if SW3 is still pressed
    BEQ SW3_release                @ If still pressed, keep waiting

    @ --- SW3 released, resume increment ---
    B increment_loop

@ Constants and Memory Mapped Registers
    .align
RCC_BASE:          .word 0x40021000  @ RCC base address
AHBENR_GPIOAB:     .word 0b1100000000000000000  @ Enable bits for GPIOA and GPIOB
GPIOA_BASE:        .word 0x48000000  @ GPIOA base address
GPIOB_BASE:        .word 0x48000400  @ GPIOB base address
MODER_OUTPUT:      .word 0x5555      @ Output mode config for GPIOB

@ Delays
LONG_DELAY_CNT:    .word 1866667     @ Long delay (approx 0.7 seconds)
SHORT_DELAY_CNT:   .word 800000      @ Short delay (approx 0.3 seconds)
