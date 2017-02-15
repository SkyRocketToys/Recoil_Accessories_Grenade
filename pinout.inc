; *****************************************************************************
; Recoil Gun Grenade code for SkyRocket Toys 2017
; 
; TR4P153CT = new hardware (emulator, otp chips)
;     as TR4P153BT but supports ADC
; TR4P153BT = new hardware (grenade)
;     14 pins (11 input/output, 1 input, vcc, gnd)
;     2048 instructions
;     256 nybbles of RAM
;     2.2V to 5.5V
; *****************************************************************************
; PINOUT for Develop board 
; 1 PD1  PWR_EN        14 PD2  LVL_3
; 2 PD0  PWR_BTN (low) 13 PD3  LVL_2
; 3 PB2  EN_A2         12 PB3  LVL_1
; 4 VDD                11 VSS 
; 5 PB1  EN_A1         10 PA0  (unused)
; 6 PB0  EN_A0          9 PA1  MOD_OUT
; 7 PA3  (input only)   8 PA2  LED (visible) (active low)
; 
; *****************************************************************************
; PINOUT for Stephens board (*=common to both chips)
; This involved switching PA3 and PB2 from the schematic 27jan2017 since PA3 is input only
; 1 PD1  PWR_EN        14 PD2  EN_LED4
; 2 PD0  PWR_BTN (low) 13 PD3  EN_LED3
; 3 PB2  LVL_1         12 PB3  EN_LED2
; 4 VDD*               11 VSS*
; 5 PB1* LVL_3         10 PA0* EN_LED1
; 6 PB0* LVL_2          9 PA1* MOD_OUT
; 7 PA3* (input only)   8 PA2* LED (visible) (active low)
; 
; *****************************************************************************
; PINOUT for Hover Racer beacon board
; TR4P151AF = old hardware (beacon)
;     6 pins (11 input/output, 1 input, vcc, gnd)
;     2048 instructions
;     256 nybbles of RAM
;     2.2V to 5.5V
; 
; PINOUT
; 1 VDD    8 VSS
; 2 PB1    7 PA0
; 3 PB0    6 PA1
; 4 PA3    5 PA2
; *****************************************************************************
; GPIO usage for Stephens board
;   PA0 = Enable first pair of LEDs
;   PA1 = 38kHz infrared (output) (high = LED on)
;   PA2 = Visible LED
;   PA3 = unused (input only)
;   PB0 = Power level 2 on infrared LEDs (active high)
;   PB1 = Power level 3 on infrared LEDs (active high)
;   PB2 = Power level 1 on infrared LEDs (active high)
;   PB3 = Enable second pair of LEDs
;   PD0 = Power detect (input)
;   PD1 = Power on (output)
;   PD2 = Enable fourth pair of LEDs
;   PD3 = Enable third pair of LEDs
; 
; *****************************************************************************
; GPIO usage for Simons development board (*=same as stephens board)
;   PA0 =         unused
;   PA1*= MOD_OUT 38kHz infrared (output) (high = LED on)
;   PA2*= LED     Visible LED
;   PA3*=         unused (input only)
;   PB0 = EN_A0   Enable address 0
;   PB1 = EN_A1   Enable address 1
;   PB2 = EN_A2   Enable address 2
;   PB3 = LVL_1   Power level 1 on infrared LEDs (active low)
;   PD0*= PWR_BTN Power detect (input)
;   PD1*= PWR_EN  Power on (output)
;   PD2 = LVL_3   Power level 3 on infrared LEDs (active low)
;   PD3 = LVL_2   Power level 2 on infrared LEDs (active low)
; 
; *****************************************************************************

#if SPECIAL_SIMON
OUTPUT_POWER	equ	0
#else
OUTPUT_POWER	equ	3
#endif

; Soft power button
PORT_PWR_EN	equ	DATA_PD
PIN_PWR_EN	equ	1
PORT_PWR_BTN	equ	DATA_PD
PIN_PWR_BTN	equ	0

; Visible LED
PORT_LED	equ	DATA_PA
PIN_LED		equ	2

#ifdef BOARD_STEPHEN
; Infrared power
PORT_LVL_1	equ	DATA_PB
PIN_LVL_1	equ	2	; Active high
PORT_LVL_2	equ	DATA_PB
PIN_LVL_2	equ	0	; Active high
PORT_LVL_3	equ	DATA_PB
PIN_LVL_3	equ	1	; Active high

; LED enables (4 to 4)
PORT_EN1	equ	DATA_PA
PIN_EN1		equ	0 ;   PA0 = Enable first pair of LEDs
PORT_EN2	equ	DATA_PB
PIN_EN2		equ	3 ;   PB3 = Enable second pair of LEDs
PORT_EN3	equ	DATA_PD
PIN_EN3		equ	3 ;   PD3 = Enable third pair of LEDs
PORT_EN4	equ	DATA_PD
PIN_EN4		equ	2 ;   PD2 = Enable fourth pair of LEDs
#endif

#ifdef BOARD_DEVELOP
; Infrared power
PORT_LVL_1	equ	DATA_PB
PIN_LVL_1	equ	3	; Active low
PORT_LVL_2	equ	DATA_PD
PIN_LVL_2	equ	3	; Active low
PORT_LVL_3	equ	DATA_PD
PIN_LVL_3	equ	2	; Active low

; LED enables (3 to 8)
PORT_ENBITS	equ	DATA_PB
MASK_ENBITS	equ	7	; PB0, PB1, PB2 = address 0..7 of LEDs
#endif

; ----------------------------------------------------------------------------
