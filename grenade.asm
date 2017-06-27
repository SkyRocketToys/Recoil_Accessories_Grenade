; *****************************************************************************
; Recoil Gun Grenade code for SkyRocket Toys 2017
; Based on TR4K-IDE Demo Code by TRITAN Technology Inc.
; 
; Note that the TR4K-IDE uses 8 character tabs.
; TR4P153CT = new hardware (emulator, otp chips)
;     as TR4P153BT but supports ADC
; TR4P153BT = new hardware (grenade)
;     14 pins (11 input/output, 1 input, vcc, gnd)
;     1536 instructions (12 bit words)
;     256 nybbles of RAM (4 bit nybbles) in banks of 32 nybbles
;     2.2V to 5.5V
; For pinout and GPIO usage see pinout.inc
; For protocol timing see protocol.inc 
; *****************************************************************************
; 
; Resource usage
;   Green mode (low speed oscillator) is not used - just normal and halt
;   Internal high speed oscillator is 32MHz +/- 2%
;   CPU clock is osc/4 = 8MHz
;   Scaler2 = MCLK/8 = 1MHz
;   Timer1 = Unused
;   Timer2 = 100uS for protocol output
;   RTC = Unused (1 second)
; 
; *****************************************************************************
; Notes about assembly language
;   jc = jump if less than
;   jnc = jump if greater or equal
; *****************************************************************************

; -----------------------------------------------------------------------------
; Defines ONLY WORK if they are defined here rather than in the include files
; Must be BEFORE include blocks
;  "BOARD_MARCH" fails to compile so we use "BOARD_8LEDS"
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Define one of these boards.
#define BOARD_8LEDS ; PCB with 8 single infrared LEDs on individual pins
;#define BOARD_STEPHEN ; Initial prototype board from stephen with 4 pairs of infrared LEDs
;#define BOARD_DEVELOP ; PCB with 8 single infrared LEDs on a 3 to 8 demuxer

; -----------------------------------------------------------------------------
; Choose an appropriate protocol from protocol.inc, and its subdefines
; since the assembler does not get them correctly.
#define PROTOCOL_MAN20A
#define PROTOCOL_MAN
#define PROTOCOL_20A
#define PROTOCOL_OUTSTATE	; Output the outstate instead of the state

;#define PROTOCOL_NEC12
;#define PROTOCOL_NEC
;#define PROTOCOL_12

;#define PROTOCOL_NEC4
;#define PROTOCOL_NEC
;#define PROTOCOL_4
;#define PROTOCOL_EXPLODE_PULSE
; Note that the explode pulse happens during the explosion state

; -----------------------------------------------------------------------------
#define USE_FIXED_SERIAL 0 ; Use a fixed serial number instead of unique serial number

; -----------------------------------------------------------------------------
; Special simon demo for analysing power supply
; 1. ADDRESS_MASK is always 0 (use LED 1 only)
; 2. OUTPUT_POWER is always 0 (power levels off)
; 3. 38kHz constantly - no modulation (IR_OnOff is assumed to be 1)
; 4. long delay on reset (no 38khz output) 500ms+
; 5. do not go to sleep
;#define SPECIAL_SIMON 1

; -----------------------------------------------------------------------------
; Special sam demo for testing range
; Keeps looping trigger
;#define SPECIAL_SAM 1

; -----------------------------------------------------------------------------
; Include Block
; -----------------------------------------------------------------------------

; Include one of these three only
#include "tr4p153ct.inc" ; Emulator (same as grenade)
;#include "tr4p153bt.inc" ;Grenade (same as emulator)
;#include "tr4p151af.inc" ; Beacon (lacks portD)

#include "protocol.inc"
#include "pinout.inc"

; -----------------------------------------------------------------------------

; grenade logic equates (internal logic)
state_unarmed	equ 0
state_cancelled	equ 1
state_priming	equ 2
state_primed	equ 3
state_10	equ 4
state_9		equ 5
state_8		equ 6
state_7		equ 7
state_6		equ 8
state_5		equ 9
state_4		equ 10
state_3		equ 11
state_2		equ 12
state_1		equ 13
state_explode	equ 14
state_waiting	equ 15	; On start up, wait for a while to decide if priming or armed

; Output "state" as requested on 27 jun 2017 (and made sensible by including primed/cancelled)
outstate_none		equ	0	; Do not output packets
outstate_explode0	equ	1	; Old cancelled button - may be more visible
outstate_explode5	equ	2
outstate_explode10	equ	3
outstate_explode15	equ	4
outstate_explode20	equ	5
outstate_explode25	equ	6
outstate_explode30	equ	7
outstate_explode35	equ	8
outstate_explode40	equ	9
outstate_explode45	equ	10
outstate_explode50	equ	11
outstate_explode55	equ	12
outstate_cancelled	equ	13
outstate_primed		equ	15


TIME_TICK	equ 100*25/2 ; Size of normal grenade ticks in ms->interrupt units
TIME_EXPLODE	equ 100*25/2 ; Size of explosion ticks in ms->interrupt units (assumed to be <=65535)
TIME_CANCELLED	equ 100*25/2 ; Size of cancelled ticks in ms->interrupt units (assumed to be <=65535)
TIME_PRIMING	equ  50*25/2 ; Size of priming ticks
TIME_PRIMED	equ 500*25/2 ; Size of primed ticks (between sending out packets)
TIME_WAITING	equ 750*25/2 ; Time to wait to see if priming or armed
COUNT_PRIMING	equ 10 ; Number of priming ticks between packets (assumed to be even) (assumed to be <255)
;;COUNT_EXPLODE	equ 20 ; Number of explosion ticks before going to sleep (assumed to be <=255)
COUNT_EXPLODE	equ 50 ; Number of explosion ticks before changing explosion counter (assumed to be <=255)
COUNT_TICK	equ 10 ; Number of update ticks before going to next state (assumed to be even) (assumed to be <15)
COUNT_CANCELLED	equ 20 ; Number of cancelled ticks before going to sleep (assumed to be <=255)
COUNT_PRIMED	equ  5 ; How long are we solid in the primed mode? (2 seconds)

#ifdef SPECIAL_SIMON
ADDRESS_MASK	equ	0
#else
#ifdef BOARD_STEPHEN
ADDRESS_MASK	equ	3
#endif
#ifdef BOARD_DEVELOP
ADDRESS_MASK	equ	7
#endif
#ifdef BOARD_8LEDS
ADDRESS_MASK	equ	7
#endif
#endif

; -----------------------------------------------------------------------------
; User defined "registers" (SRAM variables)
; -----------------------------------------------------------------------------
;#define Optimal_MAH		; System MAH Optimal (Only optimal MAH)
#define Optimal_MAH_PCH		; System MAH + PCH Optimal (Optimal MAH and PCH)
; This means that ldpch commands are not required since they will be ignored and regenerated by the compiler
; So I only put them in the header so we can see how they affect the padding
; This means that ldmah commands are not required since they will be generated by the compiler when variables in this struct are used
; So I only put them in where they are needed to override the assembler

VARRM = {
; -------------------------------------
; System variables
A_SRT          ; Save the A register during an interrupt
SramChk0_IN    ; Canary byte for checking SRAM corruption
SramChk1_IN    ; ^

; The 16 bit payload data
; In hover racer this was the MCU unique ID read from OTP flash (set on startup)
Payload0        ; 16 bit packet data
Payload1        ; ^
Payload2        ; ^
Payload3        ; ^

; -------------------------------------
; Infrared packet (output)
; (up to) 32 bit data packet for output (set on each packet output; destroyed during transmission)
IR_PktData0    ; (was IR_CRC_Byte0) First bit sent is the low bit of this nybble
IR_PktData1    ; (was IR_CRC_Byte1)
IR_PktData2    ; (was IR_Data_Byte0)
IR_PktData3    ; (was IR_Data_Byte1)
IR_PktData4    ; (was IR_Data_Byte2)
IR_PktData5    ; (was IR_Data_Byte3)
IR_PktData6    ; (was IR_User_Byte0)
IR_PktData7    ; (was IR_User_Byte1) Last bit sent is the high bit of this nybble

; -------------------------------------
; CRC variables (first bank)
Dat_Len_Num    ; Number of bits that have been processed via CRC

IR_CRC_Buf0    ; Calculated CRC output buffer
IR_CRC_Buf1    ; ^

; -------------------------------------
; Packet timing variables
Tim_SendPkt1   ; 12 bit counter (incremented once per Timer2 interrupt)
Tim_SendPkt2   ; ^  Used for determining timing of sending packets
Tim_SendPkt3   ; ^  SHARED between main thread and interrupt thread! Be careful!

IR_Group0      ; 12 bit counter (incremented every packet) for checking fast vs slow packets
IR_Group1      ; ^  For main thread
IR_Group2      ; ^  For main thread

IR_Enable_Flag ; Zero = Do not touch infrared, One = allow infrared
IR_Flag        ; Bit0=header pulse has been sent.
	       ; Bit1=header gap has been sent
	       ; Bit2=payload has been sent
	       ; Bit3=stop bit has been sent
IR_Num0        ; 8 bits: the number of payload bits that have been sent in this packet
IR_Num1        ; ^ For interrupt thread

IR_BaseTim0    ; 8 bit counter incremented every timer2 (100uS) "Do basic time waveform used"
IR_BaseTim1    ; ^ this is reset between phases (e.g. header, gap, each payload bit)
IR_BIT_OK_FLAG ; True iff IR_Bit is valid. False if a new payload bit should be extracted from the packet.
IR_Bit         ; The value of the current transmitting payload bit (0 or 1)
; (would be a bank boundary if this was handled manually)
LastTim1       ; Used for checking that multi nybble comparisons have not been invalidated
g_poweroff     ; Flag to turn the power off (no sleep mode in this project)
g_quiet        ; In primed mode, this means we are quiet

; CRC variables (second bank)
CRC_DATA0      ; Calculated 16 bit CRC input buffer
CRC_DATA1      ; ^
CRC_DATA2      ; ^
CRC_DATA3      ; ^

; Grenade logic variables
g_update       ; true when we want to update the output packet (Payload0 has changed)
g_send         ; true when we want to keep on sending a group of packets
g_timer0       ; Increments every 100us
g_timer1       ; Increments every 1.6ms
g_timer2
g_timer3

g_state
g_substate0
g_substate1
g_outi		; Which port to use for output

g_tmp		; For masking
g_random	; Increments fast when the button is pressed

; The MCU unique ID read from OTP flash (set on startup)
Mcu_ID0         ; 16 bit packet data
Mcu_ID1         ; ^
Mcu_ID2         ; ^
Mcu_ID3         ; ^

; The processed payload (gggggg01) = mangled serial of chip + 11 for grenade
Weapon0		; Goes to Payload2
Weapon1         ; Goes to Payload3

; Pressing button
BtnTimer	; For spacing out the read (debouncing)
BtnLast		; Last button was pressed? (active high)
BtnNow		; Button is pressed? (active high)
BtnSmooth	; Button value 0 (off) ... 15 (on) for debouncing the other variables

; Fast explosion logic
SendExplode	; 0=normal, 1=Explosion enabled
ExplodePhase	; 0=off phase, 1=on phase during explosion

OutState	; outstate_xxx for output

; (would be a bank boundary if this was handled manually)

; Initial delay timer
CntDelay0
CntDelay1
CntDelay2
CntDelay3
CntDelay4

}

IR_FLAG_HDR_MARK  equ 1 ; Bit0=header pulse has been sent.
IR_FLAG_HDR_SPACE equ 2 ; Bit1=header gap has been sent
IR_FLAG_PAYLOAD   equ 4 ; Bit2=payload has been sent
IR_FLAG_STOP      equ 8 ; Bit3=stop bit has been sent
IR_FLAG_PACKET	  equ 15; All bits set

; This is cached to reduce interrupt jitter on the infrared output enable
IR_OnOff	equ USER1 ; set nonzero for on (copy to RTC register at next timer2 interrupt)

; -----------------------------------------------------------------------------
; General Parameters for the program
Ram_chk_Dat     equ 5AH   ; Canary value for SRAM sanity checking
VINT_MAH        equ 00    ; The SRAM bank we want to use inside the interrupt [Mind you, Optima overrides this]
DELAY_MAH       equ 03    ; The SRAM bank we want to use for delays

BIT_PWR_BTN	equ	1<<PIN_PWR_BTN

; -----------------------------------------------------------------------------
; PROGRAM Entry points
; -----------------------------------------------------------------------------

	; (0) Entry point for program start
	org	0
	ldpch	PGMSRT
	jmp	PGMSRT
	nop
	nop
	
	; (4) Entry point for wakeup
	ldpch	WakeUp
	jmp	WakeUp
	nop
	nop

	; (8) Entry point for interrupt start (Timer1, Timer2, RTC)
	ldpch	INT_Start
	jmp	INT_Start

; -----------------------------------------------------------------------------
; Main Interrupt routine for Timer1, Timer2, RTC 
; Hardware will store the following registers and restore them on RETI
;	MAH = Memory address high
;       PCH = Program counter high
;       PCL = Program counter low
;       CZ  = Carry and Zero flags of the status register
;       ENINT = ENINT flag of the SYS0 register
; Cannot call subroutines; must preserve other registers
; -----------------------------------------------------------------------------
INT_Start:
	ldmah	#VINT_MAH ; (Optima will do this anyway)
	ld	(A_SRT),A ; Preserve the A register over the interrupt

	; Despatch the interrupt
	; Is this interrupt from the real time clock? (once per second)
	ld	a,(RTC)
	and	A,#1000B;RTCFG/F38K/RTCS1/RTCS0
	jz	RTC_Acked
	; This interrupt is from the real time clock (not important)
	clr	#3,(RTC) ; Clear the real time clock overflow flag
RTC_Acked:

	; Is this interrupt from Timer1? (should not happen)
	ld	A,(STATUS);TM2IFG/TM1IFG/CF/ZF
	and	A,#0100B
	jz	Timer1_Acked
	clr	#2,(STATUS) ; Clear the timer1 interrupt flag
Timer1_Acked:
	
	; Is this interrupt from Timer2?
	ld	a,(STATUS)
	and	A,#1000B
	jnz	Tim2_Int_Prc
	; No timer2 interrupt this time round
	jmp	INT_End_NoInc

; This interrupt is from Timer2
Tim2_Int_Prc:
	clr	#3,(STATUS) ; Clear the timer2 interrupt flag

	; Update timer for grenade logic
	inc	(g_timer0)
	adr	(g_timer1)
	adr	(g_timer2)
	adr	(g_timer3)

	; Support special grenade explosion
	ld	a,(SendExplode)
	cmp	a,#1	; Just in case this is set to a bad value
	jz	IR_SendExplode

	; Are we sending IR data?
	ld	a,(IR_Enable_Flag)
	jz	INT_End

	; Where are we in the transmission?
	inc	(IR_BaseTim0) ; Increment 8 bit timer
	adr	(IR_BaseTim1)
	
	; CPM: output high or low here at fixed location instead of all over the IRQ routine causing jitter
#ifdef SPECIAL_SIMON
	ld	a,#1	; Always on (38kHz carrier)
#else
	ld	a,(IR_OnOff)
#endif
	jz	IrIsOff
	set	#2,(RTC) ; PA1 output infrared beam
	jmp	IrIsDone
IrIsOff:
	clr	#2,(RTC) ; PA1 no output infrared beam
IrIsDone:

	ld	a,(IR_Flag)
	and	a,#IR_FLAG_HDR_MARK
	jnz	IR_9MS_OK_Prc ; Have we sent the initial header?

	; Code for outputting the header pulse
Head_9ms_Prc:
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
	
	; Are we still in the header pulse (first "9ms")?
	ld	a,(IR_BaseTim0)
	cmp	a,#Tim_9ms.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#Tim_9ms.n1
	jc	INT_End

	; We have just finished the header pulse
	ld	a,(IR_Flag)
	or	a,#IR_FLAG_HDR_MARK ; Flag the header pulse as having been sent
	ld	(IR_Flag),a
	; Reset the phase timer
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	jmp	INT_End

IR_9MS_OK_Prc:
Ir_45ms_Chk_Prc:
	; Have we sent the header gap?
	ld	a,(IR_Flag)
	and	a,#IR_FLAG_HDR_SPACE
	jnz	IR_45MS_Ok_Prc

	; Have we finished the gap?
	ld	a,(IR_BaseTim0)
	cmp	a,#tim_45ms.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#tim_45ms.n1
	jc	IR_StartGap
#ifdef PROTOCOL_MAN
	; Do we want a start bit?
	ld	a,(IR_BaseTim0)
	cmp	a,#tim_gapms.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#tim_gapms.n1
	jc	IR_StartBit
#endif
	; Just finished the header gap
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	ld	a,(IR_Flag)
	or	a,#IR_FLAG_HDR_SPACE
	ld	(IR_Flag),a
	jmp	INT_End

IR_StartGap:
	; Send the header gap
	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
	jmp	INT_End
	
#ifdef PROTOCOL_MAN
IR_StartBit:	
	; Send the Manchester start bit
	set	#2,(IR_OnOff) ; PA1 no output infrared beam
	jmp	INT_End
#endif

IR_45MS_Ok_Prc:
Data_rx_Prc:
	ld	A,(IR_Flag)
	and	A,#IR_FLAG_PAYLOAD
	jnz	DATA_RX_OK_Prc

	; ====================================
	; Send the payload bits (originally 32 bits; now can also be 20, 16, 12, 9 or 4 bits)
Data_32bit_RX_Prc:
	ld	a,(IR_Num0)
	cmp	a,#IR_BitNum_Dat.n0
	ld	a,(IR_Num1)
	sbc	a,#IR_BitNum_Dat.n1
	jnc	RX_Data_32Bit_OK_Prc

	; Do we need to grab a new payload bit from the packet?
	ld	a,(IR_BIT_OK_FLAG)
	jnz	Rx_Ir_01_Prc
	ld	a,#01
	ld	(IR_BIT_OK_FLAG),a

	; Grab the next payload bit from the packet (destructively)
	ld	a,#0
	ld	(IR_Bit),a ; Default payload bit is zero
	clr	C
	rrc	(IR_PktData7) 
	rrc	(IR_PktData6)
	rrc	(IR_PktData5)
	rrc	(IR_PktData4)
	rrc	(IR_PktData3)
	rrc	(IR_PktData2)
	rrc	(IR_PktData1)
	rrc	(IR_PktData0) 

	jnc	Rx_Ir_01_Prc	; Are we a one or zero? Jump if zero
	ld	a,#01
	ld	(IR_Bit),a ; This payload bit is a one bit
	
Rx_Ir_01_Prc:
	ld	a,(IR_Bit)
	jnz	Bit_1_RX_Prc

	; ====================================
	; Transmit a zero payload bit
Bit_0_RX_Prc:
Bit_0_1_RX_Prc:
	; Are we transmitting the first or second half of a zero bit?
	ld	a,(IR_BaseTim0)
	cmp	a,#IR_01_Dat.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#IR_01_Dat.n1
	jnc	Bit_0_0_RX_Prc 

	; First half of a zero bit
#ifdef PROTOCOL_MAN
	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
#else
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
#endif
	jmp	INT_End
	
Bit_0_0_RX_Prc:
	; Have we finished transmitting the zero bit?
	ld	a,(IR_BaseTim0)
	cmp	a,#ir_00_dat.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#ir_00_dat.n1
	jnc	RX_0_OK_Prc

	; Second half of a zero bit
#ifdef PROTOCOL_MAN
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
#else
	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
#endif
	jmp	INT_End

RX_0_OK_Prc:
	; Completed transmitting a zero bit
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	ld	(IR_BIT_OK_FLAG),a
	; New payload bit
	inc	(IR_Num0)
	adr	(IR_Num1)
	jmp	INT_End

	; ====================================
	; Transmit a one payload bit
Bit_1_RX_Prc:
Bit_1_1_RX_Prc:
	; Are we transmitting the first or second half of a one bit?
	ld	a,(IR_BaseTim0)
	cmp	a,#IR_11_Dat.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#IR_11_Dat.n1
	jnc	Bit_1_0_RX_Prc 

	; First half of a one bit
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
	jmp	INT_End
	
Bit_1_0_RX_Prc:
	; Have we finished transmitting the one bit?
	ld	a,(IR_BaseTim0)
	cmp	a,#ir_10_dat.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#ir_10_dat.n1
	jnc	RX_1_OK_Prc

	; Second half of a one bit
	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
	jmp	INT_End

RX_1_OK_Prc:
	; Completed transmitting a one bit
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	ld	(IR_BIT_OK_FLAG),a
	; New payload bit
	inc	(IR_Num0)
	adr	(IR_Num1)
	jmp	INT_End

; Have just transmitted all payload bits
RX_Data_32Bit_OK_Prc:
	ld	a,#0
	ld	(IR_Num0),a
	ld	(IR_Num1),a

	ld	a,(IR_Flag)
	or	a,#IR_FLAG_PAYLOAD
	ld	(IR_Flag),a
	; Transmit the stop bit (don't delay 100us!)
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
	jmp	INT_End 

DATA_RX_OK_Prc:
	; ====================================
	; Transmit the stop bit (all payload has been sent)
	ld	a,(IR_Flag)
	and	a,#IR_FLAG_STOP
	jnz	Ir_Last_data_Rx_Ok_Prc
IR_Last_Dat_RX_Prc:
	; Send the stop bit - have we finished sending it?
	ld	a,(IR_BaseTim0) 
	cmp	a,#IR_Last_Tim.n0
	ld	A,(IR_BaseTim1)
	sbc	a,#IR_Last_Tim.n1
	jnc	Last_Dat_RX__OK_Prc
	; Nope, still sending the stop bit
	set	#2,(IR_OnOff) ; PA1 output 38kHz infrared beam
	jmp	INT_End
	
Last_Dat_RX__OK_Prc:
	; Idle period after complete packet has been sent
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	ld	a,(IR_Flag)
	or	a,#IR_FLAG_STOP	; If not already true
	ld	(IR_Flag),a
	jmp	INT_End

; The complete packet has been sent
Ir_Last_data_Rx_Ok_Prc:
	ld	a,(IR_Flag)
	cmp	a,#IR_FLAG_PACKET
	jnz	INT_End 

	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
#ifdef SPECIAL_SIMON
#else
	clr	#2,(RTC) ; PA1 no output infrared beam
#endif
	ld	A,#00
	ld	(IR_Enable_Flag),A

; End of interrupt for timer2
INT_End:
	inc	(Tim_SendPkt1) ; Increment the 12 bit timer counter
	adr	(Tim_SendPkt2)
	adr	(Tim_SendPkt3)
; End of interrupt for timer1, timer2, RTC
INT_End_NoInc:
	ld	a,(A_SRT) ; Restore the A register
	reti

; Interrupt Routine to send special grenade explosion pulse instead of normal packet
IR_SendExplode:
	; Where are we in the transmission?
	inc	(IR_BaseTim0) ; Increment 8 bit timer
	adr	(IR_BaseTim1)

	ld	a,(IR_BaseTim0)
	cmp	a,#tim_explode_pulse.n0
	ld	a,(IR_BaseTim1)
	sbc	a,#tim_explode_pulse.n1
	jc	INT_End
	
	; Pulse has changed phase
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a

	inc	(ExplodePhase)
	ld	A,(ExplodePhase)
	and	A,#3
	jz	XIrIsOn
	cmp	A,#1
	jz	XIrIsOff
	; Reduce duty cycle on pulses to give receiver a chance to see them
	clr	#2,(RTC) ; PA1 no output infrared beam
	jmp	INT_End
XIrIsOn:
	set	#2,(RTC) ; PA1 output infrared beam
	jmp	INT_End
XIrIsOff:
	clr	#2,(RTC) ; PA1 no output infrared beam

	; Duplicate of code in Grenade_update_outi
	; Since on the Tritan chip an interrupt cannot call a subroutine (safely)
	inc	(g_outi)
	ld	a,(g_outi)
	and	a,#ADDRESS_MASK
#ifdef BOARD_STEPHEN
	#error
#endif

#ifdef BOARD_DEVELOP
	#error
#endif

#ifdef BOARD_8LEDS
	and	a,#7
	jz	igou_0
	cmp	a,#1
	jz	igou_1
	cmp	a,#2
	jz	igou_2
	cmp	a,#3
	jz	igou_3
	cmp	a,#4
	jz	igou_4
	cmp	a,#5
	jz	igou_5
	cmp	a,#6
	jz	igou_6
igou_7:
	clr	#PIN_EN7,(PORT_EN7)
	set	#PIN_EN8,(PORT_EN8)
	jmp	INT_End
igou_6:
	clr	#PIN_EN6,(PORT_EN6)
	set	#PIN_EN7,(PORT_EN7)
	jmp	INT_End
igou_5:
	clr	#PIN_EN5,(PORT_EN5)
	set	#PIN_EN6,(PORT_EN6)
	jmp	INT_End
igou_4:
	clr	#PIN_EN4,(PORT_EN4)
	set	#PIN_EN5,(PORT_EN5)
	jmp	INT_End
igou_3:
	clr	#PIN_EN3,(PORT_EN3)
	set	#PIN_EN4,(PORT_EN4)
	jmp	INT_End
igou_2:
	clr	#PIN_EN2,(PORT_EN2)
	set	#PIN_EN3,(PORT_EN3)
	jmp	INT_End
igou_1:
	clr	#PIN_EN1,(PORT_EN1)
	set	#PIN_EN2,(PORT_EN2)
	jmp	INT_End
igou_0:
	clr	#PIN_EN8,(PORT_EN8)
	set	#PIN_EN1,(PORT_EN1)
#endif
	jmp	INT_End


; -----------------------------------------------------------------------------
; MAIN PROGRAM entry point
PGMSRT:
;ERR_PC_ADR:
;PGM_Delay:
	ld	A,#0
	ld	(SYS0),A	; Disable interrupts
	ld	(USER1),A	; This nybble could be used by user code (and is)
	ld	(USER2),A	; This nybble could be used by user code (but isn't)

	; Initialise input/output (in powered down state)
	call	PODY_IO_Init
	
	; Delay loop (uses SRAM before it is cleared)
	; Should be 11 cycles * (0x100000-0xF5000) / 2 Mhz = about 0.25 seconds
	ldmah	#DELAY_MAH
	ld	A,#0
	ld	(CntDelay0),A
	ld	(CntDelay1),A
	ld	(CntDelay2),A
	ld	A,#05H ; Short delay
	ld	(CntDelay3),A
	ld	A,#0FH ; Short delay
	ld	(CntDelay4),A ; 20 bit timer 0xF5000
DELAY1:
	ld	A,#05H
	ld	(WDT),A ; Kick the watchdog

	ld	A,(CntDelay0)
	clr	C
	adc	A,#1
	ld	(CntDelay0),A
	adr	(CntDelay1)
	adr	(CntDelay2)
	adr	(CntDelay3)
	adr	(CntDelay4)
	jnc	DELAY1

	; Initialise input/output again
	ld	A,#0
	ld	(SYS0),A	; Disable interrupts
	call	PODY_IO_Init
	
	; Check that the button is pressed
	ld	a,(PORT_PWR_BTN)
	and	a,#BIT_PWR_BTN ; PD0 = user button
	jz	SkipLongDelay


	; Delay loop (uses SRAM before it is cleared)
	; Should be 11 cycles * (0x100000-0xE0000) / 2 Mhz = about 0.72 seconds
	ldmah	#DELAY_MAH
	ld	A,#0
	ld	(CntDelay0),A
	ld	(CntDelay1),A
	ld	(CntDelay2),A
	ld	(CntDelay3),A
#ifdef SPECIAL_SIMON
	ld	A,#09h ; Longer delay
#else
	ld	A,#0DH ; Short delay
#endif
	ld	(CntDelay4),A ; 20 bit timer 0xD0000
DELAY2:
	ld	A,#05H
	ld	(WDT),A ; Kick the watchdog

	ld	A,(CntDelay0)
	clr	C
	adc	A,#2
	ld	(CntDelay0),A
	adr	(CntDelay1)
	adr	(CntDelay2)
	adr	(CntDelay3)
	adr	(CntDelay4)
	jnc	DELAY2
SkipLongDelay:

	; Lets ensure these are cleared
	ld	A,#0
	ld	(SYS0),A	; Disable interrupts
	ld	(USER1),A	; This nybble could be used by user code (and is)
	ld	(USER2),A	; This nybble could be used by user code (but isn't)
	
	; Power up
	set	#PIN_PWR_EN,(PORT_PWR_EN)

	; Clear Banks 0..3 of SRAM
	ldmah	#3
	call	Clear_SRAM_INIT
	ldmah	#2
	call	Clear_SRAM_INIT
	ldmah	#1
	call	Clear_SRAM_INIT
	ldmah	#0
	call	Clear_SRAM_INIT

	; Setup timer2 interrupt every 100uS
	ldmah	#0
	call	Timer2_Init

	; Read the unique ID from OTP
	Call	Read_Mcu_ID
	call	SetWeapon

	; Initialise the grenade logic
	call	Grenade_init_button
	call	Grenade_init_logic

	; Create the CRC for the data packet
	call	CRC_Chk_Code
	; Setup a packet
	call	Data_Int_Code

	; Set the canary value for checking SRAM has not been modified
	ld	A,#Ram_chk_Dat.N0
	ld	(SramChk0_IN),A
	ld	A,#Ram_chk_Dat.N1
	ld	(SramChk1_IN),A 

	; Set the group counter to be the start value, but dont trigger it yet
	ld	A,#0
	ld	(IR_Group0),A
	ld	(IR_Group1),A
	ld	(IR_Group2),A


; -----------------------------------------------------------------------------
; Where wakeup code would return to
WakeUp:
	nop
	nop
	; We want the RTC interrupt to be every 1s instead of 125ms to avoid disruption
	set	#1,(RTC)
	set	#0,(RTC)
	; Initialise output to be sure
	ld	a,#0
	ld	(IR_Enable_Flag),A
	ld	(SendExplode),A
	ld	(g_send),a
	ld	(g_update),a
	ld	(Payload0),a
	ld	(Payload1),a
	ld	(Payload2),a
	ld	(Payload3),a
	ld	(IR_CRC_Buf0),A
	ld	(IR_CRC_Buf1),A
	call	Data_Int_Code
	
; -----------------------------------------------------------------------------
; Main loop for background tasks
; The call stack is too short to implement routines but these tasks are effectively subroutines
; They can call subroutines themselves.
MAIN_LOOP:
	; Task 1 - system sanity check
	jmp	SYS_Check_Prc
SYS_Check_RP:	

	; Task 2 - check for sending packets
	jmp	Tim_SendPkt_Chk_Prc
Tim_SendPkt_Chk_RP: 

	; Task 3 - check for sleep
	jmp	Chk_Halt_Tim_Prc
Chk_Halt_Tim_RP:

	nop
	nop
	nop
	nop
	jmp	MAIN_LOOP
	
	
; -----------------------------------------------------------------------------
; Main loop task 1/3
; System sanity check
SYS_Check_Prc:
	; Setup i/o ports (relevant if we have woken up recently)
	ldmah	#0

	; Set ports for input/output
#ifdef BOARD_STEPHEN
	ld	a,#0111b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B direction (0=input/1=output)
	ld	a,#1110b
	ld	(IOC_PD),a	; Port D direction (0=input/1=output)
#endif

#ifdef BOARD_DEVELOP
	ld	a,#0110b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B direction (0=input/1=output)
	ld	a,#1110b
	ld	(IOC_PD),a	; Port D direction (0=input/1=output)
#endif

#ifdef BOARD_8LEDS
	ld	a,#0111b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B direction (0=input/1=output)
	ld	a,#1111b
	ld	(IOC_PD),a	; Port D direction (0=input/1=output)
#endif

	; Kick the watchdog
	ld	a,#05h
	ld	(WDT),a
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid
	
	; Check the SRAM canary to see if it has been corrupted
	ld	a,#Ram_chk_Dat.n0
	cmp	a,(SramChk0_IN)
	jnz	PGMSRT
	ld	a,#Ram_chk_Dat.n1
	cmp	a,(SramChk1_IN)
	jnz	PGMSRT

	; Return to the next task in the main loop
	jmp	SYS_Check_RP

; -----------------------------------------------------------------------------
; Main loop task 2/3
; Check for the packet trigger events (30ms / 100ms), i.e. that a new packet should be triggered
Tim_SendPkt_Chk_Prc:

	; Update the logic to see if we want to trigger off a group
	call	Grenade_read_button
	call	Grenade_update_logic
	call	Grenade_update_visible

	; Are we wanting to transmit now?
	ld	a,(g_update)
	or	a,(g_send)
	jz	Tim_SendPkt_Chk_RP
	
	; Are we in special explode pulse mode?
	ld	a,(SendExplode)
	jnz	Tim_SendPkt_Chk_RP	; Let the interrupt take care of grenade cycling


SendPkt_TestAgain:
	; Are we in explosion mode (many fast+slow packets) or tick mode (fewer fast packets)
	ld	a,(g_state)
	cmp	a,#state_explode
	jnz	SendPkt_TestFast	; Just test for fast packets

SendPkt_TestExplode:
	; OK we want to be sending packets now
	; Are we in fast packet mode or slow packet mode
	ld	a,(IR_Group0)
	cmp	a,#ExplodeNumFast.n0
	ld	a,(IR_Group1)
	sbc	a,#ExplodeNumFast.n1
	ld	a,(IR_Group2)
	sbc	a,#ExplodeNumFast.n2
	jc	SendPkt_TestFast

SendPkt_TestSlow:
	; Test timing for slow packets
	; Note that an interrupt here can cause the counter to be inconsistent
	ld	a,(Tim_SendPkt1)
	ld	(LastTim1),a	; Because Tim_SendPkt1 is volatile!
	cmp	a,#Tim_SendPkt_S.n0
	ld	a,(Tim_SendPkt2)
	sbc	a,#Tim_SendPkt_S.n1
	ld	a,(Tim_SendPkt3)
	sbc	a,#Tim_SendPkt_S.n2
	
	jc	Tim_SendPkt_Chk_RP
	jmp 	SendPkt_Trigger

SendPkt_TestFast:
	; Test timing for fast packets
	; Note that an interrupt here can cause the counter to be inconsistent
	ld	a,(Tim_SendPkt1)
	ld	(LastTim1),a	; Because Tim_SendPkt1 is volatile!
	cmp	a,#Tim_SendPkt_F.n0
	ld	a,(Tim_SendPkt2)
	sbc	a,#Tim_SendPkt_F.n1
	ld	a,(Tim_SendPkt3)
	sbc	a,#Tim_SendPkt_F.n2
	jc	Tim_SendPkt_Chk_RP

SendPkt_Trigger:
	; Sanity check - has the bottom nybble of the 12 bit number changed during the comparison, invalidating it?
	ld	a,(Tim_SendPkt1)
	cmp	a,(LastTim1)
	jnz	SendPkt_TestAgain	; Redo the test! If the number went from 1FF to 200 during the test (because interrupt) we might think it was 2FF!
	
	; We can start triggering a new packet - stop current output
	ld	a,#0 ; Disable messing with IR
	ld	(IR_Enable_Flag),a ; Should already be set though
	
	; Increment packet count within the group
	inc	(IR_Group0)
	adr	(IR_Group1)
	adr	(IR_Group2)

	; Are we in the explosion state?
	ld	a,(g_state)
	cmp	a,#state_explode
	jz	SendPkt_Xgroup

	; Have we finished the set of fast packets for the tick?
	ld	a,(IR_Group0)
	cmp	a,#WarningNumTotal.n0
	ld	a,(IR_Group1)
	sbc	a,#WarningNumTotal.n1
	ld	a,(IR_Group2)
	sbc	a,#WarningNumTotal.n2
	jc	SendPkt_NoWrap
	jmp	SendPkt_EndGroup

SendPkt_Xgroup:
	; Have we finished the set of fast and slow packets for the explosion?
	ld	a,(IR_Group0)
	cmp	a,#ExplodeNumTotal.n0
	ld	a,(IR_Group1)
	sbc	a,#ExplodeNumTotal.n1
	ld	a,(IR_Group2)
	sbc	a,#ExplodeNumTotal.n2
	jc	SendPkt_NoWrap
SendPkt_EndGroup:
	; End of this group.
	; Start the next set of packets - but only when the logic say so
	ld	a,#0
	ld	(IR_Group0),a
	ld	(IR_Group1),a
	ld	(IR_Group2),a
	ld	(g_send),a
SendPkt_NoWrap:
	
	ld	a,(g_update)
	jz	spk_same
	dec	(g_update)
	inc	(g_send)
	call	CRC_Chk_Code
spk_same:
	call	Data_Int_Code

	; Look at grenade logic for changing the payload
	ld	a,#OUTPUT_POWER ; full power
	call	Grenade_update_power
	call	Grenade_update_outi
	ld	a,(g_send)
	jz	Tim_SendPkt_Chk_RP

	; Now it is safe to trigger the new packet
	; Clear interrupts before resetting packet count to make it atomic
	clr	#1,(SYS0) ; Clear ENINT, disabling all interrupts
	nop	; Safety (recommended after clearing ENINT)
	ld	a,#0
	ld	(Tim_SendPkt1),a
	ld	(Tim_SendPkt2),a
	ld	(Tim_SendPkt3),a
	ld	a,#1
	ld	(IR_Enable_Flag),a ; IR enabled
	ld	a,#0
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid

	; Return to the next task in the main loop
	jmp	Tim_SendPkt_Chk_RP

; -----------------------------------------------------------------------------
; Main loop task 3/3
; Check for going to sleep (after sending many packets).
; This should only happen when the logic sets sleep count to a high value.
Chk_Halt_Tim_Prc:
#ifdef SPECIAL_SIMON
	jmp	Chk_Halt_Tim_RP
#endif
	ld	a,(g_poweroff)
	jz	Chk_Halt_Tim_End

	; OK now we want to turn off the power
	clr	#1,(SYS0) ; Clear ENINT and disable interrupts
	nop
	
	; Turn ON the visible LED and infrared LED, to draw down the capacitor
	set	#2,(RTC)        ; PA1 no output infrared beam
#ifdef BOARD_STEPHEN
	ld	a,#1000b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0011b
	ld	(data_pd),a	; Port D data (0=low/1=high)
#endif
#ifdef BOARD_DEVELOP
	ld	a,#0000b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#1000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#1111b
	ld	(data_pd),a	; Port D data (0=low/1=high)
#endif
#ifdef BOARD_8LEDS
	; Important pins: PB0 = 0 (power off)
	; PA2 = 0 (LED on to draw some current)
	; PD1 = 1 (IR LED on to draw some current)
	ld	a,#1010b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0010b
	ld	(data_pd),a	; Port D data (0=low/1=high)
#endif
	nop
PowerOffLoop:
	; We could be stuck here a while if the button is being held!
	nop
	nop
	jmp	PowerOffLoop
	nop
	
	; Orphaned code = this used too little current so it did not drain the capacitor.
	halt 
Chk_Halt_Tim_End:
	jmp	Chk_Halt_Tim_RP

; -----------------------------------------------------------------------------
; CRC check procedure
; Create a checksum of the grenade ID
; (Does not need to have the packet setup like it used to)
CRC_Chk_Code:
	; Copy the wanted data into the CRC work buffer
#ifdef PROTOCOL_9
	; No CRC required, but no harm done.
#endif
	ld	a,(Payload0)
	ld	(CRC_DATA0),a
	ld	a,(Payload1)
	ld	(CRC_DATA1),a
	ld	a,(Payload2)
	ld	(CRC_DATA2),a
	ld	a,(Payload3)
#ifdef PROTOCOL_20
	and	a,#1	; 13 bits instead of 16
#endif
	ld	(CRC_DATA3),a
#ifdef PROTOCOL_12
	ld	a,#0
	ld	(CRC_DATA2),a
	ld	(CRC_DATA3),a
#endif
#ifdef PROTOCOL_4
	ld	a,#0
	ld	(CRC_DATA1),a
	ld	(CRC_DATA2),a
	ld	(CRC_DATA3),a
#endif

	; Setup the initial CRC value
	ld	a,#C_InitVal.n0
	ld	(IR_CRC_Buf0),a
	ld	a,#C_InitVal.n1
	ld	(IR_CRC_Buf1),a

	; Loop through the CRC calculations
	ld	a,#0
	ld	(Dat_Len_Num),a
	
	; For each payload bit
CRC_LOOP:
	rlc	(IR_CRC_Buf0)
	rlc	(IR_CRC_Buf1) 
	rlc	(CRC_DATA0) ; Carry in is not a problem since it was zero
	rlc	(CRC_DATA1)
	rlc	(CRC_DATA2)
	rlc	(CRC_DATA3) 
	jnc	CRC_Div_End
	; This bit (starting from msb) was set
	ld	a,(CRC_DATA3)
	xor	a,#GenPoly.n1
	ld	(CRC_DATA3),a
	ld	a,(CRC_DATA2)
	xor	a,#GenPoly.n0
	ld	(CRC_DATA2),a
CRC_Div_End:
	; Go to next bit
	inc	(Dat_Len_Num)
	ld	a,(Dat_Len_Num)
	cmp	a,#CRC_DataCnt.n0
	jnz	CRC_LOOP

	; Save the generated byte (8 bits)
	ld	a,(CRC_DATA2)
	ld	(IR_CRC_Buf0),a
	ld	a,(CRC_DATA3)
	ld	(IR_CRC_Buf1),a
	rets

	
; -----------------------------------------------------------------------------
; Setup the data packet for output
; Called on startup and in main thread
; This is the code that determines the logical packet layout (up to 32 bits per packet)
; (Needs to have the CRC calculated already)
Data_Int_Code:
#ifdef PROTOCOL_32
	; 8 bits CRC
	ld	a,(IR_CRC_Buf0)
	ld	(IR_PktData0),a
	ld	a,(IR_CRC_Buf1)
	ld	(IR_PktData1),a
	; 16 bits serial number
	ld	a,(Payload0)
	ld	(IR_PktData2),A
	ld	a,(Payload1)
	ld	(IR_PktData3),A
	ld	a,(Payload2)
	ld	(IR_PktData4),a
	ld	a,(Payload3)
	ld	(IR_PktData5),a
	; 8 bits fixed data
	ld	A,#User_Dat_2Byte.N0
	ld	(IR_PktData6),A
	ld	A,#User_Dat_2Byte.N1
	ld	(IR_PktData7),A
#endif
#ifdef PROTOCOL_20
	ld	A,#0
	ld	(IR_PktData3),A
	ld	(IR_PktData6),A
	ld	(IR_PktData7),A
	; 6 bit CRC
	ld	a,(IR_CRC_Buf0)
	ld	(IR_PktData4),a
	ld	a,(IR_CRC_Buf1)
	ld	(IR_PktData5),a
	clr	c
	rrc	(IR_PktData5)
	rrc	(IR_PktData4)
	rrc	(IR_PktData3)
	clr	c
	rrc	(IR_PktData5)
	rrc	(IR_PktData4)
	rrc	(IR_PktData3)
	; 13 bits serial number+1bit beacon
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	ld	a,(Payload1)
	ld	(IR_PktData1),a
	ld	a,(Payload2)
	ld	(IR_PktData2),A
	ld	a,(Payload3)
	and	a,#1
	or	a,(IR_PktData3)
	ld	(IR_PktData3),A
#endif
#ifdef PROTOCOL_20A
	; 16 bits serial number+1 bit beacon+3 bits crc
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	ld	a,(Payload1)
	ld	(IR_PktData1),a
	ld	a,(Payload2)
	ld	(IR_PktData2),A
	ld	a,(Payload3)
	ld	(IR_PktData3),A
	; Beacon (bit 0) + CRC (bits 1..3)
	ld	a,(IR_CRC_Buf0)
	and	a,#1110b
	ld	(IR_PktData4),a
	ld	a,#0
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
#ifdef PROTOCOL_16
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	ld	a,(Payload1)
	ld	(IR_PktData1),a
	ld	a,(Payload2)
	ld	(IR_PktData2),A
	ld	a,(Payload3)
	ld	(IR_PktData3),A
	ld	a,#0
	ld	(IR_PktData4),a
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
#ifdef PROTOCOL_9
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	xor	a,#15
	ld	(IR_PktData1),a
	ld	a,#0
	ld	(IR_PktData2),a
	ld	(IR_PktData3),a
	ld	(IR_PktData4),a
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
#ifdef PROTOCOL_12
	; 4 bits state, 4 bits id, 4 bits crc
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	ld	a,(Payload1)
	ld	(IR_PktData1),a
	ld	a,(IR_CRC_Buf0)
	clr	c
	adc	a,(IR_CRC_Buf1)
	ld	(IR_PktData2),a
	ld	a,#0
	ld	(IR_PktData3),a
	ld	(IR_PktData4),a
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
#ifdef PROTOCOL_4
	; 4 bits state
	ld	a,(Payload0)
	ld	(IR_PktData0),a
	ld	a,#0
	ld	(IR_PktData1),a
	ld	(IR_PktData2),a
	ld	(IR_PktData3),a
	ld	(IR_PktData4),a
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
	; Trigger packet
	ld	A,#0
	ld	(IR_Flag),A
	ld	(IR_BIT_OK_FLAG),a
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	rets

; -----------------------------------------------------------------------------
; Retrieve the 16 bit unique ID (serial number) from the OTP memory
; Output: Mcu_ID is valid
Read_Mcu_ID:
#ifdef USE_FIXED_SERIAL
	ld	a,#05H
	ld	(Mcu_ID0),A
	ld	(Mcu_ID1),A
	ld	(Mcu_ID2),A
	ld	(Mcu_ID3),A
#else
	; Setup pointer to program OTP
	ld	a,#05H
	ld	(Dma2),a
	ld	a,#0ch
	ld	(dma1),a
	ld	a,#0eh
	ld	(dma0),a
	nop
	nop
	nop 
	nop
	nop
	nop
	nop
	nop
	; Read the first 12 bits
	ld	a,(dmdl)
	ld	(Mcu_ID0),A
	ld	a,(dmdm)
	ld	(Mcu_ID1),A
	ld	A,(dmdh)
	ld	(Mcu_ID2),A

	; Read the next 4 bits
	inc	(dma0)
	nop
	nop
	nop
	nop
	nop
	nop
	ld	a,(DMDL)
	ld	(Mcu_ID3),A
	 
	nop
	nop
#endif
	rets 

; -----------------------------------------------------------------------------
; Combine the 16 bit unique ID (serial number) to make an ID payload
; Input: Mcu_ID is valid
; Output: Weapon is valid
SetWeapon:
	clr	c
	ld	a,(Mcu_ID0)
	adc	a,(Mcu_ID1)
	ld	(Weapon1),a
	adc	a,(Mcu_ID2)
	adc	a,(Mcu_ID3)
	and	a,#12
	or	a,#3
	ld	(Weapon0),a
	rets

; -----------------------------------------------------------------------------
; IO port initialization settings (e.g. pullup/pulldown/wakeup)
PODY_IO_Init:
#ifdef BOARD_STEPHEN
	; Pin A0* EN_LED1 (active high) = output high for firing
	; Pin A1* MOD_OUT (active high) = output low but PWM high on firing
	; Pin A2* LED     (active low)  = output low
	; Pin A3*         (unused)      = input pull high
	ld	a,#0111b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1001b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pawk),a	; Port A wakeup - none
	ld	a,#1000b
	ld	exio(papu),a	; Port A pull up 100kOhm resistor
	ld	a,#0000b
	ld	exio(papl),a	; Port A pull down 100kOhm resistor
	
	; Pin B0* LVL_2   (active high) = output low
	; Pin B1* LVL_3   (active high) = output high
	; Pin B2  LVL_1   (active high) = output low
	; Pin B3  EN_LED2 (active high) = output low
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
	ld	a,#0010b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pbwk),a	; Port B wakeup
	ld	a,#0000b
	ld	exio(pbpu),a	; Port B pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pbpl),a	; Port B pull down 100kOhm resistor

	; Pin D0  PWD_BTN (active low)  = input (pull up) wakeup
	; Pin D1  PWR_EN  (active high) = output high
	; Pin D2  EN_LED4 (active high) = output low
	; Pin D3  EN_LED3 (active high) = output low
	ld	a,#1110b
	ld	(IOC_PD),a	; Port D dir (0=input/1=output)
	ld	a,#0011b
	ld	(data_pd),a	; Port D data (0=low/1=high)
	ld	a,#0001b
	ld	exio(pdwk),a	; Port D wakeup
	ld	a,#0001b
	ld	exio(pdpu),a	; Port D pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pdpl),a	; Port B pull down 100kOhm resistor - none
#endif
#ifdef BOARD_DEVELOP
	; Pin A0          (unused)      = input pull high
	; Pin A1  MOD_OUT (active high) = output low but PWM high on firing
	; Pin A2  LED     (active low)  = output low
	; Pin A3          (unused)      = input pull high
	ld	a,#0110b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1001b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pawk),a	; Port A wakeup - none
	ld	a,#1001b
	ld	exio(papu),a	; Port A pull up 100kOhm resistor
	ld	a,#0000b
	ld	exio(papl),a	; Port A pull down 100kOhm resistor

	; Pin B0  EN_A0   (active high) = output low
	; Pin B1  EN_A1   (active high) = output low
	; Pin B2  EN_A2   (active high) = output low
	; Pin B3  LVL_1   (active low)  = output high
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
	ld	a,#1000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pbwk),a	; Port B wakeup
	ld	a,#0000b
	ld	exio(pbpu),a	; Port B pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pbpl),a	; Port B pull down 100kOhm resistor

	; Pin D0  PWD_BTN (active low)  = input (pull up) wakeup
	; Pin D1  PWR_EN  (active high) = output high
	; Pin D2  LVL_3   (active low)  = output high
	; Pin D3  LVL_2   (active low)  = output high
	ld	a,#1110b
	ld	(IOC_PD),a	; Port D dir (0=input/1=output)
	ld	a,#1111b
	ld	(data_pd),a	; Port D data (0=low/1=high)
	ld	a,#0001b
	ld	exio(pdwk),a	; Port D wakeup
	ld	a,#0001b
	ld	exio(pdpu),a	; Port D pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pdpl),a	; Port B pull down 100kOhm resistor - none
#endif
#ifdef BOARD_8LEDS
	; Pin A0  EN_     (active high) = output low
	; Pin A1  MOD_OUT (active high) = output high but PWM high on firing
	; Pin A2  LED     (active low)  = output low
	; Pin A3  PWD_BTN (active low)  = input (pull up) no wakeup
	ld	a,#0111b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1010b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pawk),a	; Port A wakeup - none
	ld	a,#1000b
	ld	exio(papu),a	; Port A pull up 100kOhm resistor
	ld	a,#0000b
	ld	exio(papl),a	; Port A pull down 100kOhm resistor

	; Pin B0  PWR_EN  (active high) = output LOW (needs to be high soon)
	; Pin B1  EN_     (active high) = output low
	; Pin B2  EN_     (active high) = output low
	; Pin B3  EN_     (active high) = output low
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pbwk),a	; Port B wakeup
	ld	a,#0000b
	ld	exio(pbpu),a	; Port B pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pbpl),a	; Port B pull down 100kOhm resistor

	; Pin D0  EN_     (active high) = output low
	; Pin D1  EN_     (active high) = output low
	; Pin D2  EN_     (active high) = output low
	; Pin D3  EN_     (active high) = output low
	ld	a,#1111b
	ld	(IOC_PD),a	; Port D dir (0=input/1=output)
	ld	a,#0000b
	ld	(data_pd),a	; Port D data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pdwk),a	; Port D wakeup
	ld	a,#0000b
	ld	exio(pdpu),a	; Port D pull up 100kOhm resistor 
	ld	a,#0000b
	ld	exio(pdpl),a	; Port B pull down 100kOhm resistor - none
#endif
	rets


; -----------------------------------------------------------------------------
; Clear a single bank (32 nybbles) of SRAM
Clear_SRAM_INIT:
	ld	A,#00H
	ld	(20H),A
	ld	(21H),A
	ld	(22H),A
	ld	(23H),A
	ld	(24H),A
	ld	(25H),A
	ld	(26H),A
	ld	(27H),A
	ld	(28H),A
	ld	(29H),A
	ld	(2AH),A
	ld	(2BH),A
	ld	(2CH),A
	ld	(2DH),A
	ld	(2EH),A
	ld	(2FH),A
	ld	(30H),A
	ld	(31H),A
	ld	(32H),A
	ld	(33H),A
	ld	(34H),A
	ld	(35H),A
	ld	(36H),A
	ld	(37H),A
	ld	(38H),A
	ld	(39H),A
	ld	(3AH),A
	ld	(3BH),A
	ld	(3CH),A
	ld	(3DH),A
	ld	(3EH),A
	ld	(3FH),A	 
	rets
	
; -----------------------------------------------------------------------------
; Set up timer 2 for 100uS interrupt (12.5kHz) +/- 2%
; High speed oscillator HRCOSC = 32Mhz
; CPU clock FMCK = /4 = 8Mhz
; Scaler2 = Div8 = 1MHz
; Timer2 = 0xB0 = 12.5kHz
Timer2_Init:
	ld	a,#0
	ld	(TMCTL),A;3:TM2EN,2:TM1EN,1:TM1SCK,0:TM1ALD
	nop 
	set	#3,(TMCTL)
	set	#3,(SYS0) ;3:TM2SK,2:TM1SK,1:ENINI,0:PWM0

	clr	#1,(SYS0) ; Clear ENINI (disable interrupts)
	nop	; Safety measure from clearing global interrupts
	clr	#0,(SYS0) ; Clear PWM0 mode
	nop

	; Set Timer2 Autoload enabled and scalar to div8 (i.e. 1Mhz)
	ld	a,#1101b
	ld	(SCALER2),A ;div2-div1-div0. 8M/8=1. auto download

	; timing = (256 - time val/(time clock)) so 0xCE is 50uS
	ld	A,#Tim2_Speed.N0 ; Low nybble
	ld	(TIM2),A
	ld	A,#Tim2_Speed.N1 ; High nybble
	ld	(TIM2),A

;	set	#1,(SYS0) ; Set ENINI
	nop
Timr2_Init_End:
	rets

; *****************************************************************************

; ----------------------------------------------------------------------------
; Reset the grenade logic
Grenade_init_logic:
	ld	a,#0
	ld	(g_send),a
	ld	(g_update),a
	ld	a,#ADDRESS_MASK
	ld	(g_outi),A
	ld	a,(BtnNow)	; Has the user pressed the button?
	jz	Grenade_Arm	; Not holding button down
	jmp	Grenade_Wait	; Holding button down
	

; ----------------------------------------------------------------------------
; Set the state to a new one
; Input: A= new state
Grenade_SetState:
	ld	(g_state),A
	nop
	ld	(g_state),a
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a
	ld	(g_substate0),a
	ld	(g_substate1),a
	rets

; ----------------------------------------------------------------------------
Grenade_Arm:
	ld	a,#outstate_none
	ld	(outstate),A
	ld	a,#state_5
	jmp	Grenade_SetState

; ----------------------------------------------------------------------------
Grenade_Wait:
	ld	a,#outstate_none
	ld	(outstate),A
	ld	a,#state_waiting
	jmp	Grenade_SetState

; ----------------------------------------------------------------------------
Grenade_Prime:
	ld	a,#outstate_none
	ld	(outstate),A
	ld	a,#state_priming
	jmp	Grenade_SetState

; ----------------------------------------------------------------------------
Grenade_Primed:
	ld	a,#0
	ld	(g_quiet),A
	ld	a,#outstate_primed
	ld	(outstate),A
	ld	a,#state_primed
	jmp	Grenade_SetState

; ----------------------------------------------------------------------------
Grenade_Cancel:
	ld	a,#outstate_cancelled
	ld	(outstate),A
	ld	a,#state_cancelled
	jmp	Grenade_SetState


; ----------------------------------------------------------------------------
; Code to update the grenade visible LED (PA2)
Grenade_update_visible:
	ld	a,(g_state)
	cmp	a,#state_unarmed
	jz	gvis_off
	cmp	a,#state_cancelled
	jz	gvis_off
	cmp	a,#state_waiting
	jz	gvis_on
	cmp	a,#state_priming
	jz	gvis_priming
	cmp	a,#state_primed
	jz	gvis_primed
	cmp	a,#state_explode
	jz	gvis_explode
	
	cmp	a,#state_1
	jz	gvis_faster
	cmp	a,#state_2
	jz	gvis_faster
	cmp	a,#state_3
	jz	gvis_fast
	cmp	a,#state_4
	jz	gvis_fast
	cmp	a,#state_5
	jz	gvis_fast
	jmp	gvis_slow
	
gvis_priming:
	; Priming
	ld	a,(g_substate0)
	and	a,#1
	jnz	gvis_on
	jmp	gvis_off
	
gvis_primed:
	; Primed
	ld	a,(g_quiet)
	jz	gvis_on
	jmp	gvis_off

gvis_explode:	
	; Explosion
	ld	a,(g_timer1)
	and	a,#12
	jz	gvis_off
	jmp	gvis_on

gvis_faster:
	; Faster ticking
	ld	a,(g_substate0)
	and	a,#1
	jz	gvis_off
	jmp	gvis_on

gvis_fast:	
	; Fast ticking
	ld	a,(g_substate1)
	jnz	gvis_on
	ld	a,(g_substate0)
	cmp	a,#1
	jz	gvis_off
	cmp	a,#6
	jz	gvis_off
	jmp	gvis_on

gvis_slow:
	; In countdown phase
	ld	a,(g_substate1)
	jnz	gvis_on
	ld	a,(g_substate0)
	cmp	a,#1
	jz	gvis_off
	; fall through

gvis_on:
	clr	#PIN_LED,(PORT_LED)
	rets

gvis_off:
	set	#PIN_LED,(PORT_LED)
	rets


; ----------------------------------------------------------------------------
; Code to update the output LED choice
; (Stephen) PA0=EN_LED1, PB3=EN_LED2, PD3=EN_LED3, PD2=EN_LED4
; (Develop) PB0, PB1, PB2 = address of LEDs (0..7)
; (8LEDS)   PD1, PD0, PB2, PB1, PB3, PD2, PA0, PD3 = EN_LED (1..8)
; If there is no packet wanted, turn them all off
Grenade_update_outi:
	ld	a,(g_update)
	or	a,(g_send)
	jz	gou_off

	inc	(g_outi)
	ld	a,(g_outi)
	and	a,#ADDRESS_MASK
#ifdef BOARD_STEPHEN
	and	a,#3
	jz	gou_0
	cmp	a,#1
	jz	gou_1
	cmp	a,#2
	jz	gou_2
gou_3:
	clr	#PIN_EN3,(PORT_EN3)
	set	#PIN_EN4,(PORT_EN4)
	rets
gou_2:
	clr	#PIN_EN2,(PORT_EN2)
	set	#PIN_EN3,(PORT_EN3)
	rets
gou_1:
	clr	#PIN_EN1,(PORT_EN1)
	set	#PIN_EN2,(PORT_EN2)
	rets
gou_0:
	clr	#PIN_EN4,(PORT_EN4)
	set	#PIN_EN1,(PORT_EN1)
	rets
gou_off:
	clr	#PIN_EN1,(PORT_EN1)
	clr	#PIN_EN2,(PORT_EN2)
	clr	#PIN_EN3,(PORT_EN3)
	clr	#PIN_EN4,(PORT_EN4)
#endif

#ifdef BOARD_DEVELOP
	and	a,#MASK_ENBITS
	ld	(g_tmp),a
	ld	a,(PORT_ENBITS)
NOTMASK_ENBITS	equ	15-MASK_ENBITS
	and	a,#NOTMASK_ENBITS
	or	a,(g_tmp)
	ld	(PORT_ENBITS),a
#endif

#ifdef BOARD_8LEDS
	and	a,#7
	jz	gou_0
	cmp	a,#1
	jz	gou_1
	cmp	a,#2
	jz	gou_2
	cmp	a,#3
	jz	gou_3
	cmp	a,#4
	jz	gou_4
	cmp	a,#5
	jz	gou_5
	cmp	a,#6
	jz	gou_6
gou_7:
	clr	#PIN_EN7,(PORT_EN7)
	set	#PIN_EN8,(PORT_EN8)
	rets
gou_6:
	clr	#PIN_EN6,(PORT_EN6)
	set	#PIN_EN7,(PORT_EN7)
	rets
gou_5:
	clr	#PIN_EN5,(PORT_EN5)
	set	#PIN_EN6,(PORT_EN6)
	rets
gou_4:
	clr	#PIN_EN4,(PORT_EN4)
	set	#PIN_EN5,(PORT_EN5)
	rets
gou_3:
	clr	#PIN_EN3,(PORT_EN3)
	set	#PIN_EN4,(PORT_EN4)
	rets
gou_2:
	clr	#PIN_EN2,(PORT_EN2)
	set	#PIN_EN3,(PORT_EN3)
	rets
gou_1:
	clr	#PIN_EN1,(PORT_EN1)
	set	#PIN_EN2,(PORT_EN2)
	rets
gou_0:
	clr	#PIN_EN8,(PORT_EN8)
	set	#PIN_EN1,(PORT_EN1)
	rets
gou_off:
	clr	#PIN_EN1,(PORT_EN1)
	clr	#PIN_EN2,(PORT_EN2)
	clr	#PIN_EN3,(PORT_EN3)
	clr	#PIN_EN4,(PORT_EN4)
	clr	#PIN_EN5,(PORT_EN5)
	clr	#PIN_EN6,(PORT_EN6)
	clr	#PIN_EN7,(PORT_EN7)
	clr	#PIN_EN8,(PORT_EN8)
	ld	a,#ADDRESS_MASK
	ld	(g_outi),A
#endif
	rets


; ----------------------------------------------------------------------------
; Input: a=power level (1..3)
; PB2/PB3=LVL_1
; PB0/PD3=LVL_2
; PB1/PD2=LVL_3
Grenade_update_power:
	cmp	a,#0
	jz	gup_0
	cmp	a,#1
	jz	gup_1
	cmp	a,#2
	jz	gup_2
	
#ifdef BOARD_STEPHEN
	; Active high
gup_3:
	clr	#PIN_LVL_1,(PORT_LVL_1)
	clr	#PIN_LVL_2,(PORT_LVL_2)
	set	#PIN_LVL_3,(PORT_LVL_3)
	rets
gup_2:
	clr	#PIN_LVL_1,(PORT_LVL_1)
	clr	#PIN_LVL_3,(PORT_LVL_3)
	set	#PIN_LVL_2,(PORT_LVL_2)
	rets
gup_1:
	clr	#PIN_LVL_2,(PORT_LVL_2)
	clr	#PIN_LVL_3,(PORT_LVL_3)
	set	#PIN_LVL_1,(PORT_LVL_1)
	rets
gup_0:
	clr	#PIN_LVL_2,(PORT_LVL_2)
	clr	#PIN_LVL_3,(PORT_LVL_3)
	clr	#PIN_LVL_1,(PORT_LVL_1)
#endif

#ifdef BOARD_DEVELOP
	; Active low
gup_3:
	set	#PIN_LVL_1,(PORT_LVL_1)
	set	#PIN_LVL_2,(PORT_LVL_2)
	clr	#PIN_LVL_3,(PORT_LVL_3)
	rets
gup_2:
	set	#PIN_LVL_1,(PORT_LVL_1)
	set	#PIN_LVL_3,(PORT_LVL_3)
	clr	#PIN_LVL_2,(PORT_LVL_2)
	rets
gup_1:
	set	#PIN_LVL_2,(PORT_LVL_2)
	set	#PIN_LVL_3,(PORT_LVL_3)
	clr	#PIN_LVL_1,(PORT_LVL_1)
	rets
gup_0:
	set	#PIN_LVL_2,(PORT_LVL_2)
	set	#PIN_LVL_3,(PORT_LVL_3)
	set	#PIN_LVL_1,(PORT_LVL_1)
#endif

#ifdef BOARD_8LEDS
	; No levels
gup_3:
gup_2:
gup_1:
gup_0:
#endif
	rets

; ----------------------------------------------------------------------------
; Initialise the button logic
Grenade_init_button:
	ld	a,(PORT_PWR_BTN)
	and	a,#BIT_PWR_BTN ; PD0 = user button
	xor	a,#BIT_PWR_BTN
	ld	(BtnLast),a
	ld	(BtnNow),a
	jz	gib_z
	ld	a,#15	; "analog" value on for button (to support debouncing)
gib_z:
	ld	(BtnSmooth),a
	ld	a,(g_timer1)
	ld	(BtnTimer),a
	rets
	
; ----------------------------------------------------------------------------
; Read the button. Smooth the transitions high or low.
; So it takes 25ms of the same value in order to change between high and low
Grenade_read_button:
	ld	a,(g_timer1)
	cmp	a,(BtnTimer)
	jnz	grb_new
	rets
grb_new:
	ld	(BtnTimer),a
	ld	a,(BtnNow)
	ld	(BtnLast),a
	
	; Read the new value of the button
	ld	a,(PORT_PWR_BTN)
	and	a,#BIT_PWR_BTN ; PD0 = user button
	xor	a,#BIT_PWR_BTN
	jz	grb_lower
	; Increment the smooth value
	ld	a,(BtnSmooth)
	cmp	a,#15
	jz	grb_on
	inc	(BtnSmooth)
	rets
	
grb_lower:
	ld	a,(BtnSmooth)
	jz	grb_off
	dec	(BtnSmooth)
	rets

; The button is pressed
grb_on:
	ld	a,#BIT_PWR_BTN
	ld	(BtnNow),a
	rets

; The button is released
grb_off:
	ld	a,#0
	ld	(BtnNow),a
	rets

; ----------------------------------------------------------------------------
; Update the grenade logic
; 
; State Name 	OnUp	Timeout	LED	
; ----------    ----    ------- ---
; 0	Unarmed	X	X	X
; 1	Cancel	X	Off	Off
; 2	Priming	Primed	X	Faster
; 3 	Primed	10	X	Solid and then off
; 4	10	Cancel	9	Slow
; 5	9	Cancel	8	Slow
; 6	8	Cancel	7	Slow
; 7	7	Cancel	6	Slow
; 8	6	Cancel	5	Slow
; 9	5	Cancel	4	Fast
; 10	4	Cancel	3	Fast
; 11	3	Cancel	2	Fast
; 12	2	Cancel	1	Faster
; 13	1	Cancel	Explode	Faster
; 14	Explode	X	Off	Solid
; 15	Waiting	10	Priming	On

Grenade_update_logic:
	ld	a,(g_state)
	cmp	a,#state_unarmed
	jz	gul_unarmed
	cmp	a,#state_cancelled
	jz	gul_cancelled
	cmp	a,#state_waiting
	jz	gul_waiting
	cmp	a,#state_priming
	jz	gul_priming
	cmp	a,#state_primed
	jz	gul_primed
	cmp	a,#state_explode
	jz	gul_explode

; -------------------------------------	
; Countdown
	; Cancelling? - on release
	ld	a,(BtnNow)
	jnz	gul_nocancel
	ld	a,(BtnLast)
	jnz	Grenade_Cancel	; Cancel the countdown
gul_nocancel:

	; 10, 9, 8, 7, 6, 5, 4, 3, 2, 1
	ld	a,#0
	ld	(SendExplode),A

	; Normal state
	ld	a,(g_timer0)
	cmp	a,#TIME_TICK.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_TICK.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_TICK.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_TICK.n3
	jnc	gul_tick ; >= tick
	rets

; We have had a 100ms tick in the normal grenade state
gul_tick:
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a

	ld	a,(g_substate0)
	or	a,(g_substate1)
	jnz	gul_notfirst
	
	; When we go into this state, send a group of packets
	ld	a,(Weapon1)
	ld	(Payload3),a
	ld	a,(Weapon0)
	ld	(Payload2),a
	ld	a,(g_random)
	ld	(Payload1),a
#ifdef PROTOCOL_OUTSTATE
	ld	a,(OutState)
	ld	(Payload0),a
	cmp	a,#outstate_none	; Implied anyway
	jz	gul_notfirst	; Don't output countdown packets any more
#else
	ld	a,(g_state)
	ld	(Payload0),a
#endif
	inc	(g_update) ; Allow new packet to be sent
gul_notfirst:

	; Go through the substates
	inc	(g_substate0)
	adr	(g_substate1)
	
	ld	a,(g_substate0)
	cmp	a,#COUNT_TICK	
	jnz	gul_not_next_state
	; Go to the next state
	inc	(g_state)
	ld	a,#0
	ld	(g_substate0),a
	ld	(g_substate1),a
	ld	a,#outstate_none
	ld	(outstate),A
	ld	a,(g_state)
	cmp	a,#state_explode
	jnz	gul_not_next_state
	; Start the explosion!
	ld	a,#outstate_explode0
	ld	(OutState),A
gul_not_next_state:
	rets


; -------------------------------------	
; In the unarmed state
gul_unarmed:
	ld	a,#0
	ld	(SendExplode),A
	rets	; Do nothing. So why are we not powered off?

; -------------------------------------	
; The grenade has been cancelled
gul_cancelled:
	ld	a,#0
	ld	(SendExplode),A
	ld	a,(g_timer0)
	cmp	a,#TIME_CANCELLED.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_CANCELLED.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_CANCELLED.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_CANCELLED.n3
	jnc	gul_ctick
	; Else do nothing
	rets
	
gul_ctick:
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a
	inc	(g_substate0)
	adr	(g_substate1)
	
	ld	a,(Weapon1)
	ld	(Payload3),a
	ld	a,(Weapon0)
	ld	(Payload2),a
	ld	a,(g_random)
	ld	(Payload1),a
#ifdef PROTOCOL_OUTSTATE
	ld	a,(OutState)
#else
	ld	a,(g_state)
#endif
	ld	(Payload0),a
	inc	(g_update) ; Allow new packet to be sent

	; Are we done?
	ld	a,(g_substate0)
	cmp	a,#COUNT_CANCELLED.n0
	ld	a,(g_substate1)
	sbc	a,#COUNT_CANCELLED.n1
	jnc	gul_off
	rets

	
gul_off:
	; Force power off
	ld	a,#1
	ld	(g_poweroff),a
	rets

; -------------------------------------	
; We are waiting to see if we will prime the grenade
gul_waiting:
	ld	a,#0
	ld	(SendExplode),A
	ld	a,(BtnNow)
	jz	Grenade_Arm

	ld	a,(g_timer0)
	cmp	a,#TIME_WAITING.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_WAITING.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_WAITING.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_WAITING.n3
#ifdef PROTOCOL_NEC4
	; There is no room for priming the grenade in this protocol
	rets	; Wait for user to release button before starting countdown
;	jnc	Grenade_Arm	; Arm it on timeout
#else
	jnc	Grenade_Prime
#endif
	; Else do nothing
	rets
	

; -------------------------------------	
; We are priming the grenade
gul_priming:
	ld	a,#0
	ld	(SendExplode),A
	inc	(g_random)	; Increment the value
	adr	(g_random)	; Avoid zero
	ld	a,(BtnNow)
	jz	Grenade_Primed

	ld	a,(g_timer0)
	cmp	a,#TIME_PRIMING.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_PRIMING.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_PRIMING.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_PRIMING.n3
	jnc	gul_priming_pkt
	; Else do nothing
	rets
	
gul_priming_pkt:
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a
	inc	(g_substate0)	; For LED flashing
	adr	(g_substate1)

	; Are we done?
	ld	a,(g_substate0)
	cmp	a,#COUNT_PRIMING.n0
	ld	a,(g_substate1)
	sbc	a,#COUNT_PRIMING.n1
	jnc	gul_priming_send
	rets
	
gul_priming_send:
	ld	a,#0
	ld	(g_substate0),a
	ld	(g_substate1),a

	; Send the priming event
	ld	a,(Weapon1)
	ld	(Payload3),a
	ld	a,(Weapon0)
	ld	(Payload2),a
	ld	a,#0		; No random yet
	ld	(Payload1),a
#ifdef PROTOCOL_OUTSTATE
	ld	a,(OutState)
	ld	(Payload0),a
	cmp	a,#outstate_none
	jz	gul_priming_exit	; No sending packet
#else
	ld	a,(g_state)
	ld	(Payload0),a
#endif
	inc	(g_update)
gul_priming_exit:
	rets

; -------------------------------------	
; We are in the primed state
gul_primed:
	ld	a,#0
	ld	(SendExplode),A
	; Arming? - on release
	ld	a,(BtnNow)
	jnz	gul_noarmp
	ld	a,(BtnLast)
	jnz	Grenade_Arm ; Arm the grenade
gul_noarmp:

	ld	a,(g_timer0)
	cmp	a,#TIME_PRIMED.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_PRIMED.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_PRIMED.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_PRIMED.n3
	jnc	gul_primed_pkt
	; Else do nothing
	rets

; We are in the quiet phase	
gul_primed_quiet:
	ld	a,#1
	ld	(g_quiet),a
	rets
		
gul_primed_pkt:
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a
	
	ld	a,(g_quiet)
	jnz	gul_primed_quiet
	
	inc	(g_substate0)	; For LED flashing
	adr	(g_substate1)

	ld	a,(g_substate0)
	cmp	a,#COUNT_PRIMED.n0
	ld	a,(g_substate1)
	sbc	a,#COUNT_PRIMED.n1
	jnc	gul_primed_quiet
	
	; Send the primed event
	ld	a,(Weapon1)
	ld	(Payload3),a
	ld	a,(Weapon0)
	ld	(Payload2),a
	ld	a,(g_random)
	ld	(Payload1),a
#ifdef PROTOCOL_OUTSTATE
	ld	a,(OutState)
#else
	ld	a,(g_state)
#endif
	ld	(Payload0),a
	inc	(g_update) ; Allow new packet to be sent

	rets

; -------------------------------------	
; In the explode state
gul_explode:
#ifdef PROTOCOL_EXPLODE_PULSE
	ld	a,#1
#else
	ld	a,#0
#endif
	ld	(SendExplode),A
	ld	a,(g_timer0)
	cmp	a,#TIME_EXPLODE.n0
	ld	a,(g_timer1)
	sbc	a,#TIME_EXPLODE.n1
	ld	a,(g_timer2)
	sbc	a,#TIME_EXPLODE.n2
	ld	a,(g_timer3)
	sbc	a,#TIME_EXPLODE.n3
	jnc	gul_boom
	; Else do nothing
	rets
	
gul_boom:
	ld	a,#0
	ld	(g_timer0),a
	ld	(g_timer1),a
	ld	(g_timer2),a
	ld	(g_timer3),a

	ld	a,(g_substate0)
	or	a,(g_substate1)
	jnz	gul_notbang

	; Send the explosion event
	ld	a,(Weapon1)
	ld	(Payload3),a
	ld	a,(Weapon0)
	ld	(Payload2),a
	ld	a,(g_random)
	ld	(Payload1),a
#ifdef PROTOCOL_OUTSTATE
	ld	a,(outstate)
#else
	ld	a,(g_state)
#endif
	ld	(Payload0),a
	inc	(g_update)
	; Reset to the beginning of the group
	ld	a,#0
	ld	(IR_Group0),a
	ld	(IR_Group1),a
	ld	(IR_Group2),a

gul_notbang:
	inc	(g_substate0)
	adr	(g_substate1)

	; Are we done?
	ld	a,(g_substate0)
	cmp	a,#COUNT_EXPLODE.n0
	ld	a,(g_substate1)
	sbc	a,#COUNT_EXPLODE.n1
	jnc	gul_done
	rets

gul_done:
#ifdef PROTOCOL_OUTSTATE
	ld	a,(OutState)
	cmp	a,#outstate_explode55
	jz	gul_alldone
	; OK we are still in explosion state but different counter
	inc	(OutState)
	ld	a,#0
	ld	(g_substate0),a
	ld	(g_substate1),a
	rets
	
gul_alldone:
#endif
	; We have finished the explosion
	ld	a,#0
	ld	(SendExplode),A
#ifdef SPECIAL_SAM
	ld	a,#state_10
#else
	ld	a,#state_unarmed
#endif
	ld	(g_state),a
	ld	a,#0
	ld	(g_substate0),a
	ld	(g_substate1),a

	; Force power off
#ifdef SPECIAL_SAM
#else
	ld	a,#1
	ld	(g_poweroff),a
#endif
	rets

; ----------------------------------------------------------------------------
