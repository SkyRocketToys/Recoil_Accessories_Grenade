; *****************************************************************************
; Recoil Gun Grenade code for SkyRocket Toys 2017
; Based on TR4K-IDE Demo Code by TRITAN Technology Inc.
;
; Note that the TR4K-IDE uses 8 character tabs.
; TR4P153CT = new hardware (emulator, otp chips)
;     as TR4P153BT but supports ADC
; TR4P153BT = new hardware (grenade)
;     14 pins (11 input/output, 1 input, vcc, gnd)
;     2048 instructions
;     256 nybbles of RAM
;     2.2V to 5.5V
; 
; PINOUT (*=common to both chips)
; This involved switching PA3 and PB2 from the schematic 27jan2017 since PA3 is input only
; 1 PD1  PWR_EN        14 PD2  EN_LED4
; 2 PD0  PWR_BTN (low) 13 PD3  EN_LED3
; 3 PB2  LVL_1         12 PB3  EN_LED2
; 4 VDD*               11 VSS*
; 5 PB1* LVL_3         10 PA0* EN_LED1
; 6 PB0* LVL_2          9 PA1* MOD_OUT
; 7 PA3* (input only)   8 PA2* LED (visible) (active low)
; 
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
; 
; Resource usage
;   Green mode (low speed oscillator) is not used - just normal and halt
;   Internal high speed oscillator is 32MHz +/- 2%
;   CPU clock is osc/4 = 8MHz
;   Scaler2 = MCLK/8 = 1MHz
;   Timer1 = Unused
;   Timer2 = 80uS for protocol output
;   RTC = Unused
; 
; GPIO usage
;   PA0 = IR power 1 (output)
;   PA1 = 38kHz infrared (output) (high = LED on)
;   PA2 = unused
;   PA3 = unused
;   PB0 = Visible LED1 (output)
;   PB1 = Visible LED2 (output)
;   PB2 = Visible LED3 (output)
;   PB3 = IR power 2 (output)
;   PD0 = Power (output)
;   PD1 = User (input)
;   PD2 = unused
;   PD3 = IR power 3
; 
; *****************************************************************************
; Timings
;   Vishay have said that the TSOP4438 requires 35ms to cool down between packets
;     So we tried sending packets once every 53ms (18Hz)
; Group patterns
;   Sets are sent continuously, consisting of
;       20 fast packets sent at 30ms intervals (see GroupNumFast and Tim_SendPkt_F)
;        2 slow packets sent at 100ms intervals (see GroupNumSlow and Tim_SendPkt_S)
;       
; *****************************************************************************
; Protocol definitions
;   True NEC protocol
;       9mS header, 4.5mS header gap, 32 bits of 562.5uS tick, stop bit
;       Payload bits: one = "10", Zero = "1000" in ticks.
;   This NEC protocol (original)
;       9mS header, 4.5mS header gap, 32 bits of 550uS tick, stop bit
;       Payload bits: one = "10", Zero = "1000" in ticks.
;   Manchester protocol
;       4.5mS header, 2.25mS header gap, start bit, 32 bits of 320uS tick, stop bit
;       Payload bits: one = "10", Zero = "01" in ticks.
; *****************************************************************************

; -----------------------------------------------------------------------------
; Include Block
; -----------------------------------------------------------------------------
#define USE_FIXED_SERIAL 1
#define SUPPORT_PORTD 1 ; defined here since it does not seem to work when it is defined within an include file
#include "tr4p153ct.inc" ; Emulator (same as grenade)
;#include "tr4p153bt.inc" ;Grenade (same as emulator)
;#include "tr4p151af.inc" ; Beacon (lacks portD)

; -----------------------------------------------------------------------------
; Primary choice: Choose the protocol to transmit
;#define PROTOCOL_NEC32 ; (Original) NEC protocol with 32 bit payload (16 bit id) and full length header [WORKS]
;#define PROTOCOL_NEC20 ; NEC protocol with 20 bit payload [MAYBE SUPPORTED]
;#define PROTOCOL_MAN20 ; Manchester protocol with 20 bit payload [WORKS]
;#define PROTOCOL_MAN16 ; Manchester protocol with 16 bit payload (pure serial number) [MAYBE SUPPORTED]
;#define PROTOCOL_NEC9  ; NEC protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
;#define PROTOCOL_MAN9  ; Manchester protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
#define PROTOCOL_MAN20A ; Manchester protocol with 20 bit payload [MAYBE SUPPORTED]

; -----------------------------------------------------------------------------
; Supposedly secondary (generated) parameters for the Infrared protocol
; Sadly the Tritan assembler does NOT process #define within a #ifdef correctly
; It defines the constant even if the control block is false!
; So the programmer needs to manually comment out the false ones
#ifdef PROTOCOL_NEC32
;#define PROTOCOL_NEC
;#define PROTOCOL_32
#endif

#ifdef PROTOCOL_NEC20
;#define PROTOCOL_NEC
;#define PROTOCOL_20
#endif

#ifdef PROTOCOL_MAN20
;#define PROTOCOL_MAN
;#define PROTOCOL_20
#endif

#ifdef PROTOCOL_MAN20A
#define PROTOCOL_MAN
#define PROTOCOL_20A
#endif

#ifdef PROTOCOL_MAN16
;#define PROTOCOL_MAN
;#define PROTOCOL_16
#endif

#ifdef PROTOCOL_NEC9
;#define PROTOCOL_NEC
;#define PROTOCOL_9
#endif

#ifdef PROTOCOL_MAN9
;#define PROTOCOL_MAN
;#define PROTOCOL_9
#endif
; -----------------------------------------------------------------------------

; grenade logic equates
state_unarmed	equ 0
state_explode	equ 1
state_1	equ 2
state_2	equ 3
state_3	equ 4
state_4	equ 5
state_5	equ 6
state_6	equ 7
state_7	equ 8
state_8	equ 9
state_9	equ 10
state_10	equ 12

; -----------------------------------------------------------------------------
; User defined "registers" (SRAM variables)
; -----------------------------------------------------------------------------
;#define Optimal_MAH		; System MAH Optimal (Only optimal MAH)
#define Optimal_MAH_PCH		; System MAH + PCH Optimal (Optimal MAH and PCH)
VARRM = {
; -------------------------------------
; System variables
A_SRT          ; Save the A register during an interrupt
SramChk0_IN    ; Canary byte for checking SRAM corruption
SramChk1_IN    ; ^

Mcu_ID0        ; 16 bit unique ID read from OTP flash (set on startup)
Mcu_ID1        ; ^
Mcu_ID2        ; ^
Mcu_ID3        ; ^

; -------------------------------------
; Infrared packet
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

IR_Group0      ; 8 bit counter (incremented every packet) for checking fast vs slow packets
IR_Group1      ; ^  For main thread

IR_Start_Flag  ; NonZero = Do not touch infrared
IR_Flag        ; Bit0=header pulse has been sent. Bit1=header gap has been sent, Bit2=payload has been sent, Bit3=stop bit has been sent

IR_Num0        ; 8 bits: the number of payload bits that have been sent in this packet
IR_Num1        ; ^ For interrupt thread

IR_BaseTim0    ; 8 bit counter incremented every timer2 (80uS) "Do basic time waveform used"
IR_BaseTim1    ; ^ this is reset between phases (e.g. header, gap, each payload bit)
IR_BIT_OK_FLAG ; True iff IR_Bit is valid. False if a new payload bit should be extracted from the packet.
IR_Bit         ; The value of the current transmitting payload bit (0 or 1)
LastTim1       ; Used for checking that multi nybble comparisons have not been invalidated
; (would be a bank boundary if this was handled manually)
Tim_SleepCount0    ; 16 bit timer counter for sleep mode. Counts up on every packet sent.
Tim_SleepCount1    ; ^ for main thread
Tim_SleepCount2    ; ^
Tim_SleepCount3    ; ^

; CRC variables (second bank)
CRC_DATA0      ; Calculated 16 bit CRC input buffer
CRC_DATA1      ; ^
CRC_DATA2      ; ^
CRC_DATA3      ; ^

; Grenade logic variables
g_timer0
g_timer1
g_timer2
g_timer3

g_timer_last0
g_timer_last1
g_timer_last2
g_timer_last3

g_state
g_substate


}

; This is cached to reduce interrupt jitter on the infrared output enable
IR_OnOff	equ USER1 ; set nonzero for on (copy to RTC register at next timer2 interrupt)

; -----------------------------------------------------------------------------
; General Parameters for the program
Ram_chk_Dat     equ 5AH   ; Canary value for SRAM sanity checking
VINT_MAH        equ 00    ; The SRAM bank we want to use inside the interrupt [Mind you, Optima overrides this]

; Parameters for the Infrared protocol - speed choice
Tim2_Speed	equ 256-80 ; 80uS timer2 interrupt
Tim_SendPkt_F   equ 375    ; (12 bits in 40uS units) 30ms base timing for sending fast packets
Tim_SendPkt_S   equ 1250   ; (12 bits in 40uS units) 100ms base timing for sending slow packets
TIME_SLEEP      equ 60000  ; (16 bits in 30ms units) Timeout for sleep mode, in units of packets sent
;TIME_SLEEP      equ 1000  ; (16 bits in 30ms units) Timeout for sleep mode, in units of packets sent

; -----------------------------------------------------------------------------
; Correctly generated parameters for the Infrared protocol
#ifdef PROTOCOL_NEC32
IR_BitNum_Dat   equ 32   ; Number of payload bits per packet
#endif

#ifdef PROTOCOL_NEC20
IR_BitNum_Dat   equ 20   ; ^
#endif

#ifdef PROTOCOL_MAN20
IR_BitNum_Dat   equ 20   ; ^
#endif

#ifdef PROTOCOL_MAN20A
IR_BitNum_Dat   equ 20   ; ^
#endif

#ifdef PROTOCOL_MAN16
IR_BitNum_Dat   equ 16   ; ^
#endif

#ifdef PROTOCOL_NEC9
IR_BitNum_Dat   equ 9    ; ^
#endif

#ifdef PROTOCOL_MAN9
IR_BitNum_Dat   equ 9    ; ^
#endif

; -----------------------------------------------------------------------------

; Payload bit coding parameters
IR_01_Dat       equ 4    ; For zero payload bits in NEC coding, the time spent high (in 80uS ticks)
IR_00_Dat       equ 8	 ; For zero payload bits in NEC coding, the time spent high+low (in 80uS ticks)
IR_11_Dat       equ 4    ; For one payload bits in NEC coding, the time spent high (in 80uS ticks)
#ifdef PROTOCOL_MAN
IR_10_Dat       equ 8   ; For one payload bits in Manchester coding, the time spent high+low (in 80uS ticks)
#else
IR_10_Dat       equ 16   ; For one payload bits in NEC coding, the time spent high+low (in 80uS ticks)
#endif

; NEC 32 parameter
User_Dat_2Byte  equ 0F0H ; The 8 bit fixed portion of the 32 bit packet format

; Parameters for the start bit
#ifdef PROTOCOL_NEC32
Tim_9ms         equ 90  ; The timer for 9ms header pulse
tim_45ms        equ 45  ; 4.5ms header gap
#else
Tim_9ms         equ 32  ; Timer for 2.56ms header pulse (in 80uS units)
tim_45ms        equ 16  ; Timer for 1.28ms header gap in 80uS units
#endif
#ifdef PROTOCOL_MAN
tim_gapms	equ tim_45ms+IR_01_Dat ; Gap (including start bit)
#else
tim_gapms	equ tim_45ms-1 ; Gap (no start bit)
#endif

; Parameters for the stop bit
IR_Last_Tim     equ 4   ; Timer for stop bit (in 80uS units)

; Parameters for the CRC calculation
GenPoly         equ 07H  ; Generator Polynomial for the CRC
C_InitVal	equ 0    ; CRC initial value
CRC_DataCnt     equ 16   ; Number of bits in the CRC (1..16 valid)

; Parameters for the group calculation
GroupNumFast	equ 30   ; Number of fast packets
GroupNumSlow	equ 2    ; Number of slow packets
; Generated parameters
GroupNumTotal	equ GroupNumSlow+GroupNumFast


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

	; (8) Entry point for interrupt start
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

	; Are we sending IR data?
	ld	a,(IR_Start_Flag)
	jnz	INT_End

	; Where are we in the transmission?
	inc	(IR_BaseTim0) ; Increment 8 bit timer
	adr	(IR_BaseTim1)
	
	; CPM: output high or low here at fixed location instead of all over the IRQ routine causing jitter
	ld	a,(IR_OnOff)
	jz	IrIsOff
	set	#2,(RTC) ; PA1 output infrared beam
	jmp	IrIsDone
IrIsOff:
	clr	#2,(RTC) ; PA1 no output infrared beam
IrIsDone:
	
	ld	a,(IR_Flag)
	and	a,#0001b
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
	or	a,#0001b ; Flag the header pulse as having been sent
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
	and	a,#0010b
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
	or	a,#0010b
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
	and	A,#0100B
	jnz	DATA_RX_OK_Prc

	; ====================================
	; Send the payload bits (originally 32 bits; now can also be 20, 16 or 9 bits)
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
	; Completed transmitting a zero bit
	ld	a,#0
	ld	(IR_BaseTim0),a
	ld	(IR_BaseTim1),a
	ld	(IR_BIT_OK_FLAG),a
	; New payload bit
	inc	(IR_Num0)
	adr	(IR_Num1)
	jmp	INT_End

; Have transmitted all payload bits
RX_Data_32Bit_OK_Prc:
	ld	a,#0
	ld	(IR_Num0),a
	ld	(IR_Num1),a

	ld	a,(IR_Flag)
	or	a,#0100B
	ld	(IR_Flag),a
	jmp	INT_End 

DATA_RX_OK_Prc:
	; ====================================
	; Transmit the stop bit (all payload has been sent)
	ld	a,(IR_Flag)
	and	a,#1000B
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
	or	a,#1000B	; If not already true
	ld	(IR_Flag),a
	jmp	INT_End

; The complete packet has been sent
Ir_Last_data_Rx_Ok_Prc:
	ld	a,(IR_Flag)
	cmp	a,#0fh
	jnz	INT_End 

	clr	#2,(IR_OnOff) ; PA1 no output infrared beam
	clr	#2,(RTC) ; PA1 no output infrared beam
	ld	A,#01
	ld	(IR_Start_Flag),A

; End of interrupt for timer2
INT_End:
	inc	(Tim_SendPkt1) ; Increment the 12 bit timer counter
	adr	(Tim_SendPkt2)
	adr	(Tim_SendPkt3)
; End of interrupt for timer1, timer2, RTC
INT_End_NoInc:
	ld	a,(A_SRT) ; Restore the A register
	reti

; -----------------------------------------------------------------------------
; MAIN PROGRAM entry point
PGMSRT:
ERR_PC_ADR:
PGM_Delay:
	ld	A,#0
	ld	(SYS0),A	; Disable interrupts
	ld	(USER1),A	; This nybble could be used by user code (and is)
	ld	(USER2),A	; This nybble could be used by user code (but isn't)
	; Initialise input/output 
	ldpch	PODY_IO_Init
	call	PODY_IO_Init
	
	; Power-up

	; Delay loop (uses SRAM before it is cleared)
	; Should be 11*0xD0000/2 = about 0.585 seconds
	ldmah	#0
	ld	A,#0
	ld	(20H),A
	ld	(21H),A
	ld	(22H),A
	ld	(23H),A
	ld	A,#0DH
	ld	(24H),A ; 20 bit timer 0xD0000
DELAY1:
	ld	A,#05H
	ld	(WDT),A ; Kick the watchdog

	ld	A,(20H)
	clr	C
	adc	A,#2
	ld	(20H),A
	adr	(21H)
	adr	(22H)
	adr	(23H)
	adr	(24H)
	jnc	DELAY1

	; Initialise input/output again
	ldpch	PODY_IO_Init
	call	PODY_IO_Init

	; Clear Banks 0..3 of SRAM
	ldmah	#3
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#2
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#1
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#0
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	; Setup timer2 interrupt every 40 / 50uS
	ldmah	#0
	ldpch	Timer2_Init
	call	Timer2_Init

	; Read the unique ID from OTP
	ldpch	Read_Mcu_ID
	Call	Read_Mcu_ID

	; Create the CRC for the data packet
	ldpch	CRC_Chk_Code
	call	CRC_Chk_Code
	; Setup a packet
	ldpch	Data_Int_Code
	call	Data_Int_Code

	; Set the canary value for checking SRAM has not been modified
	ld	A,#Ram_chk_Dat.N0
	ld	(SramChk0_IN),A
	ld	A,#Ram_chk_Dat.N1
	ld	(SramChk1_IN),A 

	; Set the group counter to be the start value
	ld	A,#0
	ld	(IR_Group0),A
	ld	(IR_Group1),A

; -----------------------------------------------------------------------------
; Where wakeup code would return to
WakeUp:
	nop
	nop
	; We want the RTC interrupt to be every 1s instead of 125ms to avoid disruption
	set	#1,(RTC)
	set	#0,(RTC)

; -----------------------------------------------------------------------------
; Main loop for background tasks
; The call stack is too short to implement routines but these are effectively subroutines
MAIN_LOOP:
	; Task 1 - system sanity check
	ldpch	SYS_Check_Prc
	jmp	SYS_Check_Prc
SYS_Check_RP:	

	; Task 2 - check for sending packets
	ldpch	Tim_SendPkt_Chk_Prc
	jmp	Tim_SendPkt_Chk_Prc
Tim_SendPkt_Chk_RP: 

	; Task 3 - check for sleep
	ldpch	Chk_Halt_Tim_Prc
	jmp	Chk_Halt_Tim_Prc
Chk_Halt_Tim_RP:

	nop
	nop
	nop
	nop
	ldpch	MAIN_LOOP
	jmp	MAIN_LOOP
	
	
; -----------------------------------------------------------------------------
; Main loop task 1/3
; System sanity check
SYS_Check_Prc:
	; Setup i/o ports (relevant if we have woken up recently)
	ldmah	#0

	; Set ports for input/output
	ld	a,#0111b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1111b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
#ifdef SUPPORT_PORTD
	ld	a,#1110b
	ld	(IOC_PD),a	; Port D dir (0=input/1=output)
#endif

	; Kick the watchdog
	ld	a,#05h
	ld	(WDT),a
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid
	
	; Check the SRAM canary to see if it has been corrupted
	ld	a,#Ram_chk_Dat.n0
	cmp	a,(SramChk0_IN)
	; CPM: will there be an extra ldpch PGMSRT here? (It is controlled by Optima)
	jnz	PGMSRT
	ld	a,#Ram_chk_Dat.n1
	cmp	a,(SramChk1_IN)
	ldpch	PGMSRT
	jnz	PGMSRT

	; Return to the next task in the main loop
	ldpch	SYS_Check_RP
	jmp	SYS_Check_RP

; -----------------------------------------------------------------------------
; Main loop task 2/3
; Check for the packet trigger events (30ms / 100ms), i.e. that a new packet should be triggered
Tim_SendPkt_Chk_Prc:
	; Are we in fast packet moon or slow packet mode
	ld	a,(IR_Group0)
	cmp	a,#GroupNumFast.n0
	ld	a,(IR_Group1)
	sbc	a,#GroupNumFast.n1
	jc	SendPkt_TestFast

	; Test timing for slow packets
	; Note that an interrupt here can cause the counter to be inconsistent
	ld	a,(Tim_SendPkt1)
	ld	(LastTim1),a	; Because Tim_SendPkt1 is volatile!
	cmp	a,#Tim_SendPkt_S.n0
	ld	a,(Tim_SendPkt2)
	sbc	a,#Tim_SendPkt_S.n1
	ld	a,(Tim_SendPkt3)
	sbc	a,#Tim_SendPkt_S.n2
	
	ldpch	Tim_SendPkt_Chk_RP 
	jc	Tim_SendPkt_Chk_RP
	jmp SendPkt_Trigger

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
	ldpch	Tim_SendPkt_Chk_RP 
	jc	Tim_SendPkt_Chk_RP

SendPkt_Trigger:
	; Sanity check - has the bottom nybble of the 12 bit number changed during the comparison, invalidating it?
	ld	a,(Tim_SendPkt1)
	cmp	a,(LastTim1)
	jnz	Tim_SendPkt_Chk_Prc	; Redo the test! If the number went from 1FF to 200 during the test (because interrupt) we might think it was 2FF!
	
	; We can start triggering a new packet - stop current output
	ld	a,#1 ; Disable messing with IR
	ld	(IR_Start_Flag),a ; Should already be set though
	
	; Increment packet count within the group
	inc	(IR_Group0)
	adr	(IR_Group1)
	; Have we finished the set of fast and slow packets?
	ld	a,(IR_Group0)
	cmp	a,#GroupNumTotal.n0
	ld	a,(IR_Group1)
	sbc	a,#GroupNumTotal.n1
	jc	SendPkt_NoWrap
	; Start the next set of packets
	ld	a,#0
	ld	(IR_Group0),a
	ld	(IR_Group1),a
SendPkt_NoWrap:
	
	; Count up for sleep mode
	inc	(Tim_SleepCount0)
	adr	(Tim_SleepCount1)
	adr	(Tim_SleepCount2)
	adr	(Tim_SleepCount3)
	ldpch	Data_Int_Code
	call	Data_Int_Code

	; Now it is safe to trigger the new packet
	; Clear interrupts before resetting packet count to make it atomic
	clr	#1,(SYS0) ; Clear ENINT, disabling all interrupts
	nop	; Safety (recommended after clearing ENINT)
	ld	a,#0
	ld	(Tim_SendPkt1),a
	ld	(Tim_SendPkt2),a
	ld	(Tim_SendPkt3),a
	ld	(IR_Start_Flag),a ; IR enabled
	ld	a,#0
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid

	; Return to the next task in the main loop
	ldpch	Tim_SendPkt_Chk_RP
	jmp	Tim_SendPkt_Chk_RP

; -----------------------------------------------------------------------------
; Main loop task 3/3
; Check for going to sleep
Chk_Halt_Tim_Prc:
	; Compare the 16 bit sleep mode counter with sleep
	clr	c
	ld	a,(Tim_SleepCount0)
	cmp	a,#TIME_SLEEP.n0
	ld	a,(Tim_SleepCount1)
	sbc	a,#TIME_SLEEP.n1
	ld	a,(Tim_SleepCount2)
	sbc	a,#TIME_SLEEP.n2
	ld	a,(Tim_SleepCount3)
	sbc	a,#TIME_SLEEP.n3
	ldpch	Chk_Halt_Tim_End
	jc	Chk_Halt_Tim_End
	; Reset the sleep mode counter
	ld	a,#0
	ld	(Tim_SleepCount0),a
	ld	(Tim_SleepCount1),a
	ld	(Tim_SleepCount2),a
	ld	(Tim_SleepCount3),a

	; Turn off the visible LED and infrared LED
	clr	#2,(RTC)        ; PA1 no output infrared beam
	ld	a,#1000b
	ld	(data_pa),a	; Port A0,1,2,3=low
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)
#ifdef SUPPORT_PORTD
	ld	a,#0011b
	ld	(data_pd),a	; Port D data (0=low/1=high)
#endif

	clr	#1,(SYS0) ; Clear ENINT and disable interrupts
	nop
	nop
	halt 
Chk_Halt_Tim_End:
	ldpch	Chk_Halt_Tim_RP
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
	ld	a,(Mcu_ID0)
	ld	(CRC_DATA0),a
	ld	a,(Mcu_ID1)
	ld	(CRC_DATA1),a
	ld	a,(Mcu_ID2)
	ld	(CRC_DATA2),a
	ld	a,(Mcu_ID3)
#ifdef PROTOCOL_20
	and	a,#1	; 13 bits instead of 16
#endif
	ld	(CRC_DATA3),a

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
	ld	a,(Mcu_ID0)
	ld	(IR_PktData2),A
	ld	a,(Mcu_ID1)
	ld	(IR_PktData3),A
	ld	a,(Mcu_ID2)
	ld	(IR_PktData4),a
	ld	a,(Mcu_ID3)
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
	ld	a,(Mcu_ID0)
	ld	(IR_PktData0),a
	ld	a,(Mcu_ID1)
	ld	(IR_PktData1),a
	ld	a,(Mcu_ID2)
	ld	(IR_PktData2),A
	ld	a,(Mcu_ID3)
	and	a,#1
	or	a,(IR_PktData3)
	ld	(IR_PktData3),A
#endif
#ifdef PROTOCOL_20A
	; 16 bits serial number+1 bit beacon+3 bits crc
	ld	a,(Mcu_ID0)
	ld	(IR_PktData0),a
	ld	a,(Mcu_ID1)
	ld	(IR_PktData1),a
	ld	a,(Mcu_ID2)
	ld	(IR_PktData2),A
	ld	a,(Mcu_ID3)
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
	ld	a,(Mcu_ID0)
	ld	(IR_PktData0),a
	ld	a,(Mcu_ID1)
	ld	(IR_PktData1),a
	ld	a,(Mcu_ID2)
	ld	(IR_PktData2),A
	ld	a,(Mcu_ID3)
	ld	(IR_PktData3),A
	ld	a,#0
	ld	(IR_PktData4),a
	ld	(IR_PktData5),a
	ld	(IR_PktData6),a
	ld	(IR_PktData7),a
#endif
#ifdef PROTOCOL_9
	ld	a,(Mcu_ID0)
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
; IO port initialization settings (e.g. pullup/pulldown/wakeup)
PODY_IO_Init:
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
#ifdef SUPPORT_PORTD
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
; Set up timer 2 for 40/50uS interrupt (25/20kHz) +/- 2%
; High speed oscillator HRCOSC = 32Mhz
; CPU clock FMCK = /4 = 8Mhz
; Scaler2 = Div8 = 1MHz
; Timer2 = 0xCE = 20kHz (or 0xD8 = 25kHz)
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
Grenade_init_logic:
	ld a,#0
	ld (g_timer0),a
	ld (g_timer1),a
	ld (g_timer2),a
	ld (g_timer3),a
	ld (g_substate),a
	ld (g_state),a
	rets

; ----------------------------------------------------------------------------
Grenade_arm:
	ld a,#state_10
	ld (g_state),A
	ld a,#state_10
	ld (g_state),a
	ld a,#0
	ld (g_timer0),a
	ld (g_timer1),a
	ld (g_timer2),a
	ld (g_timer3),a
	ld (g_substate),a
	rets

; ----------------------------------------------------------------------------
; Code to update the grenade visible LED (PA2)
Grenade_update_visible:
	ld a,(g_state)
	cmp a,#state_unarmed
	jz gvis_off
	cmp a,#state_explode
	jnz gvis_notexp
	; Explosion
	ld a,(g_substate)
	and a,#1
	jz gvis_off
	jmp gvis_on

gvis_notexp:
	; Countdown
	ld a,(g_substate)
	cmp a,#0
	jz gvis_off

gvis_on:
	clr #2,(DATA_PA)
	rets

gvis_off:
	set #2,(DATA_PA)
	rets


; ----------------------------------------------------------------------------
Grenade_update_logic:
	ld a,(g_state)
	cmp a,#state_unarmed
	jnz gul_notunarmed
	ld a,(DATA_PD)
	and a,#1 ; PD0 = user button
	jz Grenade_Arm
	rets

gul_notunarmed:
	rets

#if 0
	; Increment g_timer according to timer
	uint16_t nTimeLimit = 1428; // 100ms
		if (grenade.state == state_explode)
		{
			nTimeLimit = 1000; // 70ms
		}
		if (grenade.timer >= nTimeLimit)
		{
			grenade.timer -= nTimeLimit;
			grenade.substate++;


			uint16_t serial;
			switch (grenade.state) {
			case state_explode:
				// 10 times per second send the explosion event
				serial = 0xCB00U;
				IRsend_queue_beacon(serial);
				if (grenade.substate == 1)
				{
					UART2_Print("# Grenade exploded\r\n");
				}
				if (grenade.substate >= 20) // 1400ms explosion
				{
					grenade.state = state_unarmed;
					grenade.substate = 0;
					SleepPart1();
				}
				break;
			case state_1:
			case state_2:
			case state_3:
			case state_4:
			case state_5:
			case state_6:
			case state_7:
			case state_8:
			case state_9:
			case state_10:
				if ((grenade.substate & 1) != 0) // 5 times per second
				{
					serial = 0xCB01U + grenade.state-state_1;
					IRsend_queue_beacon(serial);
				}
				if (grenade.substate >= 10) // 1000ms in each tick
				{
					UART2_Print("# Grenade tick\r\n");
					grenade.state--;
					grenade.substate = 0;
				}
				break;
			case state_unarmed:
			default:
				break;
			}
		}
	}

#endif
