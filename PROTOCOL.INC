; *****************************************************************************
; Recoil Gun Grenade code for SkyRocket Toys 2017
; Infrared protocol
; *****************************************************************************
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
; Primary choice: Choose the protocol to transmit
;#define PROTOCOL_NEC32 ; (Original) NEC protocol with 32 bit payload (16 bit id) and full length header [WORKS]
;#define PROTOCOL_NEC20 ; NEC protocol with 20 bit payload [MAYBE SUPPORTED]
;#define PROTOCOL_MAN20 ; Manchester protocol with 20 bit payload [WORKS]
;#define PROTOCOL_MAN16 ; Manchester protocol with 16 bit payload (pure serial number) [MAYBE SUPPORTED]
;#define PROTOCOL_NEC9  ; NEC protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
;#define PROTOCOL_MAN9  ; Manchester protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
;;;#define PROTOCOL_MAN20A ; Manchester protocol with 20 bit payload [SUPPORTED]	; moved to grenade.asm

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
;;;#define PROTOCOL_MAN	; moved to grenade.asm
;;;#define PROTOCOL_20A	; moved to grenade.asm
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
; Parameters for the Infrared protocol - speed choice
Tim2_Speed	equ 256-80   ; 80uS timer2 interrupt (so 1ms=25/2 interrupts)
Tim_SendPkt_F   equ 30*25/2  ; (12 bits in 40uS units) 30ms base timing for sending fast packets
Tim_SendPkt_S   equ 100*25/2 ; (12 bits in 40uS units) 100ms base timing for sending slow packets
TIME_SLEEP      equ 60000    ; (16 bits in 30ms units) Timeout for sleep mode, in units of packets sent
;TIME_SLEEP      equ 1000    ; (16 bits in 30ms units) Timeout for sleep mode, in units of packets sent

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

