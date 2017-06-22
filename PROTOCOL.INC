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
;   Sets are sent once per second, consisting of
;       Warning sets (countdown to explosion)
;          16 fast packets sent at 30ms intervals (see WarningNumFast and Tim_SendPkt_F)
;       Explosion sets sent at
;          24 fast packets sent at 30ms intervals (see ExplodeNumFast and Tim_SendPkt_F)
;           8 slow packets sent at 100ms intervals (see ExplodeNumSlow and Tim_SendPkt_S)
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
;       3.2mS header, 1.6mS header gap, start bit, 32 bits of 400uS tick, stop bit
;       Payload bits: one = "10", Zero = "01" in ticks.
;   Grenade NEC protocol
;       2.4mS header, 2.4mS header gap, 4 or 12 bits of 600uS tick, stop bit
;       Payload bits: zero = "10", one = "100" in ticks. (shorter)
; *****************************************************************************

; -----------------------------------------------------------------------------
; Primary choice: Choose the protocol to transmit
;#define PROTOCOL_NEC32 ; (Original) NEC protocol with 32 bit payload (16 bit id) and full length header [WORKS]
;#define PROTOCOL_NEC20 ; NEC protocol with 20 bit payload [MAYBE SUPPORTED]
;#define PROTOCOL_MAN20 ; Manchester protocol with 20 bit payload [WORKS]
;#define PROTOCOL_MAN16 ; Manchester protocol with 16 bit payload (pure serial number) [MAYBE SUPPORTED]
;#define PROTOCOL_NEC9  ; NEC protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
;#define PROTOCOL_MAN9  ; Manchester protocol with 9 bit payload (4 bit id) [NOT SUPPORTED]
;;;#define PROTOCOL_NEC4   ; NEC protocol with 4 bit payload (4 bit id) [SUPPORTED] ; moved to grenade.asm
;;;#define PROTOCOL_NEC12  ; NEC protocol with 8 bit payload (4 bit id) [SUPPORTED] ; moved to grenade.asm
;;;#define PROTOCOL_MAN20A ; Manchester protocol with 20 bit payload [SUPPORTED] ; moved to grenade.asm

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

#ifdef PROTOCOL_NEC12
;#define PROTOCOL_NEC
;#define PROTOCOL_12
#endif

#ifdef PROTOCOL_NEC4
;#define PROTOCOL_NEC
;#define PROTOCOL_4
#endif

; -----------------------------------------------------------------------------
; Parameters for the Infrared protocol - speed choice
Tim2_Speed	equ 256-100  ; 100uS timer2 interrupt (so 1ms=20/2 interrupts)
#ifdef PROTOCOL_NEC12
Tim_SendPkt_F   equ 40*20/2  ; (12 bits in 50uS units) 40ms base timing for sending fast packets (since NEC8 packets could take 34ms)
#else
Tim_SendPkt_F   equ 30*20/2  ; (12 bits in 50uS units) 30ms base timing for sending fast packets
#endif
Tim_SendPkt_S   equ 100*20/2 ; (12 bits in 50uS units) 100ms base timing for sending slow packets
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

#ifdef PROTOCOL_NEC12
IR_BitNum_Dat   equ 12    ; ^
#endif

#ifdef PROTOCOL_NEC4
IR_BitNum_Dat   equ 4    ; ^
#endif

; -----------------------------------------------------------------------------

; Payload bit coding parameters
#ifdef PROTOCOL_NEC4
IR_01_Dat       equ 6    ; For zero payload bits in NEC coding, the time spent high (in 100uS ticks)
IR_00_Dat       equ 12	 ; For zero payload bits in NEC coding, the time spent high+low (in 100uS ticks)
IR_11_Dat       equ 6    ; For one payload bits in NEC coding, the time spent high (in 100uS ticks)
IR_10_Dat       equ 18   ; For one payload bits in NEC coding, the time spent high+low (in 100uS ticks)
#else
#ifdef PROTOCOL_NEC12
IR_01_Dat       equ 6    ; For zero payload bits in NEC coding, the time spent high (in 100uS ticks)
IR_00_Dat       equ 12	 ; For zero payload bits in NEC coding, the time spent high+low (in 100uS ticks)
IR_11_Dat       equ 6    ; For one payload bits in NEC coding, the time spent high (in 100uS ticks)
IR_10_Dat       equ 18   ; For one payload bits in NEC coding, the time spent high+low (in 100uS ticks)
#else
IR_01_Dat       equ 4    ; For zero payload bits in NEC coding, the time spent high (in 100uS ticks)
IR_00_Dat       equ 8	 ; For zero payload bits in NEC coding, the time spent high+low (in 100uS ticks)
IR_11_Dat       equ 4    ; For one payload bits in NEC coding, the time spent high (in 100uS ticks)
#ifdef PROTOCOL_MAN
IR_10_Dat       equ 8   ; For one payload bits in Manchester coding, the time spent high+low (in 100uS ticks)
#else
IR_10_Dat       equ 16   ; For one payload bits in NEC coding, the time spent high+low (in 100uS ticks)
#endif
#endif
#endif

; NEC 32 parameter
User_Dat_2Byte  equ 0F0H ; The 8 bit fixed portion of the 32 bit packet format

; Parameters for the start bit
#ifdef PROTOCOL_NEC32
Tim_9ms         equ 90  ; The timer for 9ms header pulse
tim_45ms        equ 45  ; 4.5ms header gap
#endif
#ifdef PROTOCOL_NEC12
Tim_9ms         equ 24  ; Timer for 2.4ms header pulse (in 100uS units)
tim_45ms        equ 24  ; Timer for 2.4ms header gap in 100uS units
#endif
#ifdef PROTOCOL_NEC4
Tim_9ms         equ 24  ; Timer for 2.4ms header pulse (in 100uS units)
tim_45ms        equ 24  ; Timer for 2.4ms header gap in 100uS units
#endif
#ifdef PROTOCOL_MAN
Tim_9ms         equ 32  ; Timer for 3.2ms header pulse (in 100uS units)
tim_45ms        equ 16  ; Timer for 1.6ms header gap in 100uS units
#endif
tim_explode_pulse	equ	48	; The timer for 4.8ms header pulse for explosions

#ifdef PROTOCOL_MAN
tim_gapms	equ tim_45ms+IR_01_Dat ; Gap (including start bit)
#else
tim_gapms	equ tim_45ms-1 ; Gap (no start bit)
#endif

; Parameters for the stop bit
#ifdef PROTOCOL_NEC12
IR_Last_Tim     equ 6   ; Timer for stop bit (in 100uS units)
#endif
#ifdef PROTOCOL_NEC4
IR_Last_Tim     equ 6   ; Timer for stop bit (in 100uS units)
#endif
#ifdef PROTOCOL_MAN
IR_Last_Tim     equ 4   ; Timer for stop bit (in 100uS units)
#endif

; Parameters for the CRC calculation
GenPoly         equ 07H  ; Generator Polynomial for the CRC
C_InitVal	equ 0    ; CRC initial value
CRC_DataCnt     equ 16   ; Number of bits in the CRC (1..16 valid)

; Parameters for the warning infrared packets
WarningNumFast	equ 16   ; Number of fast packets
WarningNumSlow	equ 0    ; Number of slow packets
; Generated parameters
WarningNumTotal	equ WarningNumSlow+WarningNumFast+1

; Parameters for the explode infrared packets
ExplodeNumFast	equ 24   ; Number of fast packets
ExplodeNumSlow	equ 8    ; Number of slow packets
; Generated parameters
ExplodeNumTotal	equ ExplodeNumSlow+ExplodeNumFast+1

