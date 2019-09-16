; File:  DMX512RecDemo.asm 
; DMX512 Receiver 
; This file uses a PIC18F2420 device to receive DMX-512 data and store it 
; into a 512 byte receive buffer.   
; For demonstration purposes, a selected data slot is written to the  
; CCP module.  The CCP module is configured in PWM mode and the received 
; data adjusts the duty cycle.  If a resistor and LED is connected to the 
; PWM output, the received DMX data can be visually observed.  
 
	list p=16f1827
	#include <P16F1827.inc>

;Configuration bits setup	 

	__CONFIG    _CONFIG1, _FCMEN_OFF & _IESO_OFF & _CLKOUTEN_OFF & _BOREN_OFF & _CPD_OFF & _CP_OFF & _MCLRE_OFF & _PWRTE_ON & _WDTE_OFF & _FOSC_HS
	__CONFIG    _CONFIG2, _LVP_OFF & _STVREN_OFF & _PLLEN_OFF & _WRT_OFF & _BORV_19

	 
; Constants 
#define CHANNEL .1            ; select the receiver slot/channel 

#define ArrayP1	0x20
#define ArrayP2 0x00+.3
 
; Variables 
	cblock h'20'
	    CountH                  ; 16-bit counter 
        CountL 
;	    RxBuffer: .512          ; 512 bytes buffer allocation 
		PWMR
		PWMG
		PWMB
		ColorOSC
		Color	 
		CurRX
		counta
		countb
		count1
	ENDC 
	 
;****************************************************************************** 
	ORG 	0x0 
 
Main 
	call	SetupSerial			;Setup Serial port and buffers 
 
MainLoop 
	 
; first loop, synchronizing with the transmitter 
WaitBreak 
;	call	InitBlink

	banksel PIR1
	btfss	PIR1,TXIF
	bra		SkipTX

	banksel	TXREG
	movlw	b'00000010'
	movwf	TXREG

SkipTX
	banksel	PIR1
	btfss	PIR1,RCIF           ; if a byte is received incorrectly 
	goto	WBElse				; skip

WBDisc
	banksel	RCREG
	movf	RCREG,W             ; if received correctly discard it 
	banksel	CurRX
	movwf	CurRX

	movlw	d'0'
	xorwf	CurRX
	btfsc	STATUS,Z	; if 0
	goto	WaitBreak	


WBElse
	banksel	RCSTA
	btfss	RCSTA,FERR			; test for frame error 
	bra		WaitBreak           ; continue waiting until a frame error is detected 
	movf	RCREG,W				; read the Receive buffer to clear the error condition 
 
; second loop, waiting for the START code 
WaitForStart 
	banksel	PIR1
	btfss	PIR1,RCIF			; wait until a byte is correctly received 
	bra		WaitForStart 
	banksel	RCSTA
	btfsc	RCSTA,FERR 
	bra		WaitForStart 
	movf	RCREG,W	 
 
; check for the START code value, if it is not 0, ignore the rest of the frame 
    andlw   0xff 
	btfss	STATUS,Z			; 
    bra     MainLoop           	; ignore the rest of the frame if not zero  
 
; init receive counter and buffer pointer         
	banksel	CountL
    clrf    CountL 
    clrf    CountH 
;    lfsr    0,RxBuffer 
 
; third loop, receiving 512 bytes of data 
WaitForData 
	banksel	RCSTA
    btfsc   RCSTA,FERR          ; if a new framing error is detected (error or short frame) 
    bra     RXend               ; the rest of the frame is ignored and a new synchronization 
    							; is attempted 

	banksel	PIR1 
	btfss	PIR1,RCIF			; wait until a byte is correctly received 
	bra		WaitForData			; 
	banksel	RCREG
	movf	RCREG,W				;  
	banksel	CurRX
	movwf	CurRX				; store byte	

	btfsc	CountH,0			; if > 255
	goto	DoneApply			; skip

	movlw	d'0'
	xorwf	CountL,w
	btfsc	STATUS,Z			; if equal then 
	bra		ApplyPWMR

	movlw	d'1'
	xorwf	CountL,w
	btfsc	STATUS,Z			; if equal then 
	bra		ApplyPWMG

	movlw	d'2'
	xorwf	CountL,w
	btfsc	STATUS,Z			; if equal then 
	bra		ApplyPWMB

	bra		DoneApply
 
ApplyPWMR 
	banksel	CurRX
	movf	CurRX,w				; collect byte
	banksel CCPR3L
	movwf	CCPR3L				; Move CH1 to PWMR
	bra		DoneApply

ApplyPWMG
	banksel	CurRX
	movf	CurRX,w				; collect byte
	banksel CCPR4L
	movwf	CCPR4L				; Move CH2 to PWMG
	bra		DoneApply

ApplyPWMB
	banksel	CurRX
	movf	CurRX,w				; collect byte
	banksel CCPR1L
	movwf	CCPR1L				; Move CH3 to PWMB
	
DoneApply	
	banksel	CountL

;	movwf	POSTINC0			; move the received data to the buffer  
                                ; (auto-incrementing pointer) 
	incf	CountL,F            ; increment 16-bit counter 
	btfss	STATUS,C 
	bra		WaitForData 
	incf	CountH,F 
 
	btfss	CountH,1            ; check if 512 bytes of data received 
	bra		WaitForData 
 
 
;****************************************************************************** 
; when a complete frame is received  
; use the selected CHANNEL data to control the CCP2 module duty cycle  
 
RXend 
;	lfsr	0,RxBuffer          ; use indirect pointer 0 to address the receiver buffer 
;GetData 
;	movlw	LOW(CHANNEL)       ; add the offset for the select channel 
;	addwf	FSR0L,F 
;	movlw	HIGH(CHANNEL) 
;	addwfc	FSR0H,F 
 
;	movff	INDF0,CCPR2L        ; retrieve the data and assign MSB to control PWM2 
 
	bra		MainLoop	        ; return to main loop  
 
 
;****************************************************************************** 
; Setup Serial port and pins
SetupSerial 

; initialize oscillator
	banksel	OSCCON	

;			movlw	b'01111010'
;			movwf	OSCCON

			clrf	OSCCON			; use OSC from CONFIG1
;			clrf	OSCTUNE			; factory OSC setting
	banksel	CLKRCON
;			clrf	CLKRCON			; ref clock disabled
			
; set alternate pin configurations (16f1827)

	banksel APFCON0
			bsf		APFCON0,RXDTSEL		; RX to RB2

	banksel APFCON1
			bsf		APFCON1,TXCKSEL		; TXCKSEL (0=RB2, 1=RB5)

;---------------------------------------------------------------------------------
; initialize PORTA/PORTB
;---------------------------------------------------------------------------------

PORT_Init	
	banksel	PORTA
			clrf	PORTA
			clrf	PORTB
	banksel	LATA 					
			CLRF 	LATA 			; clear PORTA Data Latch
			CLRF 	LATB 			; clear PORTB Data Latch

	banksel	TRISA
 

; RA0 = AN0
; RA1 = unused
; RA2 = unused
; RA3 = PWM output CCP3, RED, SET the pin to disable output driver for PWM
; RA4 = PWM output CCP4, GREEN, SET the pin to disable output driver for PWM
; RA5 = unused
; RA6 = output, oscillator out pin 1
; RA7 = output, oscillator in pin 2

	banksel TRISA
;			16f1827
			movlw	b'00111111'
			movwf	TRISA

; RB0 = unused
; RB1 = unused
; RB2 = UART input, DMX RX
; RB3 = PWM output CCP1, BLUE, SET the pin to disable output driver for PWM
; RB4 = Set as INPUT to transmit.  Output pulled low creates break signal. 
; RB5 = UART output, DMX TX
; RB6 = unused
; RB7 = unused

;			16f1827
			movlw 	b'11011111'
			movwf	TRISB
 
; Setup EUSART 

UART_Init				

	banksel	TXSTA					; also RCSTA, BAUDCON, SPBRGx

			bsf		RCSTA,SPEN  	; doesn't matter, we're in asynch mode, but whatever

	bsf	RCSTA,RX9
	bsf	TXSTA,TX9
	bsf	TXSTA,TX9D

			bcf		TXSTA,SYNC		; asynchronus mode (bit 4)		
			bsf 	TXSTA,TXEN		; enable Transmit (bit 5)

			bsf		TXSTA,BRGH		; high speed baud rate (bit 2)
			bsf		BAUDCON,BRG16

			clrf	SPBRGH
			movlw	d'19'			; 12mhz osc w/ high brgh
			movwf	SPBRGL			; init BRG 31250 bauds

			bcf		RCSTA,CREN  	; clear CREN, reset RX
			bsf		RCSTA,CREN  	; continuous receive enabled, restart RX


; ********************************************************************

;	banksel TXSTA

;	bsf		BAUDCON,BRG16	; select EUART 16-bit Asynchrnounou mode operation 
 
;	movlw	.19			    ; init baud rate generator for 250k baud (assume Fosc=20MHz) 
;	movwf	SPBRGL		 
;	clrf	SPBRGH


;			  ,-------------- CSRC - Doesn't matter for async, sync: 1 = master, 0 = slave
;			  |,------------- TX9 - 1 = 9 bit tranmission, 0 = 8 bit
;			  ||,------------ TXEN - transmit enabled, 1 = enabled, 0 = disabled
;			  |||,----------- SYNC - 1 = synchronous mode, 0 = asynch
;			  ||||,---------- SENDB - 1 = Send sync on next tx, 0 = sync break completed
;			  |||||,--------- BRGH - High baud rate select bit, 1 = high speed, 0 = low
;			  ||||||,-------- TRMT - transmit shift register bit, 1 = TSR empty, 0 = full
;			  |||||||,------- TX9D - 9th bit of transmit data			
;			  ||||||||
;	movlw	b'01100101'		; enable TX, 9-bit mode, high speed mode, 9th bit =1 (2 stop)
;	movwf	TXSTA			  

;			  ,-------------- SPEN - serial port enable
;			  |,------------- RX9 - 9 bit receive, 1 = 9 bit, 0 = 8 bit
;			  ||,------------ SREN - Single receive.  Only matters in Sync mode as master.  
;			  |||,----------- CREN - Continuous receive enable. 1 = enable receiver, 0 = disables.
;			  ||||,---------- ADDEN - address detection enable. 0 = disabled.  
;			  |||||,--------- FERR - Framing error bit.  1 = frame error received, 0 = none.
;			  ||||||,-------- OERR - 1 over error, (clear by clearing CREN).
;			  |||||||,------- RX9D - 9th bit of data received.  			
;			  |||||||| 
;	movlw	b'11010000'		; enable serial port and reception 
;	movwf	RCSTA			 
 
;	bcf		RCSTA,CREN  	; clear CREN, reset RX
;	bsf		RCSTA,CREN  	; continuous receive enabled, restart RX

;**************************************************************
	 
;Setup PWM module 

	banksel	CCPTMRS
;					  ,,_______ C4TSEL 00 = Timer2, 01 = Timer4, 10 = Timer 6
;					  ||,,_____ C3TSEL 00 = Timer2, 01 = Timer4, 10 = Timer 6
;					  ||||,,___ C2TSEL 00 = Timer2, 01 = Timer4, 10 = Timer 6
;					  ||||||,,_ C1TSEL 00 = Timer2, 01 = Timer4, 10 = Timer 6
			movlw	b'01010101'
			movwf	CCPTMRS	

	banksel	PR4
			movlw	d'255'		; was 127 for ColorSynth (MIDI)
			movwf	PR4

	banksel	CCP1CON
;					  ,,_______ PxM - CCPxM = 11xx then 00 = single output, PxA modulated
;					  ||,,_____ DCxB - PWM Duty Cycle least significant bits
;					  ||||,,,,_ CCPxM - Mode select - 1100 = PWM, bits 0/1 are enhanced mode
			movlw	b'00001100'
			movwf	CCP1CON
			clrf	CCP2CON
	banksel	CCP3CON
			movlw	b'00001100'
			movwf	CCP3CON
			movwf	CCP4CON

	banksel CCPR1L
			clrf	CCPR1L			; Set PWM off
	banksel CCPR3L
			clrf	CCPR3L			; Set PWM off
	banksel CCPR4L
			clrf	CCPR4L			; Set PWM off
 
; Init timers

T4Init   ; LED hardware PWM timer
	banksel	T4CON					
			movlw	b'00000100'		; Post scale 1:1, TMR4 ON, pre scale 1:1
			movwf	T4CON	

	banksel	TRISA
			bcf		TRISA,3			; Set RB3 to output, turn on PWM RED
			bcf		TRISA,4			; Set RB3 to output, turn on PWM GREEN
			bcf		TRISB,3			; Set RB3 to output, turn on PWM BLUE

	return 



; DELAYS

delay	
		banksel	count1
		movlw	d'250'			;delay 83.3 ms (12 MHz clock)
		movwf	count1
d1		movlw	0xC7			;delay .3mS
		movwf	counta
		movlw	0x01
		movwf	countb


Delay_0
		decfsz	counta, f
		goto	$+2
		decfsz	countb, f
		goto	Delay_0
		decfsz	count1	,f
		goto	d1
    	retlw	0x00

; BLINKS

InitBlink	
			banksel	CCPR3L
			movlw	d'127'
			movwf	CCPR3L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR3L
			clrf	CCPR3L
			movlw	d'127'
			banksel	CCPR4L
			movwf	CCPR4L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR4L
			clrf	CCPR4L
			movlw	d'127'
			banksel	CCPR1L
			movwf	CCPR1L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR1L
			clrf	CCPR1L

			banksel	CCPR3L
			movlw	d'127'
			movwf	CCPR3L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR3L
			clrf	CCPR3L
			movlw	d'127'
			banksel	CCPR4L
			movwf	CCPR4L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR4L
			clrf	CCPR4L
			movlw	d'127'
			banksel	CCPR1L
			movwf	CCPR1L
			call 	delay
			call 	delay
			call 	delay
			banksel	CCPR1L
			clrf	CCPR1L

			RETURN




	END 