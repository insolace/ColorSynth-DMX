; File:  DMX512RecDemo.asm 
; DMX512 Receiver 
; This file uses a PIC18F2420 device to receive DMX-512 data and store it 
; into a 512 byte receive buffer.   
; For demonstration purposes, a selected data slot is written to the  
; CCP module.  The CCP module is configured in PWM mode and the received 
; data adjusts the duty cycle.  If a resistor and LED is connected to the 
; PWM output, the received DMX data can be visually observed.  
 
	list p=PIC18F24J10				; define traget processor 
	#include <P18F24J10.inc>		; include processor specific definition 
	 
;Configuration bits setup 
	CONFIG	CCP2MX = ALTERNATE		; assign CCP2 output to pin RB3 
	CONFIG 	WDTEN = OFF         	; To use ICD2 as a debugger disable Watch Dog Timer	 
    CONFIG 	STVREN = ON          	; Reset on stack overflow/underflow enabled 
    CONFIG  XINST = OFF         	; Instruction set extension and Indexed Addressing  
									; mode disabled (Legacy mode) 
	CONFIG  CP0 = OFF        		; Program memory is not code-protected 
    CONFIG 	FOSC = ECPLL        	; EC oscillator, PLL enabled and under software  
									; control, CLKO function on OSC2 
    CONFIG 	FOSC2 = ON          	; Clock selected by FOSC as system clock is enabled 
									; when OSCCON<1:0> = 00 
    CONFIG 	FCMEN = OFF         	; Fail-Safe Clock Monitor disabled 
    CONFIG 	IESO = OFF          	; Two-Speed Start-up disabled 
    CONFIG 	WDTPS = 32768        	; 1:32768 
 
	 
; Constants 
#define CHANNEL .510            ; select the receiver slot/channel 
 
; Variables 
	CBLOCK	0x8 
	    CountH                  ; 16-bit counter 
        CountL 
	    RxBuffer: .512          ; 512 bytes buffer allocation 
	 
	ENDC 
	 
;****************************************************************************** 
	ORG 	0x0 
 
Main 
	call	SetupSerial			;Setup Serial port and buffers 
 
MainLoop 
	 
; first loop, synchronizing with the transmitter 
WaitBreak 
	btfsc	PIR1,RCIF           ; if a byte is received correctly 
	movf	RCREG,W             ; discard it 
	btfss	RCSTA,FERR			; else  
	bra		WaitBreak           ; continue waiting until a frame error is detected 
	movf	RCREG,W				; read the Receive buffer to clear the error condition 
 
; second loop, waiting for the START code 
WaitForStart 
	btfss	PIR1,RCIF			; wait until a byte is correctly received 
	bra		WaitForStart 
	btfsc	RCSTA,FERR 
	bra		WaitForStart 
	movf	RCREG,W	 
 


; check for the START code value, if it is not 0, ignore the rest of the frame 
    andlw   0xff 
    bnz     MainLoop           ; ignore the rest of the frame if not zero  
 
; init receive counter and buffer pointer         
    clrf    CountL 
    clrf    CountH 
    lfsr    0,RxBuffer 
 
; third loop, receiving 512 bytes of data 
WaitForData 
    btfsc   RCSTA,FERR          ; if a new framing error is detected (error or short frame) 
    bra     RXend               ; the rest of the frame is ignored and a new synchronization 
    							; is attempted 
 
	btfss	PIR1,RCIF			; wait until a byte is correctly received 
	bra		WaitForData			; 
	movf	RCREG,W				;  
 
MoveData 
	movwf	POSTINC0			; move the received data to the buffer  
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
	lfsr	0,RxBuffer          ; use indirect pointer 0 to address the receiver buffer 
GetData 
	movlw	LOW(CHANNEL)       ; add the offset for the select channel 
	addwf	FSR0L,F 
	movlw	HIGH(CHANNEL) 
	addwfc	FSR0H,F 
 
	movff	INDF0,CCPR2L        ; retrieve the data and assign MSB to control PWM2 
 
	bra		MainLoop	        ; return to main loop  
 
 
;****************************************************************************** 
; Setup Serial port and buffers 
SetupSerial 
 
;Clear the receive buffer 
	lfsr	0,RxBuffer 
CBloop 
	clrf	POSTINC0	    ; clear INDF register then increment pointer 
	incf	CountL,F 
	btfss	STATUS,C 
	bra		CBloop 
	incf	CountH,F 
 
	btfss	CountH,1 
	bra		CBloop	 
 
; Setup EUSART 
	bsf		TRISC,7         ; allow the EUSART RX to control pin RC7 
	bsf		TRISC,6         ; allow the EUSART TX to control pin RC6 
 
	movlw	0x04			; Disable transmission 
	movwf	TXSTA			; enable transmission and CLEAR high baud rate 
 
	movlw	0x90 
	movwf	RCSTA			; enable serial port and reception 
 
	bsf		BAUDCON,BRG16	; Enable UART for 16-bit Asyn operation 
	clrf	SPBRGH 
 
	movlw	.15				; Baud rate is 250KHz for 16MHz Osc. freq. 
	movwf	SPBRG	 
	 
;Setup PWM module 
	movlw   0x0c            ; configure CCP2 for PWM mode 
    movwf   CCP2CON 
	 
;Timer2 control 
    movlw   0x04            ; enable Timer2, select a prescale of 1:1 
	movwf	T2CON 
	 
;PWM period 
	movlw	0xFF            ; 256 x .25us = 64us period 
	movwf	PR2 
	 
; init I/O  
    movlw   b'11110111'      ; make pin RB3 (CCP2) output 
    movwf   TRISB 
 
 
	return 
 
	END 