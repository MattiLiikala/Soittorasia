;;;;;;; Description ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Harjoitus 1
; 
;  1. 5v voltage for sensor.
;
; 
; 
;    
; 
;	

;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
;

;;;;;;; Assembler directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        list  P=PIC18F452, F=INHX32, C=160, N=0, ST=OFF, MM=OFF, R=DEC, X=ON
        #include P18F452.inc
        __CONFIG  _CONFIG1H, _HS_OSC_1H  ;HS oscillator
        __CONFIG  _CONFIG2L, _PWRT_ON_2L & _BOR_ON_2L & _BORV_42_2L  ;Reset
        __CONFIG  _CONFIG2H, _WDT_OFF_2H  ;Watchdog timer disabled
        __CONFIG  _CONFIG3H, _CCP2MX_ON_3H  ;CCP2 to RC1 (rather than to RB3)
        __CONFIG  _CONFIG4L, _LVP_OFF_4L  ;RB5 enabled for I/O
        errorlevel -314, -315          ;Ignore lfsr messages

;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        cblock  0x000                  ; Beginning of Access RAM
        COUNT                          ; Counter available as local to subroutines
        ALIVECNT                       ; Counter for blinking "Alive" LED (D2)
        DELRPG                         ; Used DELRPG to let the programmer know whether
                                       ;  or not the RPG moved and at which direction 
                                       ;  (-1 for left or +1 for right).
        OLDPORTD                       ; Holds the old value of PortD to compare with
                                       ;  the new value. Used in RPG.
        PBSTATE                        ; Byte that holds all necessary bit values to 
                                       ;  make SW3 function.  Set in PButton subroutine.
        PBCOUNT                        ; Used in PButton subroutine.  If it's less than
                                       ;  PBthresh, then ISC is set.  Otherwise, ISA is set.
                                       ;  This lets PButton know if it's a short or long push.
        POTVALUE                       ; Holds the current value of the POT.
        POTVALUECOMP                   ; Holds the complemented (0xff-POTVALUE) value of POTVALUE.
        TEMPSTR:6                      ; String that hold the current temperature to be displayed on the LCD.
        HEXSTR:4                       ; String used to display the hex values located
                                       ;  on the right side of the LCD (output of the POT and the DAC)
        BYTE                           ; Temporary variable used for anything.
        SPIRECEIVE                     ; Not used in this program, but if something
                                       ;  was to be sent back to the SPI, it would 
                                       ;  be saved in this variable.
		SPIDATA						   ;	!	
		VOLT					   
        RPGCNT                         ; Used internally for the RPG subroutine.
	CNT_TITLE		       ; Counter for the titles.
	VALUE			       ; Variable used by RPG.
	STORETEMP		       ; For storing the temperature
        endc

#include c:\math18\mathvars.inc

;;;;;;; Macro definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; These are the binary variables used for the PButton and RPG subroutines.
ISC     equ 0                          ; Initiate screen change for long press.
ISA     equ 1                          ; Initiate secondary action for short press.
PDONE   equ 2                          ; Pushbutton action has been taken.
OLDPB   equ 3                          ; Old state of pushbutton.
NEWPB   equ 4                          ; New state of pushbutton.
PBthres equ 30                         ; Pushbutton threshold ofr a long press.
RGPTresh equ 6			       ; RPG treshold for fast turns.


; Lets the programmer store a literal value in a RAM location directly.
;

MOVLF   macro  literal,dest
        movlw  literal
        movwf  dest
        endm

; Used to point TBLPTRH to a constant string (stored in program memory) so that 
; it can be displayed on the LCD.
;

POINT   macro  stringname
        MOVLF  high stringname, TBLPTRH
        MOVLF  low stringname,  TBLPTRL
        endm

;;;;;;; Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        org   0x0000                   ; Reset vector.
        nop 
        goto  Mainline

        org   0x0008                   ; High priority interrupt vector.
        goto  $                        ; If interrupt is generated, go into this infinite loop.

        org   0x0018                   ; Low priority interrupt vector.
        goto  $                        ; If interrupt is generated, go into this infinite loop.

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

Mainline
        rcall  Initial                 ;Initialize everything.
		
        ;LOOP_
		rcall SetDAC
		rcall ADC_muunnos
		
		

;;;;;;; Initial subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine performs all initializations of variables and registers.

Initial
        MOVLF B'01000000', ADCON1      ; Enable PORTA & PORTE digital I/O pins for the A/D converter.
        MOVLF B'01111001', ADCON0      ; Sets up the A/D converter for the POT.
        MOVLF B'11100001', TRISA       ; Set I/O for PORTA. '1' is input while '0' is output.
        MOVLF B'11011100', TRISB       ; Set I/O for PORTB.
        MOVLF B'00000000', TRISC       ; Set I/0 for PORTC. DAC-pinnit 0,3,5 pakko olla output tilassa
        MOVLF B'00001111', TRISD       ; Set I/0 for PORTD.
        MOVLF B'00000100', TRISE       ; Set I/0 for PORTE.
        MOVLF B'00010000', PORTA       ; Turn off all four LEDs driven from PORTA.
        MOVLF B'00100000', SSPCON1     ; Sets up the SPI interface for the DAC
        MOVLF B'11000000', SSPSTAT     ;  to use with a 2.5 MHz clock.

                                       ; This sets up Timer1 and CCP1.
        MOVLF B'00001000', T3CON       ; Sets up so that T1 is used with CCP1.
        MOVLF B'10000001', T1CON       ; continue setting up T1 to CCP1.
        MOVLF B'00001011', CCP1CON     ; Set up to trigger special event so that PIR1, CCP1IF will be set.
        MOVLF B'01100001', CCPR1H      ; Set the comparator to 25,000.
        MOVLF B'10101000', CCPR1L	

                                       ; Initialize variables for RPG and PButton subroutines.
        movff PORTD, OLDPORTD          ; Initialize "old" value for RPG.
        clrf RPGCNT                    ; Initialize the RPG counter.

        MOVLF B'00001000', PBSTATE     ; Initialize pushbutton states.
        clrf PBCOUNT
	clrf VALUE
                                       ; Set up the characters that will not ever change
        MOVLF 0xC0, TEMPSTR            ; Sets the position of TEMPSTR to the lower left hand corner.
        MOVLF 0xDF, TEMPSTR+3          ; Displays the degree symbol.
        MOVLF A'C', TEMPSTR+4          ; Displays 'C' for the temperature.
        MOVLF 0x00, TEMPSTR+5          ; Terminating byte for a string.

        MOVLF 0x00, HEXSTR+3           ; Terminating byte for HEXSTR will never change.

        rcall InitLCD                  ; Initialize LCD.
        POINT NAME                     ; Display name on the LCD.
        rcall DisplayC		

	MOVLF D'1', CNT_TITLE	       ; Initialize CNT_TITLE
	
        return
;;;;;;; SetDAC subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; 5V j√§nnite sensorille
; Asetetaan DAC kanava A (0x21) arvoon 5V (0xFF)
SetDAC
		MOVLF 0x21, SPIDATA		;l√§hetet√§√§n kanavan osoite
		rcall SPItransfer
		MOVLF 0xFF, SPIDATA
		rcall SPItransfer
		
		
;;;;;;; InitLCD subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Initialize the Optrex 8x2 character LCD.
; First wait for 0.1 second, to get past display's power-on reset time.

InitLCD
        MOVLF  10,COUNT                ; Wait 0.1 second.
        ;REPEAT_
L2
          rcall  LoopTime              ; Call LoopTime 10 times.
          decf   COUNT,F
        ;UNTIL_  .Z.
        bnz	L2
RL2

        bcf    PORTE,0                 ; RS=0 for command.
        POINT  LCDstr                  ; Set up table pointer to initialization string.
        tblrd*                         ; Get first byte from string into TABLAT.
        ;REPEAT_
L3
          bsf    PORTE,1               ; Drive E high.
          movff  TABLAT,PORTD          ; Send upper nibble.
          bcf    PORTE,1               ; Drive E low so LCD will process input.
          rcall  LoopTime              ; Wait ten milliseconds.
          bsf    PORTE,1               ; Drive E high.
          swapf  TABLAT,W              ; Swap nibbles.
          movwf  PORTD                 ; Send lower nibble.
          bcf    PORTE,1               ; Drive E low so LCD will process input.
          rcall  LoopTime              ; Wait ten milliseconds.
          tblrd+*                      ; Increment pointer and get next byte.
          movf   TABLAT,F              ; Is it zero?
        ;UNTIL_  .Z.
        bnz	L3
RL3
        return

;;;;;;; T40 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Pause for 40 microseconds  or 40/0.4 = 100 clock cycles.
; Assumes 10/4 = 2.5 MHz internal clock rate.

T40
        movlw  100/3                   ; Each REPEAT loop takes 3 cycles.
        movwf  COUNT
        ;REPEAT_
L4
          decf  COUNT,F
        ;UNTIL_  .Z.
        bnz	L4
RL4
        return


;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, or
; an occasionally used two-byte cursor-positioning code of the form 0x00hh.

DisplayC
        bcf  PORTE,0                   ; Drive RS pin low for cursor-positioning code.
        tblrd*                         ; Get byte from string into TABLAT.
        movf TABLAT,F                  ; Check for leading zero byte.
        ;IF_  .Z.
        bnz	L5
          tblrd+*                      ; If zero, get next byte.
        ;ENDIF_
L5
        ;REPEAT_
L6
          bsf   PORTE,1                ; Drive E pin high.
          movff TABLAT,PORTD           ; Send upper nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will accept nibble.
          bsf   PORTE,1                ; Drive E pin high again.
          swapf TABLAT,W               ; Swap nibbles.
          movwf PORTD                  ; Write lower nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will process byte.
          rcall T40                    ; Wait 40 usec.
          bsf   PORTE,0                ; Drive RS pin high for displayable characters.
          tblrd+*                      ; Increment pointer, then get next byte.
          movf  TABLAT,F               ; Is it zero?
        ;UNTIL_  .Z.
        bnz	L6
RL6
        return

;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
	
DisplayV
        bcf  PORTE,0                   ; Drive RS pin low for cursor positioning code.
        ;REPEAT_
L7
          bsf   PORTE,1                ; Drive E pin high.
          movff INDF0,PORTD            ; Send upper nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will accept nibble.
          bsf   PORTE,1                ; Drive E pin high again.
          swapf INDF0,W                ; Swap nibbles.
          movwf PORTD                  ; Write lower nibble.
          bcf   PORTE,1                ; Drive E pin low so LCD will process byte.
          rcall T40                    ; Wait 40 usec.
          bsf   PORTE,0                ; Drive RS pin high for displayable characters.
          movf  PREINC0,W              ; Increment pointer, then get next byte.
        ;UNTIL_  .Z.                   ; Is it zero?
        bnz	L7
RL7
        return
        
;;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine decyphers RPG changes into values of DELRPG of 0, +1, or -1.
; DELRPG = +1 for CW change, 0 for no change, and -1 for CCW change.
; If 2 changes occur within (RPGTreshold * 10 ms), then multiply the values by 
; 8 (3 times shift left).

RPG
	movf   RPGCNT,w		       ; Copy RPGCNT to W.
	btfss  STATUS, Z	       ; skip if zero
	decf   RPGCNT		       ; if not zero, decrement
	bcf    PORTA,RA3
        clrf   DELRPG                  ; Clear for "no change" return value.
        movf   PORTD,W                 ; Copy PORTD into W.
        movwf  TEMP                    ;  and TEMP.
        xorwf  OLDPORTD,W              ; Any change?
        andlw  B'00000011'             ; If not, set the Z flag.
        ;IF_  .NZ.                     ; If the two bits have changed then...
        bz	L8
          rrcf OLDPORTD,W              ; Form what a CCW change would produce.
          ;IF_  .C.                    ; Make new bit 1 = complement of old bit 0.
          bnc	L9
            bcf  WREG,1
          ;ELSE_
          bra	L10
L9
            bsf  WREG,1
          ;ENDIF_
L10
          xorwf  TEMP,W                ; Did the RPG actually change to this output?
          andlw  B'00000011'
          ;IF_  .Z.                    ; If so, then change  DELRPG to -1 for CCW.
          bnz	L11
            decf DELRPG,F
          ;ELSE_                       ; Otherwise, change DELRPG to  +1 for CW.
          bra	L12
L11
            incf DELRPG,F
          ;ENDIF_
L12
        ;ENDIF_
	movf   RPGCNT,w		       ; Copy RPGCNT to W.
	btfsc  STATUS, Z	       ; skip if not zero
	bra    CNTUPDATE	       ; 	
	bsf    PORTA,RA3	       ; Flash D4, for a fast turn of the RPG.
	bcf    STATUS,C		       ; Shift value 3 times to left = multiply by 8
	rlcf   DELRPG		       ; Carry is forced to 0.
	bcf    STATUS,C
	rlcf   DELRPG 
	bcf    STATUS,C
	rlcf   DELRPG 		       ; *****

CNTUPDATE
	MOVLF	RGPTresh, RPGCNT       ; set RGPCNT to treshold		
L8
        movff  TEMP,OLDPORTD           ; Save PORTD as OLDPORTD for ten ms from now.
        return

;;;;;;;;;;;;;;;;;; Pbutton subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine sorts out long and short pushbuttons presses into two outputs:
;	ISC=1:   for slow press.
;	ISA=1:   for fast press.
;	PDONE=1: One of the above actions has occurred for this press.

Pbutton
        bcf PBSTATE,ISC                ; Clear initiate screen change bit.
        bcf PBSTATE,ISA                ; Clear initiate secondary action bit.
        
        ;IF_ PORTD,RD3 == 1            ; Copy pushbutton state to NEWPB.
        btfss PORTD,RD3
        bra	L13
          bsf PBSTATE, NEWPB
        ;ELSE_
        bra	L14
L13
          bcf PBSTATE, NEWPB
        ;ENDIF_
L14
        
        ;IF_ PBSTATE,OLDPB == 1        ; Look for leading edge. 
        btfss PBSTATE,OLDPB
        bra	L15
          ;IF_ PBSTATE,NEWPB == 0      ;  (OLDPB = 1, NEWPB = 0).
          btfsc PBSTATE,NEWPB
          bra	L16
            MOVLF PBthres, PBCOUNT     ; Start counter.
          ;ENDIF_
L16
        ;ENDIF_
L15

        ;IF_ PBSTATE,NEWPB == 0        ; Pushbutton is still pressed.
        btfsc PBSTATE,NEWPB
        bra	L17
          movf PBCOUNT, F
          ;IF_ .Z.                     ; and counter has passed threshold.
          bnz	L18
            ;IF_ PBSTATE,PDONE == 0    ; and no action has yet been taken.
            btfsc PBSTATE,PDONE
            bra	L19
              bsf PBSTATE,ISC          ; Set ISC = 1 to indicate change in button.
              bsf PBSTATE,PDONE        ; Done with pulse.
            ;ENDIF_
L19
          ;ENDIF_
L18
        ;ELSE_                         ; Pushbutton has been release.
        bra	L20
L17
          bcf PBSTATE,PDONE            ; so clear PDONE.
        ;ENDIF_
L20

        ;IF_ PBSTATE,OLDPB == 0        ; Look for trailing edge.
        btfsc PBSTATE,OLDPB
        bra	L21
          ;IF_ PBSTATE,NEWPB == 1      ; (OLDPB = 0, NEWPB = 1).
          btfss PBSTATE,NEWPB
          bra	L22
            movf PBCOUNT, F
            ;IF_ .NZ.                  ; Fast pulse.
            bz	L23
              bsf PBSTATE,ISA          ; Initiate secondary action.
            ;ENDIF_
L23
            bcf PBSTATE,PDONE          ; Done with pulse.
            clrf PBCOUNT               ; Finish counting.
          ;ENDIF_
L22
        ;ENDIF_
L21

        movf PBCOUNT, F                ; Has counter reached zero?
        ;IF_ .NZ.                      ; If not, then decrement it.
        bz	L24
          decf PBCOUNT, F
        ;ENDIF_
L24

        ;IF_ PBSTATE,NEWPB == 1        ; Copy NEWPB to OLDPB.
        btfss PBSTATE,NEWPB
        bra	L25
          bsf PBSTATE,OLDPB
        ;ELSE_
        bra	L26
L25
          bcf PBSTATE,OLDPB
        ;ENDIF_
L26
        
        return

;;;;;;;;;;;;;;;;;; ReadPot ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine reads the value of the POT using the A/D and stores it in POTVALUE.
; It also complements POTVALUE and stores it in POTVALUECOMP.  Next, depending on 
; whether or not PORTA, RA1 (LED D6) is low or high, either POTVALUE or POTVALUECOMP
; is displayed respectively.

ReadPot
        bsf ADCON0,GO_DONE             ; Begins the A/D conversion.
        ;REPEAT_
L27
        ;UNTIL_ ADCON0,GO_DONE == 0    ; Wait until A/D conversion is finished.
        btfsc ADCON0,GO_DONE
        bra	L27
RL27
        movff ADRESH,POTVALUE          ; Move the upper byte from the A/D into POTVALUE.
        movf  ADRESH,W                 ; Perform 0xFF-POTVALUE and store it in POTVALUECOMP.
        sublw 0xff
        movwf POTVALUECOMP
        MOVLF 0xC6,HEXSTR              ; Set the position of HEXSTR to the lower right hand corner of the LCD.
        
        ;IF_ PORTA,RA1 == 1            ; Is D6 lit?
        btfss PORTA,RA1
        bra	L28
          movff POTVALUECOMP, BYTE     ; .. then set BYTE to display the complement.
        ;ELSE_ 
        bra	L29
L28
          movff POTVALUE, BYTE         ; .. otherwise display the value of the POT.
        ;ENDIF_
L29

        rcall HexDisplay               ; Converts BYTE into a displayable string and displays it.
        return
        
;;;;;; HexDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine takes in BYTE and converts into an ASCII string in hex and stores it into
; HEXSTR for display.

HexDisplay
        rcall ConvertLowerNibble       ; Converts lower nibble of BYTE to hex and stores it in W.
        movwf HEXSTR+2                 ; Move the ASCII value into the most significant byte of HEXSTR.

        swapf BYTE,F                   ; Swap nibbles of BYTE to convert the upper nibble.

        rcall ConvertLowerNibble       ; Convert the old upper nibble of BYTE to hex and stores it in W.
        movwf HEXSTR+1                 ; Move the ASCII value into the 2nd byte of HEXSTR.

        lfsr  0, HEXSTR                ; Loads address of HEXSTR to fsr0 to display HEXSTR.
        rcall DisplayV                 ; Call DisplayV to display HEXSTR on LCD.
        return

;;;;;; ConvertLowerNibble subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine takes lower nibble of BYTE and converts it into its ASCII hex value.

ConvertLowerNibble                              
        movf BYTE, W                   ; Loads BYTE into W.
        andlw B'00001111'              ; Masks out the upper nibble.
        sublw 0x09                     ; Test if it's greater than 9. 
        ;IF_ .N.                       ; If, after masking, it is greater than 9, then it is a letter.
        bnn	L30
          movf BYTE,W                  ; Load BYTE into W.
          andlw B'00001111'            ; Mask out the upper nibble.
          addlw 0x37                   ; Add offset to obtain the letter's ASCII value.
        ;ELSE_                         ; If it's less than 9, then it's a number.
        bra	L31
L30
          movf BYTE,W
          andlw B'00001111'
          iorlw 0x30                   ; Then add offset of 30 to obtain the numeric's ASCII value.
        ;ENDIF_
L31
        return	


;;;;;;; PBToggle subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine tests ISA and ISC to see if SW3 has been pressed and toggles the state
; of LED D6 accordingly.

PBToggle
        ;IF_ PBSTATE,ISA == 1          ; If ISA = 1, the button's been pressed.
        btfss PBSTATE,ISA
        bra	L32

	decfsz CNT_TITLE		;decrement TITLE counter, skip if zero
	bra tro_L0
	movlw D'4'
	movwf CNT_TITLE
tro_L0
	movlw D'1'
	subwf CNT_TITLE, 0
	bz tro_L1			;was 1
	movlw D'2'
	subwf CNT_TITLE, 0
	bz tro_L2			;was 2
	movlw D'3'
	subwf CNT_TITLE, 0
	bz tro_L3			;was 3
	movlw D'4'
	subwf CNT_TITLE, 0
	bz tro_L4			;was 4
	bra tro_L6

tro_L1

        POINT NAME                     ; Display name on the LCD.
	bra tro_L5
tro_L2
	POINT TITLE1
	bra tro_L5
tro_L3
	POINT TITLE2
	bra tro_L5
tro_L4
	POINT TITLE3
	bra tro_L5
tro_L5	
	rcall DisplayC
tro_L6

        ;ENDIF_
L32

        ;IF_ PBSTATE,ISC == 1          ; Or if ISC = 1, the button's been pressed.
        btfss PBSTATE,ISC
        bra	L33
          btg PORTA,RA1                ; Then toggle RA1(LED D6).
        ;ENDIF_
L33
        return

;;;;;;; RPGCounter subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; In this subroutine, the value of DELRPG is added to VALUE and shown on LCD.

RPGCounter


	movf  DELRPG,W
	addwf VALUE
        MOVLF 0x86,HEXSTR              ; Load cursor position byte (upper right) to HEXSTR.
        movff VALUE,BYTE               ; Load the converted result to BYTE for display.
        rcall HexDisplay               ; Call HexDisplay to display the result in hex.

        return

;;;;;;; PotToDAC subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine deals with DAC(D/A Converter) It takes the value of POT
; and sends it to the DAC-A and the complement of the value to the DAC-B.  

PotToDAC
                                       ; Instructions dealing with DAC.
        bcf   PORTC,RC0                ; Clear PORTC,RC0 to change DAC output.
        MOVLF 0x21,BYTE                ; Load control byte 0x21 to BYTE for the output to go to DAC-A.
        rcall SPItransfer              ; Transfer what is in BYTE.
        movff POTVALUE,BYTE            ; Load the byte to be converted.
        rcall SPItransfer                      
        bsf   PORTC,RC0                ; Set RC0 to finish transfer.

        bcf   PORTC,RC0
        MOVLF 0x22,BYTE                ; Load control byte 0x22 to BYTE for the output to go to DAC-B.
        rcall SPItransfer
        movff POTVALUECOMP,BYTE
        rcall SPItransfer
        bsf   PORTC,RC0                        
        
         
        return

;;;;;;;; SPI, D/A muunnin;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

SPItransfer
			bcf PIR1,SSPIF             ;Clear PIR1,SSPIF to ready for; transfer.
			movff SPIDATA,SSPBUF       ; Initiates write when anything is
			BANKSEL SSPSTAT            ; placed in SSPBUF.
Wait_SPI                               ; Wait until transfer is finished.
			btfss PIR1,SSPIF
			bra Wait_SPI
			BANKSEL SSPBUF
			movf SSPBUF,W
			return



;;;;;;;;ADC_muunnos  muunnin;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

ADC_muunnos
			bsf ADCON0, ADON 	;avaa ADC
			bsf ADCON0,GO		;k‰ynnist‰ muunnos
ADC_tee
			btfsc ADCON0, GO	;GO muuttuu nollaksi kun k‰‰nnˆs on tehty
			bra ADC_tee
			movff ADRESL, VOLT	;ADRESH on muuntimen 8 ylint‰ bitti‰, muunnos on 10 bittinen
								;VOLT on itse m‰‰ritelty muuttuja, jonne muutokset tulos tallennetaan
			bcf ADCON0, ADON	;sulje ADC
			return
        
;;;;;; DecDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine takes BYTE, converts it to an ASCII decimal value of BYTE and displays
; it.  This function does not set the positioning code.  For the temperature, the code
; is constant, so the position is initialized in the Initial subroutine.

TempDisplay

        lfsr 0, TEMPSTR+2              ; Loads FSR0 register to BYTESTR + 2.

        ;REPEAT_
L44
          clrf  WREG                   ; Clear work register.
          movff BYTE, AARGB0           ; Move BYTE to AARGB0 to be divided.
          MOVLF D'10', BARGB0          ; Divide BYTE by 10.
          call  FXD0808U               ; Perform division.
          movf  REMB0, W               ; Move remainder to work register.
          
          iorlw 0x30                   ; Add offset to convert to an ASCII decimal number.
          movwf POSTDEC0               ; Load the ASCII value to the string and move to next string byte.
          
          movff AARGB0, BYTE           ; Move result to divisor to be divided again.	
          movf  FSR0L,W                ; Done?
          sublw low TEMPSTR
        ;UNTIL_ .Z.			
        bnz	L44
RL44
        
        lfsr 0, TEMPSTR                ; Set pointer to display temperature string: TEMPSTR.
        rcall   DisplayV               ; Call DisplayV to display temperature, a variable string.
        return
        
;;;;;;; BlinkAlive subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; This subroutine briefly blinks the LED next to the PIC every two-and-a-half
; seconds.

BlinkAlive
        bsf  PORTA,RA4                 ; Turn off LED.
        decf ALIVECNT,F                ; Decrement loop counter and return if not zero.
        ;IF_  .Z.
        bnz	L45
          MOVLF 250,ALIVECNT           ; Reinitialize BLNKCNT.
          bcf   PORTA,RA4              ; Turn on LED for ten milliseconds every 2.5 sec.
        ;ENDIF_
L45
        return

;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;Gives a 10ms looptime by using Timer1 and comparing it to 25,000.

LoopTime
        ;REPEAT_                       ; Repeat until TIMER1 has reached 25,000. 
L46
        ;UNTIL_ PIR1, CCP1IF == 1	       
        btfss PIR1,CCP1IF
        bra	L46
RL46
        bcf    PIR1, CCP1IF            ; When it has, reset it to start over.
        return

;;;;;;; Constant strings ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LCDstr  db  0x33,0x32,0x28,0x01,0x0c,0x06,0x00  ; Initialization string for LCD.
NAME	db  0x80,'E','T','T','_','1',' ',0x00   ; Declaration of name string on LCD (max 6 chars).
TITLE3  db  0x80,'B','y',':',' ',' ',' ',0x00	; All strings must be of same lenght, 
TITLE2  db  0x80,'t','e','a','n','s','a',0x00	; else the last letters of previous line
TITLE1  db  0x80,'J','a','n','0','5',' ',0x00	; will remain on the screen...

        #include c:\math18\FXD2416U.INC
        #include c:\math18\FXD0808U.INC
        #include c:\math18\FXM1608U.INC

        end
