**************************************************************
* stangps.asm
* -----------
*
* WildStang Positioning System
*  Reads an array of sensors including light sensors,
*  potentiometers, accelerometers, and a solid state gyro
*  and computes the robot's position on the field using
*  those values.
*
* Copyright 2003 WildStang Robotics Team
*
*
* Version History:
* ----------------
* 01/31/03 David Flowerday      Initial version
* 02/03/03 David Flowerday      Adding support for gyroscope
* 02/04/03 David Flowerday      Adding support for potentiometer
* 02/04/03 David Flowerday      Adding support for handling RC requests
* 02/05/03 David Flowerday      Average gyro readings to find gyro center
* 02/11/03 David Flowerday      Change light sensor distance to 16 bit fraction
* 02/19/03 David Flowerday      Changed direction of light sensors due to
*                               hardware change
* 02/24/03 David Flowerday      Added additional datasets which can be
*                               requested using different command bytes
* 02/27/03 David Flowerday      Enabled COP watchdog
* 03/12/03 David Flowerday      Reverted to Comp_2003-02-19_Shipped Serial ISR
**************************************************************

$INCLUDE 'gpgtregs.inc'         ; Processor defines

; Memory Map Constants
RAMStart                EQU $0040
FLASHStart              EQU $8000
VectorStart             EQU $FFDC

; Display (LED Bargraph) Constants
LED_PORT                EQU PORTA
LED_RX                  EQU $01
LED_UNKNOWN_CMD         EQU $02
LED_VALID_CMD           EQU $04
LED_GYRO_INT            EQU $08
LED_UNUSED1             EQU $10
LED_GETB_TIMEOUT        EQU $20
LED_XYT_RECVD           EQU $40
LED_WARM_RESET          EQU $80

; Photo Sensor Constants
; 0.2958984375 = RC tick per light sensor tick
PHOTO_SENSOR_PORT       EQU PORTD
DISTANCE_RES_HIGH       EQU $4B
DISTANCE_RES_LOW        EQU $C0

; Analog to Digital Converter Constants
ADC_POT_CHAN            EQU $1
ADC_GYRO_CHAN           EQU $0
ADC_ENABLE              EQU $00

; Potentiometer Constants
; REAL ROBOT
;POT_CENTER_VAL          EQU 124t
;POT_MAX_RIGHT_VAL       EQU 12t
;POT_MAX_OFFSET          EQU 128t
;BRADS_PER_TICK_H        EQU 1t
;BRADS_PER_TICK_L        EQU 37t
; PROTO ROBOT
POT_CENTER_VAL          EQU 127t
POT_MAX_RIGHT_VAL       EQU 13t
POT_MAX_OFFSET          EQU 128t
BRADS_PER_TICK_H        EQU 1t
BRADS_PER_TICK_L        EQU 30t

; Gyroscope Constants
; First set is for 64 degs/sec gyro
;GYRO_PDEGS_HIGH         EQU $1e
;GYRO_PDEGS_LOW          EQU $48
;GYRO_NDEGS_HIGH         EQU $1e
;GYRO_NDEGS_LOW          EQU $48
; This set is for 75 degs/sec gyro (needs different factor
; for positive versus negative for some reason)
GYRO_PDEGS_HIGH         EQU $22
GYRO_PDEGS_LOW          EQU $50
GYRO_NDEGS_HIGH         EQU $20
GYRO_NDEGS_LOW          EQU $F9

; Serial Communications Constants
REQ_X_ONLY              EQU 85t
REQ_SET_XYT             EQU 120t
REQ_SET_WAYPT           EQU 130t
REQ_GET_WAYPT           EQU 140t
REQ_Y_ONLY              EQU 170t
REQ_XYTHETA             EQU 180t
REQ_THETA_ONLY          EQU 240t
REQ_INVALID             EQU 255t

**************************************************************
**************************************************************
**************************************************************
* Start of RAM
**************************************************************
**************************************************************
**************************************************************
        org RAMStart
TempWord1:
        ds $02
TempWord2:
        ds $02
TempLWord:
        ds $04
ITempWord1:
        ds $02
ITempWord2:
        ds $02
ITempLWord:
        ds $04
AbsoluteX:
        ds $04
AbsoluteY:
        ds $04
DeltaX:
        ds $02
DeltaY:
        ds $02
DeltaXNeg:
        ds $01
DeltaYNeg:
        ds $01
Distance:
        ds $02
DistanceNeg:
        ds $01
RobotAngle:
        ds $01
LastPhotoVals:
        ds $01
NumQuadErrors:
        ds $01
RobotTheta:
        ds $04
GyroValNeg:
        ds $01
GyroValue:
        ds $01
GyroCenter:
        ds $02
GyroLoopCount:
        ds $01
PotValue:
        ds $01
AbsHeading:
        ds $01
PotBrads:
        ds $01
ResetStatus:
        ds $01
RCCurrentWaypt:
        ds $01
DataRequest:
        ds $01
PosInitialized:
        ds $01

**************************************************************
**************************************************************
**************************************************************
* Start of FLASH
**************************************************************
**************************************************************
**************************************************************
        org FLASHStart
$INCLUDE 'sinetable.inc'
$INCLUDE 'pottable.inc'

**************************************************************
**************************************************************
* Quadrature Decoding Functions
**************************************************************
**************************************************************

**************************************************************
* QuadratureTable - This is a jump table used by the
*                   quadrature decoding algorithm.  It is
*                   indexed by the previous readings and
*                   the current readings.
**************************************************************
QuadratureTable:
        ; This is a lookup table used by the quadrature
        ; decoding algorithm.
        jmp QNoChange           ; 00 00 -> No change
        jmp QReverse            ; 00 01 -> Reverse rotation
        jmp QForward            ; 00 10 -> Forward rotation
        jmp QError              ; 00 11 -> Shouldn't happen
        jmp QForward            ; 01 00 -> Forward rotation
        jmp QNoChange           ; 01 01 -> No change
        jmp QError              ; 01 10 -> Shouldn't happen
        jmp QReverse            ; 01 11 -> Reverse rotation
        jmp QReverse            ; 10 00 -> Reverse rotation
        jmp QError              ; 10 01 -> Shouldn't happen
        jmp QNoChange           ; 00 00 -> No change
        jmp QForward            ; 10 11 -> Forward rotation
        jmp QError              ; 11 00 -> Shouldn't happen
        jmp QForward            ; 11 01 -> Forward rotation
        jmp QReverse            ; 11 10 -> Reverse rotation
        jmp QNoChange           ; 00 00 -> No change

**************************************************************
* QNoChange - There was no change in the light sensor
*             readings so don't update the position
**************************************************************
QNoChange:
        mov #0,Distance
        mov #0,{Distance+1}
        mov #0,DistanceNeg
        rts                     ; Just return to the caller

**************************************************************
* QForward - Wheels turned forward, so increment
*            the position
**************************************************************
QForward:
        lda #DISTANCE_RES_LOW   ; Low byte of distance resolution
        sta {Distance+1}
        lda #DISTANCE_RES_HIGH  ; High byte of distance resolution
        sta Distance
        clr DistanceNeg         ; Distance is not negative
        rts                     ; Return to main loop

**************************************************************
* QReverse - Wheels turned backwards, so decrement
*            the position
**************************************************************
QReverse:
        lda #DISTANCE_RES_LOW   ; Low byte of distance resolution
        sta {Distance+1}
        lda #DISTANCE_RES_HIGH  ; High byte of distance resolution
        sta Distance
        mov #$80,DistanceNeg    ; Distance is negative
        rts                     ; Return to main loop

**************************************************************
* QError - Illegal light sensor state change
**************************************************************
QError:
        inc NumQuadErrors       ; Increment our error count
        mov #0,Distance
        mov #0,{Distance+1}
        mov #0,DistanceNeg
        rts                     ; Return to main loop


**************************************************************
**************************************************************
* Initialization Functions
**************************************************************
**************************************************************

**************************************************************
* InitPorts - Initializes all I/O ports and sets appropriate
*             pullup enable registers
**************************************************************
InitPorts:
        mov #$FF,DDRA           ; Port A is all outputs (LEDs)
        mov #$00,LED_PORT       ; Initialize all LEDs to off
        mov #$00,DDRD           ; Port D is all inputs
        mov #$03,PTDPUE         ; Enable pullup on D0 and D1
        rts

**************************************************************
* InitSCI - Turns on the asyncronous communications port
*           for TX, RX at specified baud N81
**************************************************************
InitSCI:
        lda SCS1
        mov #$04,SCBR           ; Baud Rate = 9600 (at 9.8MHz clock)
        mov #$40,SCC1           ; Enable the SCI peripheral
        mov #$2C,SCC2           ; Enable the SCI for TX and RX
        mov #$00,SCC3           ; No SCC error interrupts, please
        rts

**************************************************************
* InitADC - Initializes the Analog to Digital Converter
**************************************************************
InitADC:
        mov #$60,ADCLK          ; Select external clock, divided by 8
        rts

**************************************************************
* CheckRSR - Checks the Reset Status Register and sets
*            the Carry Bit if a warm reset is appropriate
**************************************************************
CheckRSR:
        lda SRSR                ; Read Reset Status Register
        sta ResetStatus         ; Store for later
        and #%11000100          ; Check for a reason that would invalidate RAM
        bne NoWarmReset         ; Branch if RAM is invalid
        lda LED_PORT            ; Load state of LEDs
        eor #LED_WARM_RESET     ; Set Warm Reset LED bit
        sta LED_PORT            ; Write changes back to LED display
        sec                     ; Set Carry bit to indicate warm reset possible
        bra CheckRSRDone
NoWarmReset:
        clc                     ; Clear Carry bit to indicate no warm reset
CheckRSRDone:
        rts

**************************************************************
* InitWait - Just wait for a period of time and display a
*            countdown sequence on the LEDs
*            (this is used to allow the gyro to stablize
*            before we begin reading it)
**************************************************************
InitWait:
        clrx                    ; Clear the index register
        clr LED_PORT            ; Turn off all LEDs
        bsr Sleep500msec        ; Sleep for .5 seconds
        mov #$80,LED_PORT       ; Light up MSB of LED bargraph
ContinueWait:
        bsr Sleep500msec        ; Sleep for .5 seconds
        lda LED_PORT            ; Read our LED status
        cmp #$FF                ; Check to see if they're all lit
        beq InitWaitDone
        asr LED_PORT            ; Shift down one more bit
        bra ContinueWait
InitWaitDone:
        clr LED_PORT            ; Turn off all LEDs
        bsr Sleep500msec        ; Sleep for .5 seconds
        rts

Sleep500msec:
        mov #$25,T2MODH         ; Used to be 0x4B
        mov #$00,T2MODL         ; Set up for .5 second delay
        mov #$16,T2SC           ; Start timer 1 (prescalar == x / 64)
SleepLoop:
        sta COPCTL              ; Kick watchdog
        brclr 7,T2SC,SleepLoop  ; Busy loop until timer expires
        rts


**************************************************************
* InitRAM - Checks reset reason to determine if RAM is still
*           valid from prior to reset.  If not, all RAM vars
*           are initialized.
**************************************************************
InitRAM:
        mov #0,RobotAngle
        mov #127t,AbsoluteX
        mov #0t,{AbsoluteX+1}
        mov #0,{AbsoluteX+2}
        mov #0,{AbsoluteX+3}
        mov #127t,AbsoluteY
        mov #0t,{AbsoluteY+1}
        mov #0,{AbsoluteY+2}
        mov #0,{AbsoluteY+3}
        mov #0,DeltaX
        mov #0,{DeltaX+1}
        mov #0,DeltaY
        mov #0,{DeltaY+1}
        mov #0,Distance
        mov #0,{Distance+1}
        mov #0,DistanceNeg
        mov #0,GyroValNeg
        mov #0,RobotTheta
        mov #0,{RobotTheta+1}
        mov #0,{RobotTheta+2}
        mov #0,{RobotTheta+3}
        mov #0,GyroLoopCount
        mov #0,GyroCenter
        mov #0,{GyroCenter+1}
        mov #0,PotValue
        mov #0,RCCurrentWaypt
        mov #0,PosInitialized
        mov #REQ_INVALID,DataRequest
        lda PHOTO_SENSOR_PORT
        and #$03
        asla
        asla
        sta LastPhotoVals
        mov #0,NumQuadErrors
        rts

**************************************************************
* InitGyro - Determines gyro center.  Turns LEDs on before
*            sampling and turns them off when done.
**************************************************************
InitGyro:
        mov #$FF,LED_PORT       ; Light up all LEDs to signal sampling of gyro
        clrx                    ; Clear the index register
InitGyroLoop:
        sta COPCTL              ; Kick watchdog
        lda #ADC_GYRO_CHAN      ; Load the channel # that the gyro is on
        ora #ADC_ENABLE         ; Set the enable bit
        sta ADSCR               ; Start an ADC conversion on the gyro chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        lda ADR                 ; Read ADC value
        add {GyroCenter+1}      ; Add to our running total (LSB)
        sta {GyroCenter+1}      ; Store to LSB
        clra
        adc GyroCenter          ; Add to our running total (MSB)
        sta GyroCenter          ; Store to MSB
        incx
        cpx #$00                ; See if we've taken 256 samples yet
        bne InitGyroLoop        ; Take another sample
        ; Done sampling, now average
        brclr 7,{GyroCenter+1},NoRound
        inc GyroCenter          ; Round up because MSB of lower byte was set
NoRound:
        clr LED_PORT            ; Turn off the LEDs to indicate center found
        rts

**************************************************************
* InitGyroTimer - Initializes the gyro timer (timer 1)
**************************************************************
InitGyroTimer:
        mov #$00,T1MODH
        mov #$26,T1MODL         ; Set up for ~1ms interrupts
        mov #$56,T1SC           ; Start timer 1 (prescalar == x / 64)
        rts


**************************************************************
**************************************************************
* Main - this is the function that will be called upon reset
**************************************************************
**************************************************************
Main:
        ; Kick watchdog
        clra
        sta COPCTL              ; Kick watchdog

        ; Initialize registers
        mov #$29,CONFIG1        ; Disable COP, disable LVI reset,
                                ; enable 5 volt LVI thresholds
        mov #$00,CONFIG2
        rsp                     ; Reset stack pointer
        clra                    ; Clear accumulator
        clrh                    ; Clear high index register
        clrx                    ; Clear index register

        ; Initialize subsystems
        jsr InitPorts           ; Initialize general purpose I/O ports
        jsr InitSCI             ; Initialize the serial comm. interface
        jsr InitADC             ; Initialize the analog to digital converter
        jsr CheckRSR            ; Check the Reset Reason
        bcs WarmReset
        jsr InitWait            ; Delay for some time to allow gyro to settle
        jsr InitRAM             ; Initialize RAM variables
        jsr InitGyro            ; Find the gyro center position
WarmReset:
        jsr InitGyroTimer       ; Initialize the gyroscope timer
        cli                     ; Enable interrupts

**************************************************************
* MainLoop - Handles reading of all sensors except gyro and
*            responds to data requests.
**************************************************************
MainLoop:
        sta COPCTL              ; Kick watchdog

        jsr ReadPhotos          ; Find our 'Distance Delta'
        jsr ReadPot             ; Find our 'Crab Angle'
        jsr ComputeHeading      ; Compute heading using our 'Crab Angle'
                                ; and our 'Robot Theta'

        eor DistanceNeg         ; Correct the angle if we're going backwards

        jsr DetermineQuad       ; Determine what quadrant our vector is in

        jsr ComputeDeltas       ; Find our 'Delta X' and 'Delta Y' using
                                ; 'Dist Delta', 'Crab Angle' and 'Robot Angle'

        jsr UpdatePosition      ; Updates our field position using
                                ; 'Delta X' and 'Delta Y'

        bra MainLoop            ; Do it all over again

**************************************************************
* ReadPhotos - Reads light sensors to determine the change in
*              distance from the last time it was called (if any)
**************************************************************
ReadPhotos:
        lda PHOTO_SENSOR_PORT   ; Read the photo sensors
        and #$03                ; Mask off the photo sensor bits
        psha                    ; Store new reading on stack for use later
        ora LastPhotoVals       ; OR in the last readings
        ldx #$03
        mul                     ; Multiply by 3 for jump table purposes
        tax                     ; Move table index into index register
        clrh
        jsr QuadratureTable,X   ; Look up the state transition
        pula                    ; Pull new reading back off stack
        asla                    ; Shift new readings into old readings place
        asla                    ;
        sta LastPhotoVals       ; Store off what are now the "old readings"
        rts                     ; Return to main loop

**************************************************************
* ReadPot - Reads the potentiometers, returns crab angle
*           in brads
**************************************************************
ReadPot:
        lda #ADC_POT_CHAN       ; Load the channel # that the pot is on
        ora #ADC_ENABLE         ; Set the enable bit
        sei                     ; Turn off interrupts
        sta ADSCR               ; Start an ADC conversion on the pot chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        ldx ADR                 ; Read ADC value
        cli
        clrh
        stx PotValue            ; Store in RAM
        lda PotTable,X          ; Get PotToBrads(ADR)
        sta PotBrads            ; Store in RAM
        rts

**************************************************************
* ComputeHeading - Using 'Robot Theta' and 'Crab Angle',
*                  calculate our heading
*                  Args: Crab angle in A, Robot Theta in
*                  global variable
*                  Returns: Heading in A
**************************************************************
ComputeHeading:
        add RobotTheta          ; Add in our robot's orientation
                                ; (measured in brads)
        brclr 7,{RobotTheta+1},NoIncHeading
        inca
NoIncHeading:
        sta AbsHeading
        rts

**************************************************************
* ComputeDeltas - Using distance and direction, calculate
*                 Delta X and Delta Y
*                 Args: Direction in A, distance on stack
**************************************************************
ComputeDeltas:
        pshx                    ; Save off index register since we use it
        psha                    ; Save direction onto stack

        ; Find Delta Y
        ldx #2
        mul                     ; Multiply A by 2
        clrh
        tax                     ; Transfer direction to index register
        lda {SineTable+1},X     ; Look up sin(direction)
        sta {TempWord1+1}       ; Store LSB of sin(dir) in TempWord1
        lda SineTable,X         ; Load MSB of sine
        sta TempWord1           ; Store MSB of sin(dir) in TempWord1
        lda {Distance+1}        ; Load LSB of Distance
        sta {TempWord2+1}       ; Store into TempWord2
        lda Distance            ; Load MSB of Distance
        sta TempWord2           ; Store into TempWord2
        jsr UMult16             ; Multiply Distance by sin(Direction)
        lda {TempLWord+1}       ; Load integer portion of Delta Y
        sta DeltaY              ; Store into DeltaY
        lda {TempLWord+2}       ; Load fractional portion of Delta Y
        sta {DeltaY+1}          ; Store into DeltaY

        ; Find Delta X
        pula                    ; Reload A with direction
        sub #64t                ; A = A - 90 degrees
        ldx #2
        mul                     ; Multiply A by 2
        clrh
        tax                     ; Transfer direction to index register
        lda {SineTable+1},X     ; Look up sin(direction) (== cos(dir))
        sta {TempWord1+1}       ; Store LSB of sin(dir) in TempWord1
        lda SineTable,X         ; Load MSB of sine
        sta TempWord1           ; Store MSB of sin(dir) in TempWord1
        lda {Distance+1}        ; Load LSB of Distance
        sta {TempWord2+1}       ; Store into TempWord2
        lda Distance            ; Load MSB of Distance
        sta TempWord2           ; Store into TempWord2
        jsr UMult16             ; Multiply Distance by cos(Direction)
        lda {TempLWord+1}       ; Load integer portion of Delta X
        sta DeltaX              ; Store into DeltaX
        lda {TempLWord+2}       ; Load fractional portion of Delta X
        sta {DeltaX+1}          ; Store into DeltaX
        pulx                    ; Restore index register
        rts

**************************************************************
* DetermineQuad - Determines the quadrant of the delta
*                 vector and sets DeltaXNeg and DeltaYNeg
*                 appropriately
*                 Args: angle in A (not modified)
**************************************************************
DetermineQuad:
        cmp #64t                ; Compare angle to 90 degrees
        bhi CheckQuad2          ; Greater than 90, check quadrant 2
        clr DeltaXNeg           ; Both X and Y are positive
        clr DeltaYNeg
        bra DetermineQuadDone
CheckQuad2:
        cmp #128t               ; Compare angle to 180 degrees
        bhi CheckQuad3          ; Greater than 180, check quadrant 3
        mov #$FF,DeltaXNeg      ; X is negative
        clr DeltaYNeg           ; Y is positive
        bra DetermineQuadDone
CheckQuad3:
        cmp #192t               ; Compare angle to 270 degrees
        bhi CheckQuad4          ; Greater than 270, must be quadrant 4
        mov #$FF,DeltaXNeg      ; X and Y are negative
        mov #$FF,DeltaYNeg
        bra DetermineQuadDone
CheckQuad4:
        clr DeltaXNeg           ; X is positive
        mov #$FF,DeltaYNeg      ; Y is negative
DetermineQuadDone:
        rts

**************************************************************
* UpdatePosition - Updates 'Absolute X' and 'Absolute Y' with
*                  values from 'Delta X' and 'Delta Y'
*                  Args: none
**************************************************************
UpdatePosition:
        ; Save registers used by function
        psha

        ; Update X coordinate first
        lda DeltaXNeg           ; Check to see if DeltaX is negative
        bne DeltaXIsNeg

        ; Delta X is positive
        lda {DeltaX+1}          ; Load LSB of DeltaX
        add {AbsoluteX+3}       ; Add LSB of AbsoluteX
        sta {AbsoluteX+3}       ; Store result
        lda DeltaX              ; Load 2nd byte of DeltaX
        adc {AbsoluteX+2}       ; Add LSB of AbsoluteX + carry from previous
        sta {AbsoluteX+2}       ; Store result
        clra                    ; Clear A (no more bytes from DeltaX)
        adc {AbsoluteX+1}       ; Add carry from 2nd byte to 3rd byte
        sta {AbsoluteX+1}       ; Store result
        clra                    ; Clear A (no more bytes from DeltaX)
        adc AbsoluteX           ; Add carry from 3rd byte to MSB
        sta AbsoluteX           ; Store result
        bra DoDeltaY            ; Go do the Y component now

DeltaXIsNeg:
        ; Delta X is negative
        lda {AbsoluteX+3}       ; Load LSB of AbsoluteX
        sub {DeltaX+1}          ; Subtract LSB of DeltaX
        sta {AbsoluteX+3}       ; Store result
        lda {AbsoluteX+2}       ; Load 2nd byte of AbsoluteX
        sbc DeltaX              ; Subtract 2nd byte of DeltaX and carry bit
        sta {AbsoluteX+2}       ; Store result
        lda {AbsoluteX+1}       ; Load 3rd byte of AbsoluteX
        sbc #0                  ; Subtract carry bit from previous
        sta {AbsoluteX+1}       ; Store result
        lda AbsoluteX           ; Load MSB of AbsoluteX
        sbc #0                  ; Subtract carry bit from previous
        sta AbsoluteX           ; Store result

DoDeltaY:
        ; Now update Y coordinate
        lda DeltaYNeg           ; Check to see if DeltaY is negative
        bne DeltaYIsNeg

        ; Delta Y is positive
        lda {DeltaY+1}          ; Load LSB of DeltaY
        add {AbsoluteY+3}       ; Add LSB of AbsoluteY
        sta {AbsoluteY+3}       ; Store result
        lda {DeltaY}            ; Load 2nd byte of DeltaY
        adc {AbsoluteY+2}       ; Add LSB of AbsoluteY + carry from previous
        sta {AbsoluteY+2}       ; Store result
        clra                    ; Clear A (no more bytes from DeltaY)
        adc {AbsoluteY+1}       ; Add carry from 2nd byte to 3rd byte
        sta {AbsoluteY+1}       ; Store result
        clra                    ; Clear A (no more bytes from DeltaY)
        adc AbsoluteY           ; Add carry from 3rd byte to MSB
        sta AbsoluteY           ; Store result
        bra UpdateDone          ; All done

DeltaYIsNeg:
        ; Delta Y is negative
        lda {AbsoluteY+3}       ; Load LSB of AbsoluteY
        sub {DeltaY+1}          ; Subtract LSB of DeltaY
        sta {AbsoluteY+3}       ; Store result
        lda {AbsoluteY+2}       ; Load 2nd byte of AbsoluteY
        sbc DeltaY              ; Subtract 2nd byte of DeltaY and carry bit
        sta {AbsoluteY+2}       ; Store result
        lda {AbsoluteY+1}       ; Load 3rd byte of AbsoluteY
        sbc #0                  ; Subtract carry bit from previous
        sta {AbsoluteY+1}       ; Store result
        lda AbsoluteY           ; Load MSB of AbsoluteY
        sbc #0                  ; Subtract carry bit from previous
        sta AbsoluteY           ; Store result

UpdateDone:
        ; Restore registers
        pula
        rts


**************************************************************
**************************************************************
* Helper Functions
**************************************************************
**************************************************************

**************************************************************
* UMult16 - Unsigned 16x16 multiply
**************************************************************
UMult16:
        ; Save registers that we're going to use
        psha
        pshh
        pshx
        ais #-6                 ; Reserve 6 bytes on stack for local storage
        clr 6,SP                ; Clear the byte used for multiplication carry
        ; Calculate first intermediate result
        ldx {TempWord1+1}       ; Load X with multiplier LSB
        lda {TempWord2+1}       ; Load A with multiplicand LSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta {TempLWord+3}       ; Store LSB of final result
        ldx TempWord1           ; Load X with multiplier MSB
        lda {TempWord2+1}       ; Load A with multiplicand LSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 2,SP                ; Store 2nd byte of interm. result 1
        bcc UMult16_2           ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 1
UMult16_2:
        stx 1,SP                ; Store MSB of interm. result 1
        clr 6,SP                ; Clear the byte used for multiplication carry
        ; Calculate second intermediate result
        ldx {TempWord1+1}       ; Load X with multiplier LSB
        lda TempWord2           ; Load A with multiplicand MSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta 5,SP                ; Store LSB of interm. result 2
        ldx TempWord1           ; Load X with multiplier MSB
        lda TempWord2           ; Load A with multiplicand MSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 4,SP                ; Store 2nd byte of interm. result 2
        bcc UMult16_3           ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 2
UMult16_3:
        stx 3,SP                ; Store MSB of interm. result 2
        ; Add interm. result 1 & 2 and store total in TempLWord
        lda 2,SP                ; Load A with 2nd byte of result 1
        add 5,SP                ; Add LSB of result 2
        sta {TempLWord+2}       ; Store 2nd byte of final result
        lda 1,SP                ; Load A with MSB of result 1
        adc 4,SP                ; Add w/carry 2nd byte of result 2
        sta {TempLWord+1}       ; Store 3rd byte of final result
        lda 3,SP                ; Load A with MSB from result 2
        adc #0                  ; Add carry from previous result
        sta TempLWord           ; Store MSB of final result
        ; Restore registers
        ais #6                  ; Deallocate local storage from stack
        pulx
        pulh
        pula
        rts

**************************************************************
* IUMult16 - Unsigned 16x16 multiply (for interrupts)
**************************************************************
IUMult16:
        ; Save registers that we're going to use
        psha
        pshh
        pshx
        ais #-6                 ; Reserve 6 bytes on stack for local storage
        clr 6,SP                ; Clear the byte used for multiplication carry
        ; Calculate first intermediate result
        ldx {ITempWord1+1}      ; Load X with multiplier LSB
        lda {ITempWord2+1}      ; Load A with multiplicand LSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta {ITempLWord+3}      ; Store LSB of final result
        ldx ITempWord1          ; Load X with multiplier MSB
        lda {ITempWord2+1}      ; Load A with multiplicand LSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 2,SP                ; Store 2nd byte of interm. result 1
        bcc IUMult16_2          ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 1
IUMult16_2:
        stx 1,SP                ; Store MSB of interm. result 1
        clr 6,SP                ; Clear the byte used for multiplication carry
        ; Calculate second intermediate result
        ldx {ITempWord1+1}      ; Load X with multiplier LSB
        lda ITempWord2          ; Load A with multiplicand MSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta 5,SP                ; Store LSB of interm. result 2
        ldx ITempWord1          ; Load X with multiplier MSB
        lda ITempWord2          ; Load A with multiplicand MSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 4,SP                ; Store 2nd byte of interm. result 2
        bcc IUMult16_3          ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 2
IUMult16_3:
        stx 3,SP                ; Store MSB of interm. result 2
        ; Add interm. result 1 & 2 and store total in TempLWord
        lda 2,SP                ; Load A with 2nd byte of result 1
        add 5,SP                ; Add LSB of result 2
        sta {ITempLWord+2}      ; Store 2nd byte of final result
        lda 1,SP                ; Load A with MSB of result 1
        adc 4,SP                ; Add w/carry 2nd byte of result 2
        sta {ITempLWord+1}      ; Store 3rd byte of final result
        lda 3,SP                ; Load A with MSB from result 2
        adc #0                  ; Add carry from previous result
        sta ITempLWord          ; Store MSB of final result
        ; Restore registers
        ais #6                  ; Deallocate local storage from stack
        pulx
        pulh
        pula
        rts

**************************************************************
* GetByte - Get byte and return it in accumulator
**************************************************************
GetByte:
        sta COPCTL              ; Kick watchdog
        lda SCS1                ; Check to see if character is available
        and #$20                ; $20 = Receiver Full bit
        beq GetByte
        lda SCDR                ; Read character
        rts

**************************************************************
* GetByteWithTimeout - Get byte and return it in accumulator
*                      Wait only for 2 milliseconds before
*                      giving up.  If we timeout, set the
*                      carry bit to indicate error.
**************************************************************
GetByteWithTimeout:
        mov #$00,T2MODH
        mov #$4D,T2MODL         ; Set up for 2 millisecond timeout
        mov #$16,T2SC           ; Timer 1 Started
GetByteTimeoutLoop:
        brset 5,SCS1,GotByte    ; Check for received waypoint byte
        brclr 7,T2SC,GetByteTimeoutLoop
        ; At this point we did not receive the byte in time
        sec                     ; No byte received so set carry bit
        lda LED_PORT
        eor #LED_GETB_TIMEOUT
        sta LED_PORT
        rts
GotByte:
        mov #$36,T2SC           ; Reset Timer 1
        lda SCDR
        clc                     ; Clear carry bit to indicate no timeout
        rts

**************************************************************
* SendByte - Send the byte in Accum out the serial port
**************************************************************
SendByte:
        ; First check to see if byte to send is equal to
        ; one of the command bytes.  If so, increment it
        ; by one to prevent seeing our own echo and getting
        ; into trouble (stupid robot controller...)
        cmp #REQ_THETA_ONLY     ; Check if byte to send is Theta Only cmd
        bne SendByteCheckX
        inca
        bra SendByteNow

SendByteCheckX:
        cmp #REQ_X_ONLY         ; Check if byte to send is X Only cmd
        bne SendByteCheckY
        inca
        bra SendByteNow

SendByteCheckY:
        cmp #REQ_Y_ONLY         ; Check if byte to send is Y Only cmd
        bne SendBCheckSetWP
        inca
        bra SendByteNow

SendBCheckSetWP:
        cmp #REQ_SET_WAYPT      ; Check if byte to send is Y Only cmd
        bne SendBCheckGetWP
        inca
        bra SendByteNow

SendBCheckGetWP:
        cmp #REQ_GET_WAYPT      ; Check if byte to send is Y Only cmd
        bne SendBCheckSetXYT
        inca
        bra SendByteNow

SendBCheckSetXYT:
        cmp #REQ_SET_XYT        ; Check if byte to send is Y Only cmd
        bne SendByteNow
        inca

SendByteNow:
        sta COPCTL              ; Kick watchdog
        brclr 7,SCS1,SendByteNow; Wait until xmitter is ready.
        sta SCDR                ; Xmit it our serial port
        rts


**************************************************************
**************************************************************
* Interrupt Service Routines
**************************************************************
**************************************************************

**************************************************************
* GyroIsr - used to sample the gyroscope and update our
*           angular robot position
**************************************************************
GyroIsr:
        inc GyroLoopCount       ; Increment our loop count
        bne GyroNoToggleLED     ; If != 0, don't toggle LED
        lda LED_PORT            ; Load LED settings
        eor #LED_GYRO_INT       ; Toggle LED 1
        sta LED_PORT            ; Set the LEDs
GyroNoToggleLED:
        lda T1SC                ; Load T1SC to clear TOF bit
        mov #$56,T1SC           ; Start timer 1 (prescalar == x / 64)
        clr GyroValNeg          ; Initialize variable
        lda #ADC_GYRO_CHAN      ; Load the channel # that the gyro is on
        ora #ADC_ENABLE         ; Set the enable bit
        sta ADSCR               ; Start an ADC conversion on the gyro chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        lda ADR                 ; Read gyro value
        sta GyroValue
        sub GyroCenter          ; Convert so it's centered around 0
        cmp #1t
        bgt GyroPositive        ; Turning in a positive direction
        cmp #-1t
        blt GyroNegative        ; Turning in a negative direction
        bra GyroDone            ; Not turning at all
GyroNegative:
        mov #$FF,GyroValNeg     ; Signal that the gyro value is negative
        nega                    ; Convert offset to a positive number for
                                ;   processing
        mov #GYRO_NDEGS_HIGH,ITempWord2
        mov #GYRO_NDEGS_LOW,{ITempWord2+1}
        bra GyroPositive2
GyroPositive:
        mov #GYRO_PDEGS_HIGH,ITempWord2
        mov #GYRO_PDEGS_LOW,{ITempWord2+1}
GyroPositive2:
        clr ITempWord1
        sta {ITempWord1+1}      ; Store A in multiplier
        jsr IUMult16            ; Multiply gyro offset by (binary degrees
                                ;   per tick per sample)
        lda GyroValNeg          ; Check to see if we need to add or subtract
        beq GyroSubtract        ; Need to add (when gyro is negative, we want to
                                ; *increment* our position)

        ; Add new offset to current angular position
        lda {ITempLWord+3}      ; Load LSB of offset
        add {RobotTheta+3}      ; Add LSB of RobotTheta
        sta {RobotTheta+3}      ; Store result
        lda {ITempLWord+2}      ; Load 2nd byte of offset
        adc {RobotTheta+2}      ; Add 2nd byte of RobotTheta + carry
        sta {RobotTheta+2}      ; Store result
        lda {ITempLWord+1}      ; Load 3rd byte of offset
        adc {RobotTheta+1}      ; Add carry from 2nd byte to 3rd byte
        sta {RobotTheta+1}      ; Store result
        lda ITempLWord          ; Load MSB of offset
        adc RobotTheta          ; Add carry from 3rd byte to MSB
        sta RobotTheta          ; Store result
        bra GyroDone            ; Go do the Y component now

GyroSubtract:
        ; Subtract new offset from angular position
        lda {RobotTheta+3}      ; Load LSB of RobotTheta
        sub {ITempLWord+3}      ; Subtract LSB of offset
        sta {RobotTheta+3}      ; Store result
        lda {RobotTheta+2}      ; Load 2nd byte of RobotTheta
        sbc {ITempLWord+2}      ; Subtract 2nd byte of offset and carry bit
        sta {RobotTheta+2}      ; Store result
        lda {RobotTheta+1}      ; Load 3rd byte of RobotTheta
        sbc {ITempLWord+1}      ; Subtract 3rd byte of offset + carry
        sta {RobotTheta+1}      ; Store result
        lda RobotTheta          ; Load MSB of RobotTheta
        sbc ITempLWord          ; Subtract MSB of offset + carry
        sta RobotTheta          ; Store result

GyroDone:
        rti

**************************************************************
* RCRequestIsr - used when the Robot Controller requests a
*                positioning update
**************************************************************
SetPositionRequest:
        jsr GetByteWithTimeout  ; Wait 2 milliseconds for a byte
        bcs RCRequestDone       ; If carry bit set then GetByte timed out
        sta {AbsoluteX+1}
        jsr GetByteWithTimeout  ; Wait 2 milliseconds for another byte
        bcs RCRequestDone       ; If carry bit set then GetByte timed out
        sta {AbsoluteY+1}
        jsr GetByteWithTimeout  ; Wait 2 milliseconds for last byte
        bcs RCRequestDone       ; If carry bit set then GetByte timed out
        sta RobotTheta
        mov #$FF,PosInitialized ; Indicate that our position has been inited
        lda LED_PORT            ; Signal by an LED that we're linked up
        eor #LED_XYT_RECVD
        sta LED_PORT
        bra RCRequestDone

RCRequestIsr:
        lda LED_PORT            ; Read the LED states
        eor #LED_RX             ; Toggle the lowest bit
        sta LED_PORT            ; Change the display
        jsr GetByte             ; Read the byte that was sent to us
        mov #$00,T2MODH
        mov #$13,T2MODL         ; Set up for 0.5 millisecond timeout
        cmp #REQ_SET_WAYPT      ; Check for request to set waypoint
        beq SetWayptRequest     ; Handle it if equal
        cmp #REQ_X_ONLY         ; Check for X request
        beq XRequest            ; Handle it if equal
        cmp #REQ_Y_ONLY         ; Check for Y request
        beq YRequest            ; Handle it if equal
        cmp #REQ_THETA_ONLY     ; Check for Theta request
        beq ThetaRequest        ; Handle it if equal
        cmp #REQ_GET_WAYPT      ; Check for request to get waypoint
        beq GetWayptRequest     ; Handle it if equal
        ; Check for extended commands
        brset 0,PosInitialized,NoExtendedCmds
        cmp #REQ_SET_XYT        ; Check for cmd to set current position
        beq SetPositionRequest  ; Handle it if equal
NoExtendedCmds:
        lda LED_PORT            ; Toggle the UNKNOWN_CMD LED
        eor #LED_UNKNOWN_CMD
        sta LED_PORT

RCRequestDone:
        lda SCS1                ; Flush receive buffer
        lda SCDR
        rti

SetWayptRequest:
        jsr GetByteWithTimeout  ; Wait 2 milliseconds for a byte
        bcs RCRequestDone       ; If carry bit set then GetByte timed out
        sta RCCurrentWaypt      ; Store current waypoint
        bra RCRequestDone

GetWayptRequest:
        ; Sleep for some time before responding
        mov #$16,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        lda RCCurrentWaypt      ; Load currently stored waypoint number
        jsr SendByte            ; Send it out
        bra RCRequestDone

XRequest:
        ; Sleep for some time before responding
        mov #$16,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        ; Check to see if the number we're sending to the RC has rolled over.
        ; If so, just keep sending back 0 or 255 as appropriate.
        lda AbsoluteX
        cmp #127t               ; Check to see if AbsoluteX+1 has rolled over
        beq XNoRollover
        blo SendZero
        bhi Send255
XNoRollover:
        ; Check if our position has been initialized and if not
        ; send back 0
        brclr 0,PosInitialized,SendZero
        ; Now round the integer that we'll be sending to the RC
        ; and send it out
        lda {AbsoluteX+1}       ; Load LSB of integer portion of X
        ldx {AbsoluteX+2}       ; Load MSB of fraction portion of X
        aslx                    ; Move MSB of X into carry bit
        adc #0                  ; Add carry bit to A
        jsr SendByte            ; Send it out
        bra RCRequestDone

YRequest:
        mov #$16,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        ; Check to see if the number we're sending to the RC has rolled over.
        ; If so, just keep sending back 0 or 255 as appropriate.
        lda AbsoluteY
        cmp #127t               ; Check to see if AbsoluteY+1 has rolled over
        beq YNoRollover
        blo SendZero
        bhi Send255
YNoRollover:
        ; Check if our position has been initialized and if not
        ; send back 0
        brclr 0,PosInitialized,SendZero
        ; Now round the integer that we'll be sending to the RC
        ; and send it out
        lda {AbsoluteY+1}       ; Load LSB of integer portion of Y
        ldx {AbsoluteY+2}       ; Load MSB of fraction portion of Y
        aslx                    ; Move MSB of X into carry bit
        adc #0                  ; Add carry bit to A
        jsr SendByte            ; Send it out
        bra RCRequestDone

ThetaRequest:
        mov #$16,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        ; Check if our position has been initialized and if not
        ; send back 0
        brclr 0,PosInitialized,SendZero
        ; Now round the integer that we'll be sending to the RC
        ; and send it out
        lda RobotTheta          ; Load LSB of integer portion of Theta
        ldx {RobotTheta+1}
        aslx                    ; Move MSB of X into carry bit
        adc #0                  ; Add carry bit to A
        jsr SendByte            ; Send it out
        bra RCRequestDone

SendZero:
        lda #0t
        jsr SendByte
        bra RCRequestDone

Send255:
        lda #255t
        jsr SendByte
        bra RCRequestDone

**************************************************************
* DummyIsr - used when we don't want to do anything in
*            response to an interrupt type
**************************************************************
DummyIsr:
        rti

**************************************************************
**************************************************************
**************************************************************
* Vectors
**************************************************************
**************************************************************
**************************************************************
        org VectorStart
        dw DummyIsr             ; Time Base Vector
        dw DummyIsr             ; ADC Conversion Complete
        dw DummyIsr             ; Keyboard Vector
        dw DummyIsr             ; SCI Transmit Vector
        dw RCRequestIsr         ; SCI Receive Vector
        dw DummyIsr             ; SCI Error Vector
        dw DummyIsr             ; SPI Transmit Vector
        dw DummyIsr             ; SPI Receive Vector
        dw DummyIsr             ; TIM2 Overflow Vector
        dw DummyIsr             ; TIM2 Channel 1 Vector
        dw DummyIsr             ; TIM2 Channel 0 Vector
        dw GyroIsr              ; TIM1 Overflow Vector
        dw DummyIsr             ; TIM1 Channel 1 Vector
        dw DummyIsr             ; TIM1 Channel 0 Vector
        dw DummyIsr             ; PLL Vector
        dw DummyIsr             ; IRQ1 Vector
        dw DummyIsr             ; SWI Vector
        dw Main                 ; Reset Vector
