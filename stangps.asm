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
**************************************************************

$INCLUDE 'gpgtregs.inc'         ; Processor defines

; Memory Map Constants
RAMStart                EQU $0040
FLASHStart              EQU $8000
VectorStart             EQU $FFDC

; Display (LED Bargraph) Constants
LED_PORT                EQU PORTA
LED_RX                  EQU $01
LED_UNKONWN_CMD         EQU $02
LED_VALID_CMD           EQU $04
LED_GYRO_INT            EQU $08
LED_UNUSED1             EQU $10
LED_UNUSED2             EQU $20
LED_UNUSED3             EQU $40
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
POT_CENTER_VAL          EQU 124t
POT_MAX_RIGHT_VAL       EQU 12t
POT_MAX_OFFSET          EQU 128t
BRADS_PER_TICK_H        EQU 1t
BRADS_PER_TICK_L        EQU 37t
; PROTO ROBOT
;POT_CENTER_VAL          EQU 127t
;POT_MAX_RIGHT_VAL       EQU 13t
;POT_MAX_OFFSET          EQU 128t
;BRADS_PER_TICK_H        EQU 1t
;BRADS_PER_TICK_L        EQU 30t

; Gyroscope Constants
; First set is for 64 degs/sec gyro
;GYRO_PDEGS_HIGH         EQU $1e
;GYRO_PDEGS_LOW          EQU $48
;GYRO_NDEGS_HIGH         EQU $1e
;GYRO_NDEGS_LOW          EQU $48
; This set is for 75 degs/sec gyro (needs different factor
; for positive versus negative for some reason)
GYRO_PDEGS_HIGH         EQU $23
GYRO_PDEGS_LOW          EQU $AC
GYRO_NDEGS_HIGH         EQU $23
GYRO_NDEGS_LOW          EQU $1F

; Serial Communications Constants
REQ_XYTHETA             EQU 180t
REQ_THETA_ONLY          EQU 181t
REQ_GET_WAYPT           EQU 182t
REQ_SET_WAYPT           EQU 183t
REQ_EVERYTHING          EQU 184t

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

**************************************************************
**************************************************************
**************************************************************
* Start of FLASH
**************************************************************
**************************************************************
**************************************************************
        org FLASHStart
$INCLUDE 'sinetable.inc'


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
* InitWait - Just wait for a period of time and display a
*            countdown sequence on the LEDs
*            (this is used to allow the gyro to stablize
*            before we begin reading it)
**************************************************************
InitWait:
        clrx                    ; Clear the index register
        clr LED_PORT            ; Turn off all LEDs
        mov #$4B,T2MODH
        mov #$00,T2MODL         ; Set up for .5 second interrupts
        mov #$16,T2SC           ; Start timer 1 (prescalar == x / 64)
        brclr 7,T2SC,$          ; Busy loop until timer expires
        mov #$80,LED_PORT       ; Light up MSB of LED bargraph
ContinueWait:
        mov #$16,T2SC           ; Start timer 1 (prescalar == x / 64)
        brclr 7,T2SC,$          ; Busy loop until timer expires
        lda LED_PORT            ; Read our LED status
        cmp #$FF                ; Check to see if they're all lit
        beq InitWaitDone
        asr LED_PORT            ; Shift down one more bit
        bra ContinueWait
InitWaitDone:
        clr LED_PORT            ; Turn off all LEDs
        rts


**************************************************************
* InitADC - Initializes the Analog to Digital Converter
**************************************************************
InitADC:
        mov #$60,ADCLK          ; Select external clock, divided by 8
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
* InitGyro - Determines gyro center and allows gyro to
*            settle down before continuing
**************************************************************
InitGyro:
        mov #$FF,LED_PORT       ; Light up all LEDs to signal sampling of gyro
        clrx                    ; Clear the index register
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
        bne FindGyroCenter      ; Take another sample
        ; Done sampling, now average
        brclr 7,GyroCenter,NoRound
        inc GyroCenter          ; Round up because MSB of lower byte was set
NoRound:
        clr LED_PORT            ; Turn off the LEDs to indicate center found
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
* CheckRSR - Checks the Reset Status Register and sets
*            the Carry Bit if a warm reset is appropriate
**************************************************************
CheckRSR:
        lda SRSR                ; Read Reset Status Register
        sta ResetStatus         ; Store for later
        and #%11000110          ; Check for a reason that would invalidate RAM
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
* InitRAM - Checks reset reason to determine if RAM is still
*           valid from prior to reset.  If not, all RAM vars
*           are initialized.
**************************************************************
InitRAM:
        mov #0,RobotAngle
        mov #0,AbsoluteX
        mov #127t,{AbsoluteX+1}
        mov #0,{AbsoluteX+2}
        mov #0,{AbsoluteX+3}
        mov #0,AbsoluteY
        mov #127t,{AbsoluteY+1}
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
        lda PHOTO_SENSOR_PORT
        and #$03
        asla
        asla
        sta LastPhotoVals
        mov #0,NumQuadErrors
        rts

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
        ldx {ITempWord1+1}       ; Load X with multiplier LSB
        lda {ITempWord2+1}       ; Load A with multiplicand LSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta {ITempLWord+3}       ; Store LSB of final result
        ldx ITempWord1           ; Load X with multiplier MSB
        lda {ITempWord2+1}       ; Load A with multiplicand LSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 2,SP                ; Store 2nd byte of interm. result 1
        bcc IUMult16_2           ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 1
IUMult16_2:
        stx 1,SP                ; Store MSB of interm. result 1
        clr 6,SP                ; Clear the byte used for multiplication carry
        ; Calculate second intermediate result
        ldx {ITempWord1+1}       ; Load X with multiplier LSB
        lda ITempWord2           ; Load A with multiplicand MSB
        mul                     ; Multiply
        stx 6,SP                ; Save carry from multiply
        sta 5,SP                ; Store LSB of interm. result 2
        ldx ITempWord1           ; Load X with multiplier MSB
        lda ITempWord2           ; Load A with multiplicand MSB
        mul                     ; Multiply
        add 6,SP                ; Add carry from previous multiply
        sta 4,SP                ; Store 2nd byte of interm. result 2
        bcc IUMult16_3           ; Check for carry from addition
        incx                    ; Increment MSB of interm. result 2
IUMult16_3:
        stx 3,SP                ; Store MSB of interm. result 2
        ; Add interm. result 1 & 2 and store total in TempLWord
        lda 2,SP                ; Load A with 2nd byte of result 1
        add 5,SP                ; Add LSB of result 2
        sta {ITempLWord+2}       ; Store 2nd byte of final result
        lda 1,SP                ; Load A with MSB of result 1
        adc 4,SP                ; Add w/carry 2nd byte of result 2
        sta {ITempLWord+1}       ; Store 3rd byte of final result
        lda 3,SP                ; Load A with MSB from result 2
        adc #0                  ; Add carry from previous result
        sta ITempLWord           ; Store MSB of final result
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
        lda SCS1                ; Check to see if character is available
        and #$20                ; $20 = Receiver Full bit
        beq GetByte
        lda SCDR                ; Read character
        rts

**************************************************************
* SendByte - Send the byte in Accum out the serial port
**************************************************************
SendByte:
        brclr 7,SCS1,$          ; Wait until xmitter is ready.
        sta SCDR                ; Xmit it our serial port
        rts


**************************************************************
**************************************************************
* Main - this is the function that will be called upon reset
**************************************************************
**************************************************************
Main:
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

MainLoop:
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
        lda ADR                 ; Read ADC value
        sta PotValue            ; Store in RAM
        cli
        sub #POT_MAX_RIGHT_VAL
        sta {TempWord1+1}       ; Prepare for multiply
        clr TempWord1           ; Clear fraction
        lda #BRADS_PER_TICK_H   ; Load scaling factor as multiplicand
        sta TempWord2
        lda #BRADS_PER_TICK_L
        sta {TempWord2+1}
        jsr UMult16             ; Multiply
        lda {TempLWord+2}       ; Load crab brads into A
        sub #POT_MAX_OFFSET     ; Convert back to straight ahead = 0 brads
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
* DataRequestIsr - used when the Robot Controller requests a
*                  positioning update
**************************************************************
DataRequestIsr:
        mov #$00,T2MODH
        mov #$13,T2MODL         ; Set up for .5 ms timeout
        lda LED_PORT            ; Read the LED states
        eor #LED_RX             ; Toggle the lowest bit
        sta LED_PORT            ; Change the display
        jsr GetByte             ; Read the byte that was sent to us
        cmp #REQ_XYTHETA        ; Compare to X, Y, theta request
        beq XYThetaRequest      ; Request for X, Y, and Theta information
        cmp #REQ_THETA_ONLY     ; Compare to theta only request
        beq ThetaRequest
        cmp #REQ_GET_WAYPT      ; Request to retrieve current waypoint
        beq GetWaypoint
        cmp #REQ_SET_WAYPT      ; Request to set current waypoint
        beq SetWaypoint
        cmp #REQ_EVERYTHING     ; Send back everything we know
        beq AllRequest
        ; If we've reached this point, then the received byte
        ; did not match any of our command bytes - toggle an LED
        ; to indicate that
        lda LED_PORT            ; Read the LED states
        eor #LED_UNKNOWN_CMD    ; Toggle the "Unknown Command" LED
        sta LED_PORT            ; Change the LED display
DataRequestDone:
        ; Clear out the receive buffer to get rid of our own echo if
        ; it's there
        lda SCS1                ; Read SCI Status Register 1
        lda SCDR                ; Read serial data register
        rti

XYThetaRequest:
        ; Sleep for some time before continuing so RC doesn't get swamped
        mov #$06,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        mov #$36,T2SC           ; Reset Timer 1
        lda {AbsoluteX+1}       ; Load LSB of integer portion of X
        brclr 7,{AbsoluteX+2},NoIncX
        inca
NoIncX:
        jsr SendByte            ; Send it out
        ; Sleep for some time before continuing so RC doesn't get swamped
        mov #$06,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        mov #$36,T2SC           ; Reset Timer 1
        lda {AbsoluteY+1}       ; Load LSB of integer portion of Y
        brclr 7,{AbsoluteX+2},NoIncY
        inca
NoIncY:
        jsr SendByte            ; Send it out
ThetaRequest:                   ; Label used if only theta is requested
        lda LED_PORT            ; Read the LED states
        eor #LED_VALID_CMD      ; Toggle the lowest bit
        sta LED_PORT            ; Change the display
        ; Sleep for some time before continuing so RC doesn't get swamped
        mov #$06,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        mov #$36,T2SC           ; Reset Timer 1
        lda RobotTheta          ; Load LSB of integer portion of Theta
        brclr 7,{RobotTheta+1},NoIncTheta
        inca
NoIncTheta:
        jsr SendByte            ; Send it out
        bra DataRequestDone     ; All done

GetWaypoint:
        ; Sleep for some time before continuing so RC doesn't get swamped
        mov #$06,T2SC           ; Timer 1 Started
        brclr 7,T2SC,$          ; Loop if the timer isn't done (bit 7 of T1SC==0)
        mov #$36,T2SC           ; Reset Timer 1
        lda RCCurrentWaypt      ; Load RC's current waypoint
        jsr SendByte            ; Send it out
        bra DataRequestDone     ; All done

SetWaypoint:
        jsr GetByte             ; Read in waypoint # from RC
        sta RCCurrentWaypt      ; Store it in our RAM
        bra DataRequestDone     ; All done

AllRequest:
        lda AbsoluteX
        jsr SendByte
        lda {AbsoluteX+1}
        jsr SendByte
        lda {AbsoluteX+2}
        jsr SendByte
        lda {AbsoluteX+3}
        jsr SendByte
        lda AbsoluteY
        jsr SendByte
        lda {AbsoluteY+1}
        jsr SendByte
        lda {AbsoluteY+2}
        jsr SendByte
        lda {AbsoluteY+3}
        jsr SendByte
        lda RobotTheta
        jsr SendByte
        lda {RobotTheta+1}
        jsr SendByte
        lda {RobotTheta+2}
        jsr SendByte
        lda {RobotTheta+3}
        jsr SendByte
        lda GyroValue
        jsr SendByte
        lda PotBrads
        jsr SendByte
        lda PotValue
        jsr SendByte
        lda ResetStatus
        jsr SendByte
        jmp DataRequestDone     ; All done

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
        dw DataRequestIsr       ; SCI Receive Vector
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
