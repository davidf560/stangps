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
**************************************************************

$INCLUDE 'gpgtregs.inc'         ; Processor defines

; Memory Map Constants
RAMStart                EQU $0040
FLASHStart              EQU $8000
VectorStart             EQU $FFDC

; Photo Sensor Constants
DISTANCE_RES_LOW        EQU 0t
DISTANCE_RES_HIGH       EQU $1
PHOTO_SENSOR_PORT       EQU PORTD

; Analog to Digital Converter Constants
ADC_POT_CHAN            EQU $1
ADC_GYRO_CHAN           EQU $0
ADC_ENABLE              EQU $00

; Potentiometer Constants
POT_CENTER_VAL          EQU 127t

; Gyroscope Constants
GYRO_DEGS_HIGH          EQU 0t
GYRO_DEGS_LOW           EQU 28t

; Serial Communications Constants
CC_ATTENTION_BYTE       EQU $00

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
TempGyroVal:
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
        jmp QIncrement          ; 00 01 -> Forward rotation
        jmp QDecrement          ; 00 10 -> Reverse rotation
        jmp QError              ; 00 11 -> Shouldn't happen
        jmp QDecrement          ; 01 00 -> Reverse rotation
        jmp QNoChange           ; 01 01 -> No change
        jmp QError              ; 01 10 -> Shouldn't happen
        jmp QIncrement          ; 01 11 -> Forward rotation
        jmp QIncrement          ; 10 00 -> Forward rotation
        jmp QError              ; 10 01 -> Shouldn't happen
        jmp QNoChange           ; 00 00 -> No change
        jmp QDecrement          ; 10 11 -> Reverse rotation
        jmp QError              ; 11 00 -> Shouldn't happen
        jmp QDecrement          ; 11 01 -> Reverse rotation
        jmp QIncrement          ; 11 10 -> Forward rotation
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
* QIncrement - Wheels turned forward, so increment
*              the position
**************************************************************
QIncrement:
        lda #DISTANCE_RES_LOW   ; Low byte of distance resolution
        sta {Distance+1}
        lda #DISTANCE_RES_HIGH  ; High byte of distance resolution
        sta Distance
        clr DistanceNeg         ; Distance is not negative
        rts                     ; Return to main loop

**************************************************************
* QDecrement - Wheels turned backwards, so decrement
*              the position
**************************************************************
QDecrement:
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
* InitSCI - Turns on the asyncronous communications port
*           for TX, RX at specified baud N81
**************************************************************
InitSCI:
        lda SCS1
;        mov #$04,SCBR           ; Baud Rate = 9600 (at 9.8MHz clock)
        mov #$02,SCBR           ; Baud Rate = 38400 (at 9.8MHz clock)
        mov #$40,SCC1           ; Enable the SCI peripheral
        mov #$2C,SCC2           ; Enable the SCI for TX and RX
        mov #$00,SCC3           ; No SCC error interrupts, please
        rts

**************************************************************
* InitRAM - Checks reset reason to determine if RAM is still
*           valid from prior to reset.  If not, all RAM vars
*           are initialized.
**************************************************************
InitRAM:
        lda SRSR                ; Read Reset Status Register
        and #%11000110          ; Check for a reason that would invalidate RAM
        beq InitRAMDone         ; RAM should still be good, don't initialize
        mov #0,RobotAngle
        mov #0,AbsoluteX
        mov #1,{AbsoluteX+1}
        mov #0,{AbsoluteX+2}
        mov #0,{AbsoluteX+3}
        mov #0,AbsoluteY
        mov #1,{AbsoluteY+1}
        mov #0,{AbsoluteY+2}
        mov #0,{AbsoluteY+3}
        mov #0,AbsoluteHeading
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
        lda PHOTO_SENSOR_PORT
        and #$03
        asla
        asla
        sta LastPhotoVals
        mov #0,NumQuadErrors
InitRAMDone:
        rts

**************************************************************
* InitPorts - Initializes all I/O ports and sets appropriate
*             pullup enable registers
**************************************************************
InitPorts:
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
        mov #$01,CONFIG1        ; Disable COP
        mov #$00,CONFIG2
        rsp                     ; Reset stack pointer
        clra                    ; Clear accumulator
        clrh                    ; Clear high index register
        clrx                    ; Clear index register

        ; Initialize subsystems
        jsr InitPorts           ; Initialize general purpose I/O ports
        jsr InitSCI             ; Initialize the serial comm. interface
        jsr InitRAM             ; Initialize RAM variables
        jsr InitADC             ; Initialize the analog to digital converter
        jsr InitGyroTimer       ; Initialize the gyroscope timer
        cli                     ; Enable interrupts

MainLoop:
        jsr ReadPhotos          ; Find our 'Distance Delta'
        jsr ReadPot             ; Find our 'Crab Angle'
        jsr ComputeHeading      ; Compute heading using our 'Crab Angle'
                                ; and our 'Robot Theta'

        ;; DEBUG
        lda #0t
        ;; END DEBUG

        eor DistanceNeg         ; Correct the angle if we're going backwards

        jsr DetermineQuad       ; Determine what quadrant our vector is in

        jsr ComputeDeltas       ; Find our 'Delta X' and 'Delta Y' using
                                ; 'Dist Delta', 'Crab Angle' and 'Robot Angle'
        jsr UpdatePosition      ; Updates our field position using
                                ; 'Delta X' and 'Delta Y'

        ;; Debug
        lda #$58                ; ASCII 'X'
        bsr SendByte
        lda AbsoluteX
        bsr SendByte
        lda AbsoluteX+1
        bsr SendByte
        lda AbsoluteX+2
        bsr SendByte
        lda AbsoluteX+3
        bsr SendByte
        
        ;;debug
        bra MainLoop

        lda #$59                ; ASCII 'Y'
        bsr SendByte
        lda AbsoluteY
        bsr SendByte
        lda AbsoluteY+1
        bsr SendByte
        lda AbsoluteY+2
        bsr SendByte
        lda AbsoluteY+3
        bsr SendByte
        
        lda #$47                ; ASCII 'G'
        bsr SendByte
        clra
        bsr SendByte
        bsr SendByte
        lda TempGyroVal
        bsr SendByte
        clra
        bsr SendByte

        lda #$45                ; ASCII 'E'
        bsr SendByte
        clra
        bsr SendByte
        bsr SendByte
        lda {RobotTheta+1}
        bsr SendByte
        lda {RobotTheta+2}
        bsr SendByte

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
* ReadPot - Reads the potentiometers
**************************************************************
ReadPot:
        lda #ADC_POT_CHAN       ; Load the channel # that the pot is on
        ora #ADC_ENABLE         ; Set the enable bit
        sei                     ; Turn off interrupts
        sta ADSCR               ; Start an ADC conversion on the pot chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        lda ADR                 ; Read ADC value
        cli
        rts

**************************************************************
* ComputeHeading - Using 'Robot Theta' and 'Crab Angle',
*                  calculate our heading
*                  Args: Crab angle in A, Robot Theta in
*                  global variable
*                  Returns: Heading in A
**************************************************************
ComputeHeading:
        sub #POT_CENTER_VAL     ; Center our pot reading around 0
        add {RobotTheta+1}      ; Add in our robot's orientation
                                ; (measured in "binary degrees")
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
        lda T1SC                ; Load T1SC to clear TOF bit
        mov #$56,T1SC           ; Start timer 1 (prescalar == x / 64)
        clr GyroValNeg          ; Initialize variable
        lda #ADC_GYRO_CHAN      ; Load the channel # that the gyro is on
        ora #ADC_ENABLE         ; Set the enable bit
        sta ADSCR               ; Start an ADC conversion on the gyro chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        lda ADR                 ; Read gyro value
        sta TempGyroVal
        sub #126t               ; Convert so it's centered around 0
        cmp #1t
        bgt GyroPositive        ; Turning in a positive direction
        cmp #-1t
        blt GyroNegative        ; Turning in a negative direction
        bra GyroDone            ; Not turning at all
GyroNegative:
        mov #$FF,GyroValNeg     ; Signal that the gyro value is negative
        nega                    ; Convert offset to a positive number for
                                ;   processing
GyroPositive:
        clr ITempWord1
        sta {ITempWord1+1}      ; Store A in multiplier
        mov #GYRO_DEGS_HIGH,ITempWord2
        mov #GYRO_DEGS_LOW,{ITempWord2+1}
        jsr IUMult16            ; Multiply gyro offset by (binary degrees
                                ;   per tick per sample)
        lda GyroValNeg          ; Check to see if we need to add or subtract
        bne GyroSubtract        ; Need to subtract

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
RCRequestIsr:
        jsr GetByte             ; Read the byte that was sent to us
        cmp #CC_ATTENTION_BYTE  ; Compare it to our attention byte
        bne RCRequestDone       ; Not for us - ignore it
        
        ; RC has requested that we send position data, so we'll
        ; do just that
        lda {AbsoluteX+2}       ; Load LSB of integer portion of X
        jsr SendByte            ; Send it out
        lda {AbsoluteY+2}       ; Load LSB of integer portion of Y
        jsr SendByte            ; Send it out
        lda {RobotTheta+1}      ; Load LSB of integer portion of Theta
        jsr SendByte            ; Send it out
RCRequestDone:
        rti
        
        
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
