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
**************************************************************

$INCLUDE 'gpgpregs.inc'         ; Processor defines

**************************************************************
**************************************************************
**************************************************************
* Start of RAM
**************************************************************
**************************************************************
**************************************************************
        org BootRAMStart
FlashRowSource:
        ds $02


**************************************************************
**************************************************************
**************************************************************
* Start of FLASH
**************************************************************
**************************************************************
**************************************************************
        org BootFLASHStart
        
**************************************************************
**************************************************************
* Constants
**************************************************************
**************************************************************
QuadratureTable:
        ; This is a lookup table used by the quadrature
        ; decoding algorithm.
        jmp QNoChange           ; 00 00 -> No change
        jmp QIncrement          ; 00 01 -> Forward rotation
        jmp QDecrement          ; 00 10 -> Reverse rotation
        jmp QError              ; 00 11 -> Shouldn't happen
        jmp QDecrement          ; 01 00 -> Reverse rotation
        jmp QNoChange           ; 00 00 -> No change
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
**************************************************************
* Quadrature Decoding Functions
**************************************************************
**************************************************************

**************************************************************
* QNoChange - There was no change in the light sensor
*             readings so don't update the position
**************************************************************
QNoChange:
        rts                     ; Just return to the caller
        
**************************************************************
* QIncrement - Wheels turned forward, so increment 
*              the position
**************************************************************
QIncrement:
        inca
        rts                     ; Return to main loop
        
**************************************************************
* QDecrement - Wheels turned backwards, so decrement
*              the position
**************************************************************
QDecrement:
        deca
        rts                     ; Return to main loop
        
**************************************************************
* QError - Illegal light sensor state change
**************************************************************
QError:
        ; ???
        rts                     ; Return to main loop                        


**************************************************************
**************************************************************
* Initialization Functions
**************************************************************
**************************************************************

**************************************************************
* InitSCI - Turns on the asyncronous communications port
*           for TX, RX at specified baud N81
**************************************************************
InitSCI:
        mov #$04,SCBR           ; Baud Rate = 9600 (at 9.8MHz clock)
        mov #$40,SCC1           ; Enable the SCI peripheral
        mov #$0C,SCC2           ; Enable the SCI for TX/RX
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
        ;;; Init ram here ;;;
InitRAMDone:
        rts

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
        rsp                     ; Reset stack pointer
        clra                    ; Clear accumulator
        clrh                    ; Clear high index register
        clrx                    ; Clear index register
        
        ; Initialize subsystems
        bsr InitSCI             ; Initialize the Serial Comm. Interface
        bsr InitRAM             ; Initialize RAM variables
        cli                     ; Enable interrupts

MainLoop:
        jsr ReadPhotos          ; Find our 'Distance Delta'
        psha                    ; Store 'Distance Delta' on stack
        jsr ReadPots            ; Find our 'Crab Angle'
        psha                    ; Store 'Crab Angle' on stack
        lda RobotAngle          ; Load the 'Robot Angle' from gyroscope
        psha                    ; Store 'Robot Angle' on stack
        jsr ComputeVector       ; Find our 'Delta X' and 'Delta Y' using
                                ; 'Dist Delta', 'Crab Angle' and 'Robot Angle'
                                ; (from stack)
        psha                    ; Store 'Delta Y' on stack
        jsr UpdatePosition      ; Updates our field position using
                                ; 'Delta X' and 'Delta Y' (from stack)
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
        mul #$03                ; Multiply by 3 for jump table purposes
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
        sta ADSCR               ; Start an ADC conversion on the pot chanel
        brclr 7,ADSCR,$         ; Wait until ADC conversion is complete
        lda ADR                 ; Read ADC value
        rts
        
**************************************************************
* ComputeVector - Using distance and direction, calculate
*                 Delta X and Delta Y
*                 Args: Direction in A, distance on stack
**************************************************************
ComputeVector:
        pshx                    ; Save off index register since we use it
        tax                     ; Transfer direction to index register
        lda SineTable,X         ; Look up sin(direction)
        psha                    ; Save sin(direction) onto stack
        sta {TempWord1+1}       ; Store sin(dir) in TempWord1
        clr TempWord1           ; Clear MSB of TempWord1
        lda 2,SP                ; Load LSB of Distance
        sta {TempWord2+1}       ; Store into TempWord2
        lda 3,SP                ; Load MSB of Distance
        sta TempWord2           ; Store into TempWord2
        jsr UMult16             ; Multiply Distance by sin(Direction)
        lda {TempLWord+1}       ; Load integer portion of Delta Y
        sta DeltaY              ; Store into DeltaY
        lda {TempLWord+2}       ; Load fractional portion of Delta Y
        sta {DeltaY+1}          ; Store into DeltaY
        pula                    ; Reload A with sin(direction)
        coma                    ; Convert A to cos(direction)
        sta {TempWord1+1}       ; Store cos(dir) in TempWord1
        clr TempWord1           ; Clear MSB of TempWord1
        lda 2,SP                ; Load LSB of Distance
        sta {TempWord2+1}       ; Store into TempWord2
        lda 3,SP                ; Load MSB of Distance
        sta TempWord2           ; Store into TempWord2
        jsr UMult16             ; Multiply Distance by cos(Direction)
        lda {TempLWord+1}       ; Load integer portion of Delta X
        sta DeltaX              ; Store into DeltaX
        lda {TempLWord+2}       ; Load fractional portion of Delta X
        sta {DeltaX+1}          ; Store into DeltaX
        pulx                    ; Restore index register
        rts


**************************************************************
**************************************************************
* Interrupt Service Routines
**************************************************************
**************************************************************

**************************************************************
* DummyIsr - used when we don't want to do anything in
*            response to an interrupt type
**************************************************************
        org DummyIsr
        rti


**************************************************************
**************************************************************
**************************************************************
* Vectors
**************************************************************
**************************************************************
**************************************************************
        org VectorStart
        dw  AppVectorStart      ; Time Base Vector
        dw  AppVectorStart+3    ; ADC Conversion Complete
        dw  AppVectorStart+6    ; Keyboard Vector
        dw  AppVectorStart+9    ; SCI Transmit Vector
        dw  AppVectorStart+12   ; SCI Receive Vector
        dw  AppVectorStart+15   ; SCI Error Vector
        dw  AppVectorStart+18   ; SPI Transmit Vector
        dw  AppVectorStart+21   ; SPI Receive Vector
        dw  AppVectorStart+24   ; TIM2 Overflow Vector
        dw  AppVectorStart+27   ; TIM2 Channel 1 Vector
        dw  AppVectorStart+30   ; TIM2 Channel 0 Vector
        dw  AppVectorStart+33   ; TIM1 Overflow Vector
        dw  AppVectorStart+36   ; TIM1 Channel 1 Vector
        dw  AppVectorStart+39   ; TIM1 Channel 0 Vector
        dw  AppVectorStart+42   ; PLL Vector
        dw  AppVectorStart+45   ; IRQ1 Vector
        dw  AppVectorStart+48   ; SWI Vector
        dw  Boot08Main          ; Reset Vector
