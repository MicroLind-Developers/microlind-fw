; -----------------------------------------------------------------
; 65c52 Bios functions for µLind
; -----------------------------------------------------------------
; Copyright Eric Lind 2024
;
; Register Location

SERIAL_BASE         = $f7c0

; COMMON REGISTERS (Offset)
; Port A
TDR1                = $03   ; Transmit Data Register
RDR1                = $03   ; Receive Data Register
IER1                = $00   ; Interupt Enable Register
ISR1                = $00   ; Interupt Status Register
CR1                 = $01   ; Control Register (Bit 7 = low)
FR1                 = $01   ; Format Register (Bit 7 = high)
CSR1                = $01   ; Control Status Register
CDR1                = $02   ; Compare Data Register (Bit 6 = low)
ACR1                = $02   ; Auxilliary Control Register (Bit 6 = high)

;Port B
TDR2                = $07   ; Transmit Data Register
RDR2                = $07   ; Receive Data Register
IER2                = $04   ; Interupt Enable Register
ISR2                = $04   ; Interupt Status Register
CR2                 = $05   ; Control Register (Bit 7 = low)
FR2                 = $05   ; Format Register (Bit 7 = high)
CSR2                = $05   ; Control Status Register
CDR2                = $06   ; Compare Data Register (Bit 6 = low)
ACR2                = $06   ; Auxilliary Control Register (Bit 6 = high)

; COMMON REGISTERS (Direct)
; Port A
TDR1_D              = $f7c3   ; Transmit Data Register
RDR1_D              = $f7c3   ; Receive Data Register
IER1_D              = $f7c0   ; Interupt Enable Register
ISR1_D              = $f7c0   ; Interupt Status Register
CR1_D               = $f7c1   ; Control Register (Bit 7 = low)
FR1_D               = $f7c1   ; Format Register (Bit 7 = high)
CSR1_D              = $f7c1   ; Control Status Register
CDR1_D              = $f7c2   ; Compare Data Register (Bit 6 = low)
ACR1_D              = $f7c2   ; Auxilliary Control Register (Bit 6 = high)

;Port B
TDR2_D              = $f7c7   ; Transmit Data Register
RDR2_D              = $f7c7   ; Receive Data Register
IER2_D              = $f7c4   ; Interupt Enable Register
ISR2_D              = $f7c4   ; Interupt Status Register
CR2_D               = $f7c5   ; Control Register (Bit 7 = low)
FR2_D               = $f7c5   ; Format Register (Bit 7 = high)
CSR2_D              = $f7c5   ; Control Status Register
CDR2_D              = $f7c6   ; Compare Data Register (Bit 6 = low)
ACR2_D              = $f7c6   ; Auxilliary Control Register (Bit 6 = high)

; -----------------------------------------------------------------
; SERIAL INIT
; input:            None
; output:           None
; clobbers:         A
; -----------------------------------------------------------------
SERIAL_INIT_A:
        lda #$c1                ; Set interrupt on Transmit Data Empty, Receive Data Full
        sta IER1_D
        lda #$0e                ; CDR, Set 38400 Baud, 1 stop bit, no echo
        sta CR1_D
        lda #$f0                ; 8 bits, No parity, DTR & RTS low 
        sta CR1_D

SERIAL_INIT_B:
        lda #$c1                ; Set interrupt on Transmit Data Empty, Receive Data Full
        sta IER2_D
        lda #$0c                ; CDR, Set 9600 Baud, 1 stop bit, no echo
        sta CR2_D
        lda #$f0                ; 8 bits, No parity, DTR & RTS low 
        sta CR2_D

; -----------------------------------------------------------------
; SERIAL PRINT
; input:            X - Points to /0 terminated string
; output:           None
; clobbers:         A, B
; -----------------------------------------------------------------
prt0:   ldb ISR1_D
        andb #$40
        beq prt0
        sta TDR1_D
SERIAL_PRINT:  
        lda ,x+
        bne prt0
        rts

;---------------------------------------------------------------------
; Input char from UART (blocking)
; Exit: character in A
; -----------------------------------------------------------------
; SERIAL INPUT (blocking)
; input:            None
; output:           A - Character
; clobbers:         A, B
; -----------------------------------------------------------------
SERIAL_INPUT_A:
        lda ISR1_D              ; Get status            
        anda #$01               ; Check if receiver is full
        beq SERIAL_INPUT_A      ; if not...
        lda RDR1                ; Get charactir in A
        rts