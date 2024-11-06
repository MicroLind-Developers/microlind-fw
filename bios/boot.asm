    include serial_6552.asm

; Set 
    org $FF00
hook_trap:
hook_swi3:
hook_swi2:
hook_swi:
hook_firq:
hook_irq:
hook_nmi:
hook_restart:
    ; Setup stack pointer for gods sake!!!
    lds #$0200              ;Set system stack pointer to bottom of stack
    jsr INIT
    jsr SERIAL_INIT_A
wait_for_cr:
    jsr SERIAL_INPUT_A
    cmpa #$0d
    bne wait_for_cr
    ldx #str_greet
    jsr SERIAL_PRINT
loop:
    bra loop



; System init function
INIT:
    jsr CLEAR_REGS
    orcc #$50
    lda #$c0
    tfr a,dp
    rts

CLEAR_REGS:
    ldq     #$00000000
    tfr     a,dp
    tfr     d,x
    tfr     d,y
    tfr     d,u
    tfr     d,v
    andcc   #$00
    rts

    org $F000
str_greet:
    fcn 'microLind initialized...'
str_start_memcheck:
    fcn 'Starting memcheck...'

; Setup vectors
    org $FFF0

V_TRAP:     fdb hook_trap
V_SWI3:     fdb hook_swi3
V_SWI2:     fdb hook_swi2
V_FIRQ:     fdb hook_firq
V_IRQ:      fdb hook_irq
V_SWI:      fdb hook_swi
V_NMI:      fdb hook_nmi
V_RESET:    fdb hook_restart

