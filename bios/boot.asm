    org $FE00
default_string
        FCN "Debug print from microLind"

    org $FF00
hook_trap:
hook_swi3:
hook_swi2:
hook_swi:
hook_firq:
hook_irq:
hook_nmi:
hook_restart:
    lds #$0100              ;Set system stack pointer to bottom of stack
    jsr clear_regs
    jsr SER_INIT

loop:
    ldx #default_string
    jsr PRINT_S1

    ;ldy #$0100              ;Start of memory area to test
    ;ldx #$dfff              ;Length of tested memory
    ;pshs x,y                ;Save parameters on stack
    bra loop
    ;jsr ramtest             ;Start test


clear_regs:
    ldq     #$00000000
    tfr     a,dp
    tfr     d,x
    tfr     d,y
    tfr     d,u
    tfr     d,v
    andcc   #$00
    rts

    org $FFF0

vtrap: fdb hook_trap
vswi3: fdb hook_swi3
vswi2: fdb hook_swi2
vfirq: fdb hook_firq
virq: fdb hook_irq
vswi: fdb hook_swi
vnmi: fdb hook_nmi
vrestart: fdb hook_restart