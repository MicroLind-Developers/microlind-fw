   
    org $FE10
IRQ_BASE           EQU $f600

IRQ_INIT:
        clra
        sta IRQ_BASE
        rts

SET_IRQ_FILTER:
        anda #$0f
        sta IRQ_BASE
        rts

GET_ACTIVE_IRQ:
        lda IRQ_BASE
        anda #$0f
        rts

GET_CURRENT_IRQ_FILTER:
        lda IRQ_BASE
        lsra
        lsra
        lsra
        lsra 
        rts