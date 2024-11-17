   
    org $FE00
PARALLEL_BASE           EQU $f600

ORB_22                  EQU PARALLEL_BASE
ORA_22                  EQU PARALLEL_BASE+$01
DDRA_22                 EQU PARALLEL_BASE+$02
DDRB_22                 EQU PARALLEL_BASE+$03
ACR_22                  EQU PARALLEL_BASE+$0b
PCR_22                  EQU PARALLEL_BASE+$0c
IFR_22                  EQU PARALLEL_BASE+$0d
IER_22                  EQU PARALLEL_BASE+$0e

PARALLEL_INIT:
        clra
        sta DDRA_22
        sta DDRB_22
        lda #$7f
        sta IER_22
        rts
