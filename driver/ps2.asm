   
    org $FE20
PS2_BASE           EQU $f600

PS2_KBD             EQU PS2_BASE+0
PS2_MOUSE           EQU PS2_BASE+1

PS2_INIT:
        rts

PS2_READ_KBD:
        sta PS2_KBD
        rts

PS2_READ_MOUSE:
        sta PS2_MOUSE
        rts