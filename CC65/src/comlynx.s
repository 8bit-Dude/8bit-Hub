;
; Copyright (c) 2020 Anthony Beaucamp.
;
; This software is provided 'as-is', without any express or implied warranty.
; In no event will the authors be held liable for any damages arising from
; the use of this software.
;
; Permission is granted to anyone to use this software for any purpose,
; including commercial applications, and to alter it and redistribute it
; freely, subject to the following restrictions:
;
;   1. The origin of this software must not be misrepresented; you must not
;   claim that you wrote the original software. If you use this software in a
;   product, an acknowledgment in the product documentation would be
;   appreciated but is not required.
;
;   2. Altered source versions must be plainly marked as such, and must not
;   be misrepresented as being the original software.
;
;   3. This notice may not be removed or altered from any distribution.
;
;   4. The names of this software and/or it's copyright holders may not be
;   used to endorse or promote products derived from this software without
;   specific prior written permission.
;
; Adapted from serial driver by Karri Kaksonen
;

;	Atari Lynx Comlynx Interface to 8bit-Hub

	.include "../inc/lynx.inc"
	.include "../inc/ser-kernel.inc"
	.include "../inc/ser-error.inc"

	.importzp ptr1
		
	.export _SerialOpen
	.export _SerialGet
	.export _SerialPut
	
	.interruptor SERIRQ, 29     ; Export as high priority IRQ handler
	
	.data	

SERIRQ:     .byte   $60, $00, $00       ; RTS plus two dummy bytes	
	
	.bss

contrl:         .res    1
SerialStat:     .res    1
RxPtrIn:        .res    1
RxPtrOut:       .res    1
TxPtrIn:        .res    1
TxPtrOut:       .res    1
TxBuffer:       .res    256
RxBuffer:       .res    256
TxDone:         .res    1

	.code	
	
; ---------------------------------------------------------------
; void __near__ _SerialOpen (void* ptr)
; ---------------------------------------------------------------
;
; The Lynx has only two correct serial data formats:
; 8 bits, parity mark, 1 stop bit
; 8 bits, parity space, 1 stop bit
;
; It also has two wrong formats;
; 8 bits, even parity, 1 stop bit
; 8 bits, odd parity, 1 stop bit
;
; Unfortunately the parity bit includes itself in the calculation making
; parity not compatible with the rest of the world.
;
; We can only specify a few baud rates.
; Lynx has two non-standard speeds 31250 and 62500 which are
; frequently used in games.
;
; The receiver will always read the parity and report parity errors.
;
; Must return an SER_ERR_xx code in a/x.

.proc _SerialOpen: near
		; Set pointer value
		;sta 	ptr1
		;stx	ptr1+1
		
        stz     RxPtrIn
        stz     RxPtrOut
        stz     TxPtrIn
        stz     TxPtrOut

        ; clock = 8 * 15625
        lda     #%00011000
        sta     TIM4CTLA

        ldx     #1
setbaudrate:
        stx     TIM4BKUP
baudsuccess:
        ldx     #TxOpenColl|ParEven
        stx     contrl
        ldx     #TxOpenColl
        stx     contrl
checkhs:
        ldx     contrl
        stx     SERCTL
        lda     SERDAT
        lda     contrl
        ora     #RxIntEnable|ResetErr
        sta     SERCTL
; Install interrupt
		lda 	#<IRQ
		sta 	SERIRQ+1
		lda 	#>IRQ
		sta 	SERIRQ+2
		lda     #$4C        ; Jump opcode
		sta     SERIRQ      ; Activate IRQ routine
        rts
.endproc

; ---------------------------------------------------------------
; unsigned char __near__ _SerialGet (unsigned char* value)
; ---------------------------------------------------------------
;
; Will fetch a character from the receive buffer and store it into the
; variable pointed to by ptr1. If no data is available, SER_ERR_NO_DATA is
; returned.

.proc _SerialGet: near
		; Set pointer value
		sta 	ptr1
		stx		ptr1+1

        lda     RxPtrIn
        cmp     RxPtrOut
        bne     GetByte
        lda     #<SER_ERR_NO_DATA
        ldx     #>SER_ERR_NO_DATA
        rts
GetByte:
        ldy     RxPtrOut
        lda     RxBuffer,y
        inc     RxPtrOut
        ldx     #$00
        sta     (ptr1,x)
        txa                     ; Return code = 0
        rts
.endproc

; ---------------------------------------------------------------
; unsigned char __near__ _SerialPut (unsigned char value)
; ---------------------------------------------------------------
;
; Output character in A.
; Must return an SER_ERR_xx code in a/x.

.proc _SerialPut: near
        tax
        lda     TxPtrIn
        ina
        cmp     TxPtrOut
        bne     PutByte
        lda     #<SER_ERR_OVERFLOW
        ldx     #>SER_ERR_OVERFLOW
        rts
PutByte:
        ldy     TxPtrIn
        txa
        sta     TxBuffer,y
        inc     TxPtrIn

        bit     TxDone
        bmi     @L1
        php
        sei
        lda     contrl
        ora     #TxIntEnable|ResetErr
        sta     SERCTL       ; Allow TX-IRQ to hang RX-IRQ
        sta     TxDone
        plp
@L1:
        lda     #<SER_ERR_OK
        tax
        rts		
.endproc

		
;----------------------------------------------------------------------------
; IRQ: Called from the builtin runtime IRQ handler as a subroutine. All
; registers are already saved, no parameters are passed, but the carry flag
; is clear on entry. The routine must return with carry set if the interrupt
; was handled, otherwise with carry clear.
;
; Both the Tx and Rx interrupts are level sensitive instead of edge sensitive.
; Due to this bug you have to disable the interrupt before clearing it.

IRQ:
        lda     INTSET          ; Poll all pending interrupts
        and     #SERIAL_INTERRUPT
        bne     @L0
        clc
        rts
@L0:
        bit     TxDone
        bmi     @tx_irq     ; Transmit in progress
        ldx     SERDAT
        lda     SERCTL
        and     #RxParityErr|RxOverrun|RxFrameErr|RxBreak
        beq     @rx_irq
        tsb     SerialStat  ; Save error condition
        bit     #RxBreak
        beq     @noBreak
        stz     TxPtrIn     ; Break received - drop buffers
        stz     TxPtrOut
        stz     RxPtrIn
        stz     RxPtrOut
@noBreak:
        lda     contrl
        ora     #RxIntEnable|ResetErr
        sta     SERCTL
        lda     #$10
        sta     INTRST
        bra     @IRQexit
@rx_irq:
        lda     contrl
        ora     #RxIntEnable|ResetErr
        sta     SERCTL
        txa
        ldx     RxPtrIn
        sta     RxBuffer,x
        txa
        inx

@cont0:
        cpx     RxPtrOut
        beq     @1
        stx     RxPtrIn
        lda     #SERIAL_INTERRUPT
        sta     INTRST
        bra     @IRQexit

@1:
        sta     RxPtrIn
        lda     #$80
        tsb     SerialStat
@tx_irq:
        ldx     TxPtrOut    ; Has all bytes been sent?
        cpx     TxPtrIn
        beq     @allSent

        lda     TxBuffer,x  ; Send next byte
        sta     SERDAT
        inc     TxPtrOut

@exit1:
        lda     contrl
        ora     #TxIntEnable|ResetErr
        sta     SERCTL
        lda     #SERIAL_INTERRUPT
        sta     INTRST
        bra     @IRQexit

@allSent:
        lda     SERCTL       ; All bytes sent
        bit     #TxEmpty
        beq     @exit1
        bvs     @exit1
        stz     TxDone
        lda     contrl
        ora     #RxIntEnable|ResetErr
        sta     SERCTL

        lda     #SERIAL_INTERRUPT
        sta     INTRST
@IRQexit:
        clc
        rts	
		