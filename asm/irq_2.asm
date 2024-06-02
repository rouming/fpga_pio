;;;
;;; Don't use ./compile for this particular test,
;;; the `adafruit_pioasm` assembles the `WAIT 0 IRQ 1`
;;; to the 0x2021 machine code, which is incorrect
;;; and should be 0x2041. Use `pioasm` from the
;;; original `pico-sdk` repo.
;;;
;;; Corresponding testbench is `sim/irq_2.v`.
;;;
.program irq_2
;;;  For SM1
    set pins 1                  ; Drive pin 0 high
    irq wait 0                  ; Set control IRQ0 and wait to be cleared
    irq clear 1                 ; Clear IRQ1 set by SM2
    irq wait 0                  ; Set control IRQ0 and wait to be cleared

;;; For SM2
    set pins 1                  ; Drive pin 1 high
    irq wait 1                  ; Set IRQ1, wait for SM1 to clear
    set pins 0                  ; Drive pin 1 low
    irq wait 1                  ; Set IRQ1, wait for SM1 to clear
