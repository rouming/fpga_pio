;;;
;;; Don't use ./compile for this particular test,
;;; the `adafruit_pioasm` assembles the `WAIT 0 IRQ 1`
;;; to the 0x2021 machine code, which is incorrect
;;; and should be 0x2041. Use `pioasm` from the
;;; original `pico-sdk` repo.
;;;
;;; Corresponding testbench is `sim/irq_1.v`.
;;;
.program irq_1
    set pins 1                  ; Drive high
    irq wait 0                  ; Set IRQ0 and wait to be cleared
    set pins 0                  ; Drive Low
    irq nowait 1                ; Set IRQ1, nowait
    set pins 1                  ; Drive high
    wait 0 irq 1                ; Wait IRQ1 to be cleared
    set pins 0                  ; Drive low
    wait 1 irq 2                ; Wait IRQ2 to be set
