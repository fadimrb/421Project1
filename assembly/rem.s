RISC-V assembly

.globl  main
.data

.text
main:
    addi    x1, x0, 12
    addi    x2, x0, 5
    rem     x3, x1, a2
    sw      x3, 0(x0)
    ret

=====

RV32I machine code

X"93", X"00", X"c0", X"00", X"13", X"01", X"50", X"00", X"b3", X"e1", X"c0", X"02", X"23", X"20", X"30", X"00", X"67", X"80", X"00", X"00",

X"93", X"00", X"c0", X"00",
X"13", X"01", X"50", X"00",
X"b3", X"e1", X"c0", X"02",
X"23", X"20", X"30", X"00",
X"67", X"80", X"00", X"00",