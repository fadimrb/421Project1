void main () {
    int a = 2;
    int b = 3;
    int c = a * b;
}

===

RISC-V Assembly

.globl  main
.data

.text
main:
    addi    a1, a0, 4
    addi    a2, a0, 7
    mul     a3, a1, a2
    sw      a3, 0(a0)
    ret

=====
RV32I Machine Code


X"93", X"05", X"45", X"00", 
X"13", X"06", X"75", X"00", 
X"b3", X"86", X"c5", X"02", 
X"23", X"20", X"d5", X"00", 
X"67", X"80", X"00", X"00",