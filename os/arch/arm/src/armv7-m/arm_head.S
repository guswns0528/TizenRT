    .text
    .section .text.startup, "ax"
    .thumb
    .code 16

    .globl arm_boot
    .globl __start
    .type __start, function

__start:
    b   arm_boot
