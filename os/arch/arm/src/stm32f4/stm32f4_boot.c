// PE15 == LED GREEN, pin number 46, port e base = 0x40021000
// RCC base 0x4002 3800 0x30, 0x4

#define RCC_AHB1ENR (*(volatile unsigned int*)0x40023830)
#define GPIOE_MODE (*(volatile unsigned int*)0x40021000)
#define GPIOE_PUPDR (*(volatile unsigned int*)0x4002100C)
#define GPIOE_BSRR (*(volatile unsigned int*)0x40021018)

#define GPIOC_MODE (*(volatile unsigned int*)0x40020800)
#define GPIOC_PUPDR (*(volatile unsigned int*)0x4002080C)
#define GPIOC_BSRR (*(volatile unsigned int*)0x40020818)

void arm_boot()
{
    RCC_AHB1ENR |= 0x14;

    GPIOE_PUPDR |= 0x80200000;
    GPIOE_MODE |= 0x40100000;
    GPIOE_BSRR = 0x00008400;

    GPIOC_PUPDR |= 0x80;
    GPIOC_MODE |= 0x40;
    GPIOC_BSRR = 0x8;

    while (1);
}
