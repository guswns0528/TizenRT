#define RCC_AHB1ENR (*(volatile unsigned int*)0x40023830)
#define GPIOE_MODE (*(volatile unsigned int*)0x40021000)
#define GPIOE_PUPDR (*(volatile unsigned int*)0x4002100C)
#define GPIOE_BSRR (*(volatile unsigned int*)0x40021018)

#define GPIOC_MODE (*(volatile unsigned int*)0x40020800)
#define GPIOC_PUPDR (*(volatile unsigned int*)0x4002080C)
#define GPIOC_BSRR (*(volatile unsigned int*)0x40020818)

static void assign_gpio_clock()
{
    RCC_AHB1ENR |= 0x14;
}
static void init_led()
{
    GPIOE_PUPDR |= 0x80200000;
    GPIOE_MODE |= 0x40100000;
    GPIOE_BSRR = 0x00008000;

    GPIOC_PUPDR |= 0x80;
    GPIOC_MODE |= 0x40;
}

#define GREEN 0
#define YELLOW 1
#define RED 2

int led = GREEN;
void up_init_lowled()
{
    assign_gpio_clock();
    init_led();
}

void toggle_led()
{
    if (led == GREEN)
    {
        GPIOE_BSRR = 0x80000400;
        led = YELLOW;
    }
    else if (led == YELLOW)
    {
        GPIOE_BSRR = 0x04000000;
        GPIOC_BSRR = 0x8;
        led = RED;
    }
    else
    {
        GPIOC_BSRR = 0x80000;
        GPIOE_BSRR = 0x8000;
        led = GREEN;
    }
}


