extern void up_init_lowled();
extern void toggle_led();

void halt_loop()
{
    while (1)
    {
        int i;
        for (i = 0; i < 100000; i++)
        {
        }
        toggle_led();
    }
}

void arm_boot()
{
    up_init_lowled();

    halt_loop();
}
