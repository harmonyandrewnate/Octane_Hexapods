#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "stepper.pio.h"

#define PIN_BASE 2
#define STEPS_PER_REV 200

void stepper_program_init(PIO pio, uint sm, uint offset, uint pin_base, float clkdiv) {
    pio_sm_config c = stepper_program_get_default_config(offset);

    sm_config_set_set_pins(&c, pin_base, 4);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 4, true);

    pio_sm_init(pio, sm, offset, &c);

    // set dir to froward by default
    pio_sm_put(pio, sm, 1);

    //pio_sm_set_enabled(pio, sm, true);
}

float rpm_to_clkdiv(float rpm) {
    float sys_clk = (float)clock_get_hz(clk_sys);

    float steps_per_sec = (rpm * STEPS_PER_REV) / 60.0;
    float instr_per_step = 32.0; // approx instructions per loop

    return sys_clk / (steps_per_sec * instr_per_step);
}

