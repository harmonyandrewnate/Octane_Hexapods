#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "stdio.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "stepper_driver.pio.h"


// Division of sys clock (200MHz) for pwm frequency.
// 8 bit int, 4 bit frac(/16)
// H-bridge good for 25kHz
// PWM freq = sysclk / (divider * (wrap+1))
#define PWM_DIV_INT_STEPPER 104 //52
#define PWM_DIV_FRAC_STEPPER 10

// PSM wrap number
#define PWM_WRAP_STEPPER 99

// Speed calculation to clkdiv constants
#define INSTR_PER_STEP 64
#define STEPS_PER_REV 200

typedef struct{
    PIO pio;
    uint sm;

    uint sliceStepPower;
    uint channelStepPower;

    // duty cycle and duty cycle limit to not overdrive
    uint powerLimit;
} stepper;



// Setup stepper driver
void init_stepper_driver(stepper *driver, uint pinAPlus, uint pinPWM, uint powerLimit, PIO pio, uint sm){
    driver->sliceStepPower = pwm_gpio_to_slice_num(pinPWM);
    driver->channelStepPower = pwm_gpio_to_channel(pinPWM);

    driver->powerLimit = powerLimit;

    driver->pio = pio;
    driver->sm = sm;
    
    // pio init
    int offset = pio_add_program(pio, &stepper_program);
    pio_stepper_init(pio, sm, offset, pinAPlus);
    
    // pwm init
    gpio_set_function(pinPWM, GPIO_FUNC_PWM);
    
    pwm_set_chan_level(driver->sliceStepPower, driver->channelStepPower, 0);

    pwm_config pwmCfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int_frac4 (&pwmCfg, PWM_DIV_INT_STEPPER, PWM_DIV_FRAC_STEPPER);
    pwm_config_set_wrap(&pwmCfg, PWM_WRAP_STEPPER);
    
    pwm_init (driver->sliceStepPower, &pwmCfg, true);
}

//clkdiv from speed(dps)
inline float get_div_from_speed(int speed){
    float targStepFreq = (speed * 360) / STEPS_PER_REV;
    float div = (clock_get_hz(clk_sys) / (targStepFreq / INSTR_PER_STEP));
    if (div > 100000) {
        return 0;
    } else if (div <= 80000) {
        return 80000;
    } else {
        return div;
    }
}

// Set velocity
void stepper_driver_set_vel(stepper *driver, int vel){
    int dir = 0;
    int speed = 0;
    if (vel < 0) {
        dir = 1;
        speed = -vel;
    }else{
        speed = vel;
    }

    int clkdiv = get_div_from_speed(speed);
    
    if (clkdiv == 0) {
        pio_sm_set_enabled(driver->pio, driver->sm, false);
    } else {
        pio_sm_set_enabled(driver->pio, driver->sm, true);    

        pio_sm_put(driver->pio, driver->sm, dir);
        pio_sm_set_clkdiv(driver->pio, driver->sm, (float)clkdiv);
    }
}

// Set power
void stepper_driver_set_power(stepper *driver, uint power){
    if (power > driver->powerLimit) {
        power = driver->powerLimit;
    }
    pwm_set_chan_level(driver->sliceStepPower, driver->channelStepPower, power);
}
