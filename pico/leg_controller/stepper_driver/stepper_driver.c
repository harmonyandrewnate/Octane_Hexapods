#include <stdlib.h>
#include <pico/stdlib.h>
//#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
//#include <hardware/irq.h>
//#include <hardware/sync.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <stepper.c>


// Minimum period in us of step clock to prevent consumption compute time
#define MIN_PERIOD 200000

// Division of sys clock (200MHz) for pwm frequency.
// 8 bit int, 4 bit frac(/16)
// H-bridge good for 25kHz
// PWM freq = sysclk / (divider * (wrap+1))
#define PWM_DIV_INT_STEPPER 52
#define PWM_DIV_FRAC_STEPPER 10

// Wrap number
#define PWM_WRAP_STEPPER 99



typedef struct{
    PIO pio;
    uint sm;

    uint sliceStepPower;
    uint channelStepPower;

    // duty cycle and duty cycle limit to not overdrive
    uint powerLimit;
    uint power;

    int speed;
    uint currentStep;
} stepper;


// power is in %
void set_stepper_power(stepper *driver, int power){
    driver->power = power;
    if (power > driver->powerLimit) {
        power = driver->powerLimit;
    }
    pwm_set_chan_level(driver->sliceStepPower, driver->channelStepPower, power);
}

// Power limit is in %
// pins for en must be consecutive in the order pinAPlus, pinAMinus, pinBPlus, pinBMinus
stepper *init_stepper(uint pinAPlus, uint pinPWM, uint powerLimit, PIO pio, uint sm){

    // Setup GPIO
    gpio_set_function(pinPWM, GPIO_FUNC_PWM);
    
    pio_gpio_init(pio, pinAPlus+0);
    pio_gpio_init(pio, pinAPlus+1);
    pio_gpio_init(pio, pinAPlus+2);
    pio_gpio_init(pio, pinAPlus+3);

    pio_sm_set_consecutive_pindirs(pio, sm, pinAPlus, 4, true);
    pio_sm_set_set_pins(pio, sm, pinAPlus, 4);

    // Allocate struct
    stepper *stepperDriver = malloc(sizeof(stepper));

    if (stepperDriver == NULL) return NULL;

    // Get PWM slice and channel
    stepperDriver->sliceStepPower = pwm_gpio_to_slice_num(pinPWM);
    stepperDriver->channelStepPower = pwm_gpio_to_channel(pinPWM);

    // Configure power limits
    stepperDriver->powerLimit = powerLimit;    
    stepperDriver->power = 0;

    // Configure PWM
    pwm_set_chan_level(stepperDriver->sliceStepPower, stepperDriver->channelStepPower, 0);

    pwm_config pwmCfg = pwm_get_default_config();

    pwm_config_set_clkdiv_int_frac4 (&pwmCfg, PWM_DIV_INT_STEPPER, PWM_DIV_FRAC_STEPPER);

    pwm_config_set_wrap(&pwmCfg, PWM_WRAP_STEPPER);
    
    pwm_init (stepperDriver->sliceStepPower, &pwmCfg, true);

    
    stepperDriver->pio = pio;
    stepperDriver->sm = sm;

    stepper_program_init(pio, sm, 0, pinAPlus, 30000);

    set_stepper_power(stepperDriver, powerLimit);    

    return stepperDriver;
}

void set_stepper_speed(stepper *driver, int speed){
    int dir = 0;
    if (speed > 0) {
        dir = 1;
        speed = -speed;
    }
    
    if (speed == 0) {
        pio_sm_set_enabled(driver->pio, driver->sm, false); 
    } else {
        pio_sm_set_enabled(driver->pio, driver->sm, true);

        pio_sm_set_clkdiv(driver->pio, driver->sm, rpm_to_clkdiv(speed));
        pio_sm_put(driver->pio, driver->sm, dir);
    }
}
