#include <stdlib.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>



// Division of sys clock (133MHz (maybe 200MHz overclocked)) for pwm frequency.
// 8 bit int, 4 bit frac(/16)
#define PWM_DIV_INT_MOTOR 52
#define PWM_DIV_FRAC_MOTOR 10

// Wrap number
#define PWM_WRAP_MOTOR 99



typedef struct {
    uint slicePlus;
    uint channelPlus;
    uint sliceMinus;
    uint channelMinus;
} motor;



motor *init_motor(uint pinPlus, uint pinMinus){
    gpio_set_function(pinPlus, GPIO_FUNC_PWM);
    gpio_set_function(pinMinus, GPIO_FUNC_PWM);

    motor *motorDriver = malloc(sizeof(motor));
    
    if (motorDriver == NULL) return NULL;

    motorDriver->slicePlus = pwm_gpio_to_slice_num(pinPlus);
    motorDriver->channelPlus = pwm_gpio_to_channel(pinPlus);
    motorDriver->sliceMinus = pwm_gpio_to_slice_num(pinMinus);
    motorDriver->channelMinus = pwm_gpio_to_channel(pinMinus);

    pwm_set_chan_level(motorDriver->slicePlus, motorDriver->channelPlus, 0);
    pwm_set_chan_level(motorDriver->sliceMinus, motorDriver->channelMinus, 0);

    pwm_config pwmCfg = pwm_get_default_config();

    pwm_config_set_clkdiv_int_frac4 (&pwmCfg, PWM_DIV_INT_MOTOR, PWM_DIV_FRAC_MOTOR);

    pwm_config_set_wrap(&pwmCfg, PWM_WRAP_MOTOR);

    pwm_init (motorDriver->slicePlus, &pwmCfg, true);
    pwm_init (motorDriver->sliceMinus, &pwmCfg, true);

    return motorDriver;
}

/*
 * Set power of motor. Takes pointer to motor-driver struct and power (-100, 100) 
 */ 
void set_motor(motor *driver, int power){
    if (driver != NULL) {
        if (power > 0) {
            pwm_set_chan_level(driver->slicePlus, driver->channelPlus, power);
            pwm_set_chan_level(driver->sliceMinus, driver->channelMinus, 0);
        } else if (power < 0) {
            pwm_set_chan_level(driver->slicePlus, driver->channelPlus, 0);
            pwm_set_chan_level(driver->sliceMinus, driver->channelMinus, -power);
        } else {
            pwm_set_chan_level(driver->slicePlus, driver->channelPlus, 0);
            pwm_set_chan_level(driver->sliceMinus, driver->channelMinus, 0);
        }
    }
}
