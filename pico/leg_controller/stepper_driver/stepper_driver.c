#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <hardware/irq.h>
#include <hardware/sync.h>


// Minimum period in us of step clock to prevent consumption compute time
#define MIN_PERIOD 200000

// Division of sys clock (133MHz (maybe 200MHz overclocked)) for pwm frequency.
// 8 bit int, 4 bit frac(/16)
// H-bridge good for 25kHz
// PWM freq = sysclk / (divider * (wrap+1))
#define PWM_DIV_INT_STEPPER 52
#define PWM_DIV_FRAC_STEPPER 10

// Wrap number
#define PWM_WRAP_STEPPER 99



typedef struct{
    alarm_pool_t *alarmPool;
    struct repeating_timer *repeatingTimer;

    // Is it worth hardcoding vvv to reduce interupt time?
    // PWM slice and channel of pins
    uint sliceAPlus;
    uint channelAPlus;
    uint sliceAMinus;
    uint channelAMinus;

    uint sliceBPlus;
    uint channelBPlus;
    uint sliceBMinus;
    uint channelBMinus;

    // duty cycle and duty cycle limit to not overdrive
    uint powerLimit;
    uint power;

    int speed;
    uint currentStep;
} stepper;


// Power limit is in %
stepper *init_stepper(uint pinAPlus, uint pinAMinus, uint pinBPlus, uint pinBMinus,
                      uint powerLimit, uint timerAlarmNum){
    // Setup GPIO
    gpio_set_function(pinAPlus, GPIO_FUNC_PWM);
    gpio_set_function(pinAMinus, GPIO_FUNC_PWM);
    gpio_set_function(pinBPlus, GPIO_FUNC_PWM);
    gpio_set_function(pinBMinus, GPIO_FUNC_PWM);
    
    // Allocate struct
    stepper *stepperDriver = malloc(sizeof(stepper));

    if (stepperDriver == NULL) return NULL;

    // Get PWM slice and channel
    stepperDriver->sliceAPlus = pwm_gpio_to_slice_num(pinAPlus);
    stepperDriver->channelAPlus = pwm_gpio_to_channel(pinAPlus);
    stepperDriver->sliceAMinus = pwm_gpio_to_slice_num(pinAMinus);
    stepperDriver->channelAMinus = pwm_gpio_to_channel(pinAMinus);
    
    stepperDriver->sliceBPlus = pwm_gpio_to_slice_num(pinBPlus);
    stepperDriver->channelBPlus = pwm_gpio_to_channel(pinBPlus);
    stepperDriver->sliceBMinus = pwm_gpio_to_slice_num(pinBMinus);
    stepperDriver->channelBMinus = pwm_gpio_to_channel(pinBMinus);

    // Configure power limits
    stepperDriver->powerLimit = powerLimit;    
    stepperDriver->power = 0;

    // Configure PWM
    pwm_set_chan_level(stepperDriver->sliceAPlus, stepperDriver->channelAPlus, 0);
    pwm_set_chan_level(stepperDriver->sliceAMinus, stepperDriver->channelAMinus, 0);
    pwm_set_chan_level(stepperDriver->sliceBPlus, stepperDriver->channelBPlus, 0);
    pwm_set_chan_level(stepperDriver->sliceBMinus, stepperDriver->channelBMinus, 0);

    pwm_config pwmCfg = pwm_get_default_config();

    pwm_config_set_clkdiv_int_frac4 (&pwmCfg, PWM_DIV_INT_STEPPER, PWM_DIV_FRAC_STEPPER);

    pwm_config_set_wrap(&pwmCfg, PWM_WRAP_STEPPER);
    
    pwm_init (stepperDriver->sliceAPlus, &pwmCfg, true);
    pwm_init (stepperDriver->sliceAMinus, &pwmCfg, true);
    pwm_init (stepperDriver->sliceBPlus, &pwmCfg, true);
    pwm_init (stepperDriver->sliceBMinus, &pwmCfg, true);

    // Configure repeating timer
    stepperDriver->alarmPool = alarm_pool_create(timerAlarmNum, 1);

    if (stepperDriver->alarmPool == NULL) return NULL;
    
    // create repeating_timer *
    struct repeating_timer timer;
    stepperDriver->repeatingTimer = &timer;

    return stepperDriver;
}

int step_stepper(stepper *driver, int dir){
    switch (driver->currentStep) {
        case 0:
            pwm_set_chan_level(driver->sliceAPlus, driver->channelAPlus, driver->power);
            pwm_set_chan_level(driver->sliceAMinus, driver->channelAMinus, 0);
            pwm_set_chan_level(driver->sliceBPlus, driver->channelBPlus, 0);
            pwm_set_chan_level(driver->sliceBMinus, driver->channelBMinus, 0);
            break;
        case 1:
            pwm_set_chan_level(driver->sliceAPlus, driver->channelAPlus, 0);
            pwm_set_chan_level(driver->sliceAMinus, driver->channelAMinus, 0);
            pwm_set_chan_level(driver->sliceBPlus, driver->channelBPlus, driver->power);
            pwm_set_chan_level(driver->sliceBMinus, driver->channelBMinus, 0);
            break;
        case 2:
            pwm_set_chan_level(driver->sliceAPlus, driver->channelAPlus, 0);
            pwm_set_chan_level(driver->sliceAMinus, driver->channelAMinus, driver->power);
            pwm_set_chan_level(driver->sliceBPlus, driver->channelBPlus, 0);
            pwm_set_chan_level(driver->sliceBMinus, driver->channelBMinus, 0);
            break;
        case 3:
            pwm_set_chan_level(driver->sliceAPlus, driver->channelAPlus, 0);
            pwm_set_chan_level(driver->sliceAMinus, driver->channelAMinus, 0);
            pwm_set_chan_level(driver->sliceBPlus, driver->channelBPlus, 0);
            pwm_set_chan_level(driver->sliceBMinus, driver->channelBMinus, driver->power);
            break;
        default:
    }
    driver->currentStep += dir;
    driver->currentStep %= 4;
}

// power is in %
void set_stepper_power(stepper *driver, int power){
    driver->power = power;
    step_stepper(driver, 0);
}

bool step_interrupt(__unused struct repeating_timer *t){
    int speed = ((stepper *)(t->user_data))->speed;
    if (speed > 0) {
        step_stepper(t->user_data, 1);
    } else if (speed < 0) {
        step_stepper(t->user_data, -1);
    } else {
        return false;
    }
    return true;
}

void set_stepper_speed(stepper *driver, int speed){
    driver->speed = speed;
    speed = abs(speed);
    
    if (speed == 0) {
        cancel_repeating_timer(driver->repeatingTimer);
        return;
    }
    return;
    int T = 1000000 / speed;
    
    uint32_t status = save_and_disable_interrupts();
    
    cancel_repeating_timer(driver->repeatingTimer);
    
    if (T > MIN_PERIOD) {
        alarm_pool_add_repeating_timer_us(driver->alarmPool, 
                                          T, &step_interrupt, 
                                          driver, driver->repeatingTimer);
    } else {
        alarm_pool_add_repeating_timer_us(driver->alarmPool, 
                                          MIN_PERIOD, &step_interrupt, 
                                          driver, driver->repeatingTimer);
    }
    
    restore_interrupts(status);
}

void set_stepper_speed(stepper *driver, int speed){
    driver->speed = speed;
    speed = abs(speed);
    
    int T = 1000000 / speed;
    
    if (speed == 0) {
        cancel_repeating_timer(driver->repeatingTimer);
        return;
    }
    
    uint32_t status = save_and_disable_interrupts();
    
    cancel_repeating_timer(driver->repeatingTimer);
    
    if (T > MIN_PERIOD) {
        alarm_pool_add_repeating_timer_us(driver->alarmPool, 
                                          T, &step_interrupt, 
                                          driver, driver->repeatingTimer);
    } else {
        alarm_pool_add_repeating_timer_us(driver->alarmPool, 
                                          MIN_PERIOD, &step_interrupt, 
                                          driver, driver->repeatingTimer);
    }
    
    restore_interrupts(status);
}
