#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include <hardware/i2c.h>
#include <pico/i2c_slave.h>

#include "stepper_driver.c"
#include "motor_driver.c"
#include "encoder.c"
#include "PID.c"

#include "forward.c"
#include "backward.c"
#include "left.c"
#include "right.c"


// DEBUG ENABLE
#define DEBUG_EN true

// Main I2C input enable
#define I2C_EN true

// Serial buffer
#define BUFF_LEN 64

// Stepper pins
#define SWING_A_PLUS 10
#define SWING_PWM 28

// Stepper cfg
#define STEP_POW_LIMIT 20

// Stepper PIO
#define STEPPER_PIO pio1

// Lift motor pins
#define LIFT_PLUS 8
#define LIFT_MINUS 9

// Elbow motor pins
#define ELBOW_PLUS 4
#define ELBOW_MINUS 5

// Which PIO is used for encoders
#define ENCODER_PIO pio0
// Max encoder rate for power saving. Passing zero sets max
#define MAX_ENC_RATE 0

// Encoder start pins
#define SWING_ENC_PIN 14
#define LIFT_ENC_PIN 6
#define ELBOW_ENC_PIN 2

// Encoder scales
// gets result in mm
#define MOTOR_SCALE  0.01436781609
// gets result in rad
#define STEPPER_SCALE 0.00007853981

/*
 * Leg ID pins
 */
#define LEG_ID_0 20
#define LEG_ID_1 21
#define LEG_ID_2 22

/*
 * I2C configuration
 */
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define I2C_BAUDRATE 100000
#define I2C_SLAVE_BASE_ADDRESS 0x10


/* ----------- 
 *  PID VALUES
 * ------------
 */
// PID period us
#define PID_PERIOD 1000

// Shoulder Swing
#define SWING_P -10000000.0
#define SWING_I 0.0
#define SWING_D 0.0

// Shoulder Lift
#define LIFT_P -50.0
#define LIFT_I 0.0
#define LIFT_D 0.0

// Elbow
#define ELBOW_P -50.0
#define ELBOW_I 0.0
#define ELBOW_D 0.0

// Trajectory update period in ms
#define TRAJ_PERIOD 200



typedef struct{
    stepper *shoulderSwing;
    encoder_t *swingEnc;
    PID_cfg *swingPID;

    motor *shoulderLift;
    encoder_t *liftEnc;
    PID_cfg *liftPID;

    motor *elbow;
    encoder_t *elbowEnc;
    PID_cfg *elbowPID;

    int legNumber;
} legModule;




// Define global memory
legModule leg;

stepper shoulderDriver;
motor shoulderLiftDriver;
motor elbowDriver;

encoder_t swingEncoder;
encoder_t liftEncoder;
encoder_t elbowEncoder;

PID_cfg swingPIDcfg;
PID_cfg liftPIDcfg;
PID_cfg elbowPIDcfg;

// trajectorie values
bool traj_running;
int curr_traj_step;
float (*curr_traj)[3];
char next_command;

// Timer structs
struct repeating_timer PID_timer;
struct repeating_timer traj_timer;

// Serial parse mem
int read_chars = 0;
char buff[BUFF_LEN] = {0};
char tmp_char = 0;



void mem_setup(){
    // Setup memory space
    leg.shoulderSwing = &shoulderDriver;

    leg.swingEnc = &swingEncoder;
    leg.swingPID = &swingPIDcfg;

    leg.shoulderLift = &shoulderLiftDriver;
    leg.liftEnc = &liftEncoder;
    leg.liftPID = &liftPIDcfg;

    leg.elbow = &elbowDriver;
    leg.elbowEnc = &elbowEncoder;
    leg.elbowPID = &elbowPIDcfg;
}

void id_leg() {
    // ideally update to use id pins
    leg.legNumber = 0;
}

// PID must be disabled when this is running
void home_axis() {
    int last = get_encoder(leg.liftEnc);
    int current = last;

    set_motor(leg.shoulderLift, -50);
    do {
        last = current;
        sleep_ms(500);
        current = get_encoder(leg.liftEnc);
        printf("current: %d, last: %d \n", current, last);
    } while (last != current);

    set_motor(leg.shoulderLift, 0);
    reset_encoder(leg.liftEnc);
    

    set_motor(leg.elbow, -50);
    do {
        last = current;
        sleep_ms(500);
        current = get_encoder(leg.elbowEnc);
        printf("current: %d, last: %d \n", current, last);
    } while (last != current);

    set_motor(leg.elbow, 0);
    reset_encoder(leg.elbowEnc);
}


// PID handler
bool PID_timer_callback(__unused struct repeating_timer *t) {
    set_motor(leg.elbow, (int)update_PID(leg.elbowPID, get_encoder(leg.elbowEnc)));
    set_motor(leg.shoulderLift, (int)update_PID(leg.liftPID, get_encoder(leg.liftEnc)));
    stepper_driver_set_vel(leg.shoulderSwing, (int)update_PID(leg.swingPID, get_encoder(leg.swingEnc)));

    // timer repeats
    return true;
}


// Trajectory target updater
bool traj_timer_callback(__unused struct repeating_timer *t) {
    
    set_PID_target(leg.swingPID, curr_traj[curr_traj_step][0]);
    set_PID_target(leg.liftPID, curr_traj[curr_traj_step][1]);
    set_PID_target(leg.elbowPID, curr_traj[curr_traj_step][2]);
    
    printf("targets: (%f, %f, %f) \n", curr_traj[curr_traj_step][0], curr_traj[curr_traj_step][1], curr_traj[curr_traj_step][2]);

    curr_traj_step ++;

    if (curr_traj_step > 119) { 
        printf("traj done \n");       
        traj_running = false;
        return false;
    } else {
        // timer repeats
        return true;
    }
}





void handle_cmd(char *buff) {
    int arg1, arg2, matches;
    char cmd[BUFF_LEN];
    matches = sscanf(buff, "%s %i %i", cmd, &arg1, &arg2);
    if (strcmp(cmd, "home") == 0 && matches == 1){
        home_axis();
        printf("homing \n");
    } else if (strcmp(cmd, "traj_enable") == 0 && matches == 1) {
        curr_traj_step = 0;
        traj_running = true;
        curr_traj = forward_trajectory[leg.legNumber];
        add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);
        printf("traj on \n");
    } else if (strcmp(cmd, "PID_enable") == 0 && matches == 1){
        add_repeating_timer_us(-PID_PERIOD, PID_timer_callback, NULL, &PID_timer);
        printf("PID on \n");
    } else if (strcmp(cmd, "PID_disable") == 0 && matches == 1){
        cancel_repeating_timer(&PID_timer);
        printf("PID off \n");
    } else if (strcmp(cmd, "stepper") == 0 && matches == 2){
        stepper_driver_set_vel(leg.shoulderSwing, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "power") == 0 && matches == 2){
        stepper_driver_set_power(leg.shoulderSwing, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "shoulder") == 0 && matches == 2){
        set_motor(leg.shoulderLift, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "elbow") == 0 && matches == 2){
        set_motor(leg.elbow, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "PID_elbow") == 0 && matches == 2){
        set_PID_target(leg.elbowPID, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "PID_lift") == 0 && matches == 2){
        set_PID_target(leg.liftPID, arg1);
        printf("set %d \n", arg1);
    } 
}

void debug_handler() {
    tmp_char = getchar();
    if (read_chars < BUFF_LEN) {
        buff[read_chars] = tmp_char;
        read_chars++;
        printf("%c", tmp_char);
        if (tmp_char == '\r') printf("\n");
        
        if (tmp_char == '\n' || tmp_char == '\r') {
            handle_cmd(buff);
            for (int i = 0; i < read_chars; i++) {
                buff[i] = 0;
            }
            read_chars = 0;
        }
    } else {
        if (tmp_char == '\n' || tmp_char == '\r') {
            printf("%c", tmp_char);
            read_chars = 0;
            tmp_char = '\0';
        }
    }
}

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    //switch (event) {
    //case (I2C_SLAVE_RECEIVE):
    char val = i2c_read_byte_raw(i2c);
    switch (val) {
    case ('f'):
        next_command = 'f';
        printf("next command: f\n");
        break;
    case ('b'):
        next_command = 'b';
        printf("next command: b\n");
        break;
    case ('l'):
        next_command = 'l';
        printf("next command: l\n");
        break;
    case ('r'):
        next_command = 'r';
        printf("next command: r\n");
        break;
    case ('s'):
        next_command = 's';
        printf("next command: s\n");
        break;
    default:
        next_command = 's';
        printf("next command: s\n");
        break;
    }
    //    break;
    //default:
    //    break;
    //}
}

void I2C_input_handler(){
    if (!traj_running) {
        if (next_command == 's') {
            return; //cancel_repeating_timer(&traj_timer);
        } else {
            switch (next_command) {
            case ('f'):
                curr_traj_step = 0;
                traj_running = true;
                curr_traj = forward_trajectory[leg.legNumber];
                add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);
                break;
            case ('b'):
                curr_traj_step = 0;
                traj_running = true;
                curr_traj = backward_trajectory[leg.legNumber];
                add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);
                break;
            case ('l'):
                curr_traj_step = 0;
                traj_running = true;
                curr_traj = left_trajectory[leg.legNumber];
                add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);
                break;
            case ('r'):
                curr_traj_step = 0;
                traj_running = true;
                curr_traj = right_trajectory[leg.legNumber];
                add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);
                break;
            }
        }
    }
}

void I2C_setup(){
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_hw_t *controll_i2c_hw = i2c_get_hw(i2c0);
    *((int *)(controll_i2c_hw+0x30)) |= 1 << 11;

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_BASE_ADDRESS + leg.legNumber, &i2c_slave_handler);    
}



int main() {
    stdio_init_all();

    mem_setup();

    // Motor Driver setup
    init_stepper_driver(leg.shoulderSwing, SWING_A_PLUS, SWING_PWM, STEP_POW_LIMIT, STEPPER_PIO, 0);
    
    stepper_driver_set_power(leg.shoulderSwing, 100);    

    init_motor(leg.shoulderLift, LIFT_PLUS, LIFT_MINUS);
    init_motor(leg.elbow, ELBOW_PLUS, ELBOW_MINUS);

    // Encoder setup
    setup_encoder_pio(ENCODER_PIO);
    
    encoder_init(leg.swingEnc, ENCODER_PIO, 0, SWING_ENC_PIN, MAX_ENC_RATE, STEPPER_SCALE);
    encoder_init(leg.liftEnc, ENCODER_PIO, 1, LIFT_ENC_PIN, MAX_ENC_RATE, MOTOR_SCALE);
    encoder_init(leg.elbowEnc, ENCODER_PIO, 2, ELBOW_ENC_PIN, MAX_ENC_RATE, MOTOR_SCALE);

    // PID setup
    init_PID(leg.swingPID, SWING_P, SWING_I, SWING_D, PID_PERIOD);
    init_PID(leg.liftPID, LIFT_P, LIFT_I, LIFT_D, PID_PERIOD);
    init_PID(leg.elbowPID, ELBOW_P, ELBOW_I, ELBOW_D, PID_PERIOD);

    
    id_leg();
    

    I2C_setup();

    // Start PID
    add_repeating_timer_us(-PID_PERIOD, PID_timer_callback, NULL, &PID_timer);
    
    curr_traj_step = 0;
    traj_running = false;
    // Start Trajectory
    // add_repeating_timer_ms(-TRAJ_PERIOD, traj_timer_callback, NULL, &traj_timer);



    printf("commands are power, stepper, shoulder, and elbow each with 1 arg. \n");



    while (1) {
        if (DEBUG_EN) debug_handler();
        if (I2C_EN) I2C_input_handler();
    }
}
