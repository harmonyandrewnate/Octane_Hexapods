#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "stepper_driver.c"
#include "motor_driver.c"
#include "encoder.c"



#define BUFF_LEN 64

#define SWING_A_PLUS 10
#define SWING_PWM 28

#define STEP_POW_LIMIT 100

#define LIFT_PLUS 8
#define LIFT_MINUS 9

#define ELBOW_PLUS 4
#define ELBOW_MINUS 5



typedef struct{
    stepper *shoulderSwing;
    motor *shoulderLift;
    motor *elbow;
} legModule;

void handle_cmd(char *buff, legModule *leg) {
    int arg1, arg2, matches;
    char cmd[BUFF_LEN];
    matches = sscanf(buff, "%s %i %i", cmd, &arg1, &arg2);
    if (strcmp(cmd, "stepper") == 0 && matches == 2){
        stepper_driver_set_vel(leg->shoulderSwing, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "power") == 0 && matches == 2){
        stepper_driver_set_power(leg->shoulderSwing, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "shoulder") == 0 && matches == 2){
        set_motor(leg->shoulderLift, arg1);
        printf("set %d \n", arg1);
    } else if (strcmp(cmd, "elbow") == 0 && matches == 2){
        set_motor(leg->elbow, arg1);
        printf("set %d \n", arg1);
    } 
}

int main() {
    stdio_init_all();

    legModule leg;

    stepper shoulderDriver;
    leg.shoulderSwing = &shoulderDriver;

    init_stepper_driver(leg.shoulderSwing, SWING_A_PLUS, SWING_PWM, STEP_POW_LIMIT, pio0, 0);
    
    stepper_driver_set_power(leg.shoulderSwing, 100);    

    leg.shoulderLift = init_motor(LIFT_PLUS, LIFT_MINUS);
    leg.elbow = init_motor(ELBOW_PLUS, ELBOW_MINUS);

    
    printf("commands are stepper, shoulder, and elbow each with 1 arg. \n");


    int read_chars = 0;
    char buff[BUFF_LEN] = {0};
    char tmp_char = 0;

    while (1) {
        tmp_char = getchar();
        if (read_chars < BUFF_LEN) {
            buff[read_chars] = tmp_char;
            read_chars++;
            printf("%c", tmp_char);
            if (tmp_char == '\r') printf("\n");
            
            if (tmp_char == '\n' || tmp_char == '\r') {
                handle_cmd(buff, &leg);
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
}
