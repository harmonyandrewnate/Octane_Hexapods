#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <hardware/i2c.h>



/*
 * I2C configuration
 */
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define I2C_BAUDRATE 10000
#define I2C_SLAVE_BASE_ADDRESS 0x10

// Serial buffer
#define BUFF_LEN 64

// Serial parse mem
int read_chars = 0;
char buff[BUFF_LEN] = {0};
char tmp_char = 0;




void I2C_setup() {
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    i2c_init(i2c0, I2C_BAUDRATE);
}


void handle_cmd(char *buff) {
    int arg1, arg2, matches;
    char cmd[BUFF_LEN];
    uint8_t msg;
    matches = sscanf(buff, "%s", cmd);
    if (buff[0] == 'f'){
        msg = 'f';
        i2c_write_blocking(i2c0, 0x10, &msg, 1, false);
        //printf("f sent\n");
    } else if (strcmp(cmd, "b") == 0 && matches == 1){
        msg = 'b';
        i2c_write_blocking(i2c0, 0x10, &msg, 1, false);
        //printf("b sent\n");
    } else if (strcmp(cmd, "l") == 0 && matches == 1){
        msg = 'l';
        i2c_write_blocking(i2c0, 0x10, &msg, 1, false);
        //printf("l sent\n");
    } else if (strcmp(cmd, "r") == 0 && matches == 1){
        msg = 'r';
        i2c_write_blocking(i2c0, 0x10, &msg, 1, false);
        //printf("r sent\n");
    } else if (strcmp(cmd, "s") == 0 && matches == 1){
        msg = 's';
        i2c_write_blocking(i2c0, 0x10, &msg, 1, false);
        //printf("s sent\n");
    }
    //printf("parsed %s\n", buff); 
}

void serial_handler() {
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



int main() {
    stdio_init_all();

    I2C_setup();    
    
    while (true) {
        serial_handler();
    }
}
