#include "quadrature_encoder.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

int offsets[8] = {0};

typedef struct {
    PIO pio;
    uint sm;
    float scale;
} encoder_t;

static inline int get_pio_num(PIO pio) {
    switch ((int)pio) {
        case (int)pio0:
            return 0;
        case (int)pio1:
            return 1;
        default:
            return -1;
    }
}

void setup_encoder_pio(PIO pio) {
    pio_add_program(pio, &quadrature_encoder_program);
} 

// Uses 2 consecutive pins
void encoder_init(encoder_t *encoderStruct, PIO pio, uint sm, uint pin, int max_step_rate, float scale) {
    quadrature_encoder_program_init(pio, sm, pin, max_step_rate);
    encoderStruct->scale = scale;
    encoderStruct->pio = pio;
    encoderStruct->sm = sm;
}

void reset_encoder(encoder_t *encoderStruct) {
    offsets[(4*get_pio_num(encoderStruct->pio))+encoderStruct->sm] = quadrature_encoder_get_count(encoderStruct->pio, encoderStruct->sm);
}

float get_encoder(encoder_t *encoderStruct) {
    int offset = offsets[(4*get_pio_num(encoderStruct->pio))+encoderStruct->sm];
    return ((float)(quadrature_encoder_get_count(encoderStruct->pio, encoderStruct->sm) - offset)) * encoderStruct->scale;
}

