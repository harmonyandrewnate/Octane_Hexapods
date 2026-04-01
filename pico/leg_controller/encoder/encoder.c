#include "quadrature_encoder.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

int offsets[8] = {0};

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
void encoder_init(PIO pio, uint sm, uint pin, int max_step_rate) {
    quadrature_encoder_program_init(pio, sm, pin, max_step_rate);
}

void reset_encoder(PIO pio, uint sm) {
    offsets[(4*get_pio_num(pio))+sm] = quadrature_encoder_get_count(pio, sm);
}

int32_t get_encoder(PIO pio, uint sm) {
    int offset = offsets[(4*get_pio_num(pio))+sm];
    return (quadrature_encoder_get_count(pio, sm) - offset);
}

