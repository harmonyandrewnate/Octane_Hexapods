typedef struct{
    float target;
    float P;
    float I;
    float D;
    float last;
    int T_us;
    float accumulator;
} PID_cfg;


// Period is in us
void init_PID(PID_cfg *cfg, float P, float I, float D, int period){
    cfg->P = P;
    cfg->I = I;
    cfg->D = D;
    cfg->T_us = period;
    cfg->last = 0;
    cfg->accumulator = 0;
}

inline void set_PID_target(PID_cfg *cfg, float target){
    cfg->target = target;
}

inline void set_PID_P(PID_cfg *cfg, float P){
    cfg->P = P;
}

inline void set_PID_I(PID_cfg *cfg, float I){
    cfg->I = I;
}

inline void set_PID_D(PID_cfg *cfg, float D){
    cfg->D = D;
}

inline int update_PID(PID_cfg *cfg, float current){
    float last = cfg->last;
    cfg->last = current;
    
    float error = current - cfg->target;
    float P_power = error * cfg->P;
    
    cfg->accumulator += error;
    float I_power = cfg->accumulator * cfg->I;
    
    float vel = current - last;
    float D_power = vel * cfg->D;

    return P_power + I_power + D_power;
}

