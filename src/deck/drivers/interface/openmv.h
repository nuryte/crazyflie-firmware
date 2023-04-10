/* Copyright (C) 2023 Michael Fitzgerald */

#include <stdbool.h>
#include <stdint.h>

typedef struct _openmv_state_t {
    bool active;
    int8_t target_x, target_y; // [-128,127] x [-128,127] in pixels
    uint8_t target_z; // [0,255] in 1/10 meters
} openmv_state_t;

void omvGetState(openmv_state_t* s);