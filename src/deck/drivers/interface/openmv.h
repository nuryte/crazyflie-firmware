/* Copyright (C) 2023 Michael Fitzgerald */

#include <stdbool.h>
#include <stdint.h>

typedef struct _openmv_state_t {
    bool active;
    int8_t target_x, target_y;
    uint8_t target_z;
} openmv_state_t;

extern openmv_state_t openmv_state;