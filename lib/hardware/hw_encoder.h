//------------------------------------------------------------
// File name: encoder.h
//------------------------------------------------------------

#include <stdint.h>

#pragma once

#define ENCODER0 0
#define ENCODER1 1
#define ENCODER2 2
#define ENCODER3 3

typedef unsigned char encoder_index_t;

extern void initialize_encoder(encoder_index_t index);
extern uint16_t get_encoder_value(encoder_index_t index);
extern uint8_t get_encoder_direction(encoder_index_t index);
