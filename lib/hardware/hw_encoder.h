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

extern void initialize_encoder(encoder_index_t index, double encoder_resolution, uint8_t is_reversed);
extern uint16_t get_encoder_value(encoder_index_t index);
extern uint8_t get_encoder_direction(encoder_index_t index);

/* Check if encoder is initialized
Parameters:
    index: Encoder index
*/
extern uint8_t encoder_is_initialized(encoder_index_t index);

/* Get encoder resolution
Parameters:
    index: Encoder index
*/
extern double encoder_get_resolution(encoder_index_t index);
