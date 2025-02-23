//------------------------------------------------------------
// File name: encoder.h
//------------------------------------------------------------

#pragma once

#include <stdint.h>

#define ENCODER0 0
#define ENCODER1 1
#define ENCODER2 2
#define ENCODER3 3

typedef unsigned char encoder_index_t;

// Function pointer types for mocking
typedef void (*initialize_encoder_fn)(encoder_index_t index, double encoder_resolution, uint8_t is_reversed);
typedef uint16_t (*get_encoder_value_fn)(encoder_index_t index);
typedef uint8_t (*get_encoder_direction_fn)(encoder_index_t index);
typedef uint8_t (*encoder_is_initialized_fn)(encoder_index_t index);
typedef double (*encoder_get_resolution_fn)(encoder_index_t index);

// Interface structure
typedef struct {
    initialize_encoder_fn initialize;
    get_encoder_value_fn get_value;
    get_encoder_direction_fn get_direction;
    encoder_is_initialized_fn is_initialized;
    encoder_get_resolution_fn get_resolution;
} hw_encoder_interface_t;

// Get the encoder interface implementation
const hw_encoder_interface_t* get_encoder_interface(void);

// Set mock interface for testing
void hw_encoder_set_interface(const hw_encoder_interface_t* interface);

// Initialize encoder interface
void hw_encoder_init(void);

// Default implementations
extern void initialize_encoder(encoder_index_t index, double encoder_resolution, uint8_t is_reversed);
extern uint16_t get_encoder_value(encoder_index_t index);
extern uint8_t get_encoder_direction(encoder_index_t index);
extern uint8_t encoder_is_initialized(encoder_index_t index);
extern double encoder_get_resolution(encoder_index_t index);
