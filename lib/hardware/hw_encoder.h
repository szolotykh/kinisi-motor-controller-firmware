//------------------------------------------------------------
// File name: encoder.h
//------------------------------------------------------------

#pragma once

#define ENCODER0 0
#define ENCODER1 1
#define ENCODER2 2
#define ENCODER3 3

typedef unsigned char encoder_index;

extern void initialize_encoder(encoder_index index);
extern unsigned int get_encoder_value(encoder_index index);
