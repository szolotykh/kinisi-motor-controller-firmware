//------------------------------------------------------------
// File name: utils.h
//------------------------------------------------------------

#pragma once

void print_controller_state(unsigned int seq, int velocity, int targetVelocity, int pwm);

double decode_double(char* bytes);