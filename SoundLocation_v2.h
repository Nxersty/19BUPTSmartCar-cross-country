#ifndef _SoundLocation_v2_h
#define _SoundLocation_v2_h


#include <math.h>
#include "debug.h"
#include "timer.h"
#include "SEEKFREE_ASSISTANT.h"

void init_low_pass_filter(float *coefficients, float cutoff_frequency, float sampling_rate, int filter_size);
void apply_low_pass_filter(float *input, float *output, float *filter_coefficients, int signal_size, int filter_size);
void preprocess(float *input, int length, float cutoff_frequency = 1000.0, float sampling_rate = 44100.0, int filter_size = 32);
void perform_fft(float *input, float *spectrum);
float calculate_angle(float frequency, float mic_distance);
void sound_beacon_localization();

#end