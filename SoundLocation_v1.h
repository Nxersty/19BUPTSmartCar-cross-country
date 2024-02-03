#ifndef _SoundLocation_v1_h
#define _SoundLocation_v1_h


#include <math.h>
#include "debug.h"
#include "timer.h"
#include "SEEKFREE_ASSISTANT.h"

direction_t locateSound(sint16* s1, sint16* s2, sint16* s3, sint16* s4);
float absFloat(float x);
int findMaxIndex(sint32* src, int len);
void xcorr(sint16* a, sint16* b, sint32* out);
void switchLED(direction_t direction);

#end