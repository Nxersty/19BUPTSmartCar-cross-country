#include <math.h>
#include "debug.h"
#include "timer.h"
#include "SEEKFREE_ASSISTANT.h"

#define XCORR_SIZE 1024
#define XCORR_MIDDLE (XCORR_SIZE / 2)
#define DIRECTION_THRESHOLD 1
#define FFT_SIZE XCORR_SIZE

typedef short  sint16;
typedef long   sint32;
typedef enum direction_t {
    Forward = 0, FrontRight = 1, RightFront = 2, RightBack = 3, BackRight = 4,
    BackLeft = 5, LeftBack = 6, LeftFront = 7, FrontLeft = 8
} direction_t;

sint32 r12[XCORR_SIZE];
sint32 r13[XCORR_SIZE];
sint32 r14[XCORR_SIZE];
sint32 r24[XCORR_SIZE];

float inputSignal[XCORR_SIZE * 2];
float outputSignal[XCORR_SIZE * 2];

unsigned long fft_count_time_us = 0;
arm_cfft_instance_f32 arm_cfft_instance_f32_len_1024;


direction_t locateSound(sint16* s1, sint16* s2, sint16* s3, sint16* s4);
float absFloat(float x);
int findMaxIndex(sint32* src, int len);

float absFloat(float x) {
    if (x < 0) {
        x = -x;
    }
    return x;
}

int findMaxIndex(sint32* src, int len) {
    int i;
    int maxIndex = 0;
    for (i = 0; i < len; i++) {
        if (src[i] > src[maxIndex]) {
            maxIndex = i;
        }
    }
    return maxIndex;
}

void xcorr(sint16* a, sint16* b, sint32* out)
{
    // 根据您实际的FFT库和配置修改此函数
    arm_cfft_f32(&arm_cfft_instance_f32_len_1024, (float*)b, 0, 1);  // 对输入信号 b 进行FFT
    arm_cmplx_mult_cmplx_f32((float*)a, (float*)b, (float*)out, XCORR_SIZE);  // 频域乘法
    arm_cfft_f32(&arm_cfft_instance_f32_len_1024, (float*)out, 1, 1);  // 对结果进行IFFT，转换回时域
    // 根据实际需求可能需要进行额外的处理
}

direction_t locateSound(sint16* s1, sint16* s2, sint16* s3, sint16* s4)
{
    direction_t direction = Forward;
    xcorr(s1,s2,r12);
    xcorr(s1,s3,r13);
    if(findMaxIndex(r12, XCORR_SIZE) < XCORR_MIDDLE)
    {
        if (findMaxIndex(r13, XCORR_SIZE) < XCORR_MIDDLE) {
            xcorr(s1, s4, r14);
            if (findMaxIndex(r14, XCORR_SIZE) < XCORR_MIDDLE) {
                xcorr(s2, s4, r24);
                if (findMaxIndex(r24, XCORR_SIZE) < XCORR_MIDDLE) {
                    direction = RightBack;
                }
                else {
                    direction = BackRight;
                }
            }
            else {
                direction = BackLeft;
            }
        }
        else {
            direction = LeftBack;
        }
    }
    else {
        if (findMaxIndex(r13, XCORR_SIZE) < XCORR_MIDDLE) {
            direction = RightFront;
        }
        else {
            xcorr(s1, s4, r14);
            if (findMaxIndex(r14, XCORR_SIZE) < XCORR_MIDDLE) {
                direction = FrontRight;
            }
            else {
                xcorr(s2, s4, r24);
                if (findMaxIndex(r24, XCORR_SIZE) < XCORR_MIDDLE) {
                    direction = FrontLeft;
                }
                else {
                    direction = LeftFront;
                }
            }
        }
    }
    if (direction == FrontRight || direction == FrontLeft) {
        if (absFloat(findMaxIndex(r14, XCORR_SIZE) - XCORR_MIDDLE) < DIRECTION_THRESHOLD) {
            direction = Forward;
        }
    }
    return direction;
}

/*void switchLED(direction_t direction) {
    LED_Ctrl(LEDALL, OFF);
    switch (direction) {
        case FrontRight:
        case RightFront:
            LED_Ctrl(LED3, ON);
            break;
        case BackRight:
        case RightBack:
            LED_Ctrl(LED2, ON);
            break;
        case FrontLeft:
        case LeftFront:
            LED_Ctrl(LED0, ON);
            break;
        case BackLeft:
        case LeftBack:
            LED_Ctrl(LED1, ON);
            break;
        case Forward:
            LED_Ctrl(LED4, ON);
            break;
        default:
            LED_Ctrl(LEDALL, ON);
            break;
    }
}*/
