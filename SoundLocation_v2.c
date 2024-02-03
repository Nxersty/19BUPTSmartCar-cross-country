#include "math.h"
#include "debug.h"
#include "timer.h"
#include "SEEKFREE_ASSISTANT.h"

#define FFT_SIZE 1024
#define MIC_DISTANCE 0.1  // 麦克风间距，单位：米
#define SOUND_SPEED 340.0  // 声速，单位：米/秒

float mic1_input[FFT_SIZE * 2];
float mic2_input[FFT_SIZE * 2];
float mic1_spectrum[FFT_SIZE * 2];
float mic2_spectrum[FFT_SIZE * 2];
float mic1_output[FFT_SIZE * 2];
float mic2_output[FFT_SIZE * 2];

uint32_t fft_count_time_us = 0;

// 初始化低通滤波器
void init_low_pass_filter(float *coefficients, float cutoff_frequency, float sampling_rate, int filter_size) {
    float omega_c = 2.0 * M_PI * cutoff_frequency / sampling_rate;
    float alpha = sin(omega_c) / (2.0 * filter_size);

    for (int i = 0; i < filter_size; ++i) {
        coefficients[i] = alpha;
    }
    coefficients[filter_size / 2] = 1.0 - alpha;

    for (int i = filter_size / 2 + 1; i < filter_size; ++i) {
        coefficients[i] = 0.0;
    }
}

// 应用低通滤波器
void apply_low_pass_filter(float *input, float *output, float *filter_coefficients, int signal_size, int filter_size) {
    for (int i = filter_size - 1; i < signal_size; ++i) {
        output[2 * i] = 0.0;
        output[2 * i + 1] = 0.0;

        for (int j = 0; j < filter_size; ++j) {
            output[2 * i] += input[2 * (i - j)] * filter_coefficients[j];
            output[2 * i + 1] += input[2 * (i - j) + 1] * filter_coefficients[j];
        }
    }
}

// 预处理函数
void preprocess(float *input, int length, float cutoff_frequency = 1000.0, float sampling_rate = 44100.0, int filter_size = 32) {
    // 初始化低通滤波器
    float low_pass_filter_coefficients[filter_size];
    init_low_pass_filter(low_pass_filter_coefficients, cutoff_frequency, sampling_rate, filter_size);

    // 应用低通滤波器到 mic1_input，结果保存到 mic1_output
    apply_low_pass_filter(input, mic1_output, low_pass_filter_coefficients, length, filter_size);

    // 应用低通滤波器到 mic2_input，结果保存到 mic2_output
    apply_low_pass_filter(input, mic2_output, low_pass_filter_coefficients, length, filter_size);

}

void perform_fft(float *input, float *spectrum) {
    // 初始化FFT对象
    arm_cfft_instance_f32 fft_instance;
    arm_cfft_init_f32(&fft_instance, FFT_SIZE);
    
    // 执行FFT运算
    arm_cfft_f32(&fft_instance, input, 0, 1);
    
    // 将FFT结果转换为幅度谱
    arm_cmplx_mag_f32(input, spectrum, FFT_SIZE);
}

float calculate_angle(float frequency, float mic_distance) {
    // 计算时间差
    float time_difference = mic_distance * sin(2 * M_PI * frequency * mic_distance / SOUND_SPEED);
    
    // 计算角度，角度转换为度
    float angle = asin(time_difference / mic_distance) * (180.0 / M_PI);
    
    return angle;
}

void sound_beacon_localization() {
    // 初始化系统
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    // 初始化输入信号
    for (int i = 0; i < FFT_SIZE; i++) {
        mic1_input[2*i] = i * 0.1024;  // 虚拟赋值，实际声音信号由ADC采集，将输入填入实部，虚部为0
        mic1_input[2*i + 1] = 0;
        mic2_input[2*i] = i * 0.1024;  // 虚拟赋值，实际声音信号由ADC采集，将输入填入实部，虚部为0
        mic2_input[2*i + 1] = 0;
    }

    // 信号预处理
    preprocess(mic1_input, FFT_SIZE*2);
    preprocess(mic2_input, FFT_SIZE*2);

    // 初始化定时器
    timer_init(TC_TIME2_CH0, TIMER_US);
    timer_start(TC_TIME2_CH0);

    // 执行FFT运算
    perform_fft(mic1_input, mic1_spectrum);
    perform_fft(mic2_input, mic2_spectrum);

    // 记录FFT运算时长
    fft_count_time_us = timer_get(TC_TIME2_CH0);
    timer_clear(TC_TIME2_CH0);
    timer_stop(TC_TIME2_CH0);

    // 初始化逐飞助手组件
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_oscilloscope_data.channel_num = 1;

    // 遍历FFT结果，计算频率对应的声音源角度
    for (int i = 0; i < FFT_SIZE; i++) {
        float frequency = i * (SOUND_SPEED / (2 * MIC_DISTANCE)) / FFT_SIZE;
        float angle1 = calculate_angle(frequency, MIC_DISTANCE);
        float angle2 = calculate_angle(frequency, MIC_DISTANCE);
    }

    // 进入死循环
    while (1) {
    }
}

