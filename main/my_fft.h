#ifndef MY_FFT_H
#define MY_FFT_H

#include "constants.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_dsp.h"
#include <math.h>

#define N_SAMPLES 1024					 // Number of samples to collect before running FFT

extern esp_err_t ret;

extern __attribute__((aligned(16))) float window[N_SAMPLES];         // Window coefficients
extern __attribute__((aligned(16))) float data_complex[N_SAMPLES*2]; // Real and imaginary part of sampled data [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
extern __attribute__((aligned(16))) float fft_output[N_SAMPLES];     // FFT output for magnitudes

void fft_init();
void fft_apply_window_on_fft_complex(float *data, float *window, float *data_complex);
void fft_calculate(float *data_complex, int length);
void fft_run_with_hann(float *data, int length);

#endif // MY_FFT_H
