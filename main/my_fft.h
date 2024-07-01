#ifndef MY_FFT_H
#define MY_FFT_H

#include "stdlib.h"
#include "constants.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_dsp.h"
#include <math.h>
#include "my_uart_com.h"
#include "data_structs.h"

extern esp_err_t ret;

extern __attribute__((aligned(16))) float window[N_SAMPLES];				   // Window coefficients
extern __attribute__((aligned(16))) float data_re_im[N_SAMPLES * 2];		   // Real and imaginary part of sampled data [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
extern __attribute__((aligned(16))) indexed_float_type plot_output[N_SAMPLES]; // FFT output for magnitudes

void fft_init();
void fft_apply_window_on_fft_complex(float *data, float *window, float *data_re_im);
void fft_calculate_re_im(float *fft_components, uint32_t arr_len);
void fft_calculate_decibels(indexed_float_type *plot_output, uint32_t arr_len);
void fft_calculate_magnitudes(indexed_float_type *plot_output, uint32_t arr_len);
void fft_plot_decibels(int length, int min, int max);
void fft_plot_magnitudes(int length, int min, int max);
int compare_indexed_floats_descending(const void *a, const void *b);
uint32_t fft_number_of_percentile_elements(float percentile, int array_length);

#endif // MY_FFT_H
