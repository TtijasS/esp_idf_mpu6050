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
extern __attribute__((aligned(16))) float fft_components[N_SAMPLES * 2];		   // Real and imaginary part of sampled data [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
// extern __attribute__((aligned(16))) float magnitudes[N_SAMPLES];
extern __attribute__((aligned(16))) indexed_float_type magnitudes_indexed[N_SAMPLES]; // FFT output for magnitudes

void fft_init();
void fft_apply_window_on_fft_complex(float *data, float *window, float *fft_components);
void fft_calculate_re_im(float *fft_components, size_t arr_len);
void fft_calculate_magnitudes(indexed_float_type *magnitudes_indexed, size_t arr_len);
void fft_sort_magnitudes(indexed_float_type *base_array, size_t arr_len);
void fft_plot_magnitudes(size_t length, int min, int max);
int compare_indexed_floats_descending(const void *a, const void *b);
size_t fft_percentile_n_components(float percentile, size_t arr_len);
void fft_prepare_metadata_buffer(uint8_t *metadata_buffer, size_t n_samples, size_t n_components);
void fft_prepare_uart_data_buffer(uint8_t *data_buffer, size_t buff_len, indexed_float_type *magnitudes, float *fft_components);
void fft_send_percentiles_over_uart(float percentile, size_t arr_len);



#endif // MY_FFT_H
