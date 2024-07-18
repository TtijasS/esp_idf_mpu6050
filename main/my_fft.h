#ifndef MY_FFT_H
#define MY_FFT_H

#include "stdlib.h"
#include "constants.h"
#include "esp_log.h"d
#include "esp_system.h"
#include "esp_dsp.h"
#include <math.h>
#include "my_uart_com.h"
#include "data_structs.h"

extern esp_err_t ret;

extern __attribute__((aligned(16))) float fft_window_arr[N_SAMPLES];					  // Window coefficients
extern __attribute__((aligned(16))) float fft_complex_arr[N_SAMPLES * 2];				  // Real and imaginary part of sampled data [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
extern __attribute__((aligned(16))) indexed_float_type fft_magnitudes_arr[N_SAMPLES / 2]; // FFT output for magnitudes

void fft_init();
void fft_prepare_window();
void fft_prepare_complex_arr(float *data, float *window, float *fft_components, uint32_t arr_len);
void fft_calculate_re_im(float *fft_components, uint32_t arr_len);
void fft_calculate_magnitudes(indexed_float_type *magnitudes_indexed, uint32_t arr_len);
void fft_sort_magnitudes(indexed_float_type *base_array, uint32_t arr_len);
void fft_plot_magnitudes(uint32_t length, int min, int max);
int compare_indexed_float_type_descending(const void *, const void *);
uint32_t fft_percentile_n_components(float percentile, uint32_t arr_len);
void fft_prepare_metadata_buffer(uint8_t *metadata_buffer, uint32_t n_samples, uint32_t n_components);
void fft_prepare_uart_data_buffer(uint8_t *data_buffer, uint32_t n_msb_components, indexed_float_type *magnitudes, float *fft_components);
void fft_send_msb_components_over_uart(uint32_t n_samples, uint32_t n_msb_elements);
void fft_debug_uart_buffers(uint8_t *metadata_buffer, uint32_t metadata_size, uint8_t *data_buffer, uint32_t data_size);

#endif // MY_FFT_H
