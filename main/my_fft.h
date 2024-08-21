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

void fft_init();
void fft_prepare_window(float *window_arr);
void fft_prepare_complex_arr(float *sampled_data_arr, float *complex_arr, uint32_t arr_len);
void fft_calculate_re_im(float *fft_components, uint32_t n_samples);
void fft_calculate_magnitudes(indexed_float_type *indexed_magnitudes_arr, float *fft_complex_arr, uint32_t magnitudes_size);
void fft_sort_magnitudes(indexed_float_type *indexed_mangitudes, uint32_t magnitudes_size);
void fft_plot_magnitudes(indexed_float_type *indexed_magnitudes, uint32_t length, int min, int max);
int compare_indexed_float_type_descending(const void *, const void *);
uint32_t fft_percentile_n_components(float percentile, uint32_t arr_len);
int fft_prepare_metadata_buffer(uint8_t *metadata_buffer, size_t metadata_size, uint32_t n_samples, uint32_t n_components);
int fft_prepare_magnitudes_buffer(uint8_t *indices_buffer, size_t indices_size, uint8_t *magnitudes_buffer, size_t magnitudes_size, indexed_float_type *indexed_magnitudes, uint32_t n_ms_components);
int fft_prepare_complex_buffer(uint8_t *complex_data_buffer, size_t complex_size, uint32_t n_fft_components, indexed_float_type *indexed_mangitudes, float *fft_components);
int fft_send_ms_components_over_uart(float *fft_complex_arr, indexed_float_type *indexed_magnitudes, uint32_t n_samples, uint32_t n_ms_elements);
int fft_debug_uart_buffers(uint8_t *metadata_buffer, size_t metadata_size, uint8_t *indices_buffer, size_t indices_size, uint8_t *magnitudes_buffer, size_t magnitudes_size, uint8_t *complex_data_buffer, size_t complex_size);

#endif // MY_FFT_H
