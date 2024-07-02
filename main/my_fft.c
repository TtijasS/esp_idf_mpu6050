#include "my_fft.h"

esp_err_t ret;

__attribute__((aligned(16))) float window[N_SAMPLES];                          // Window coefficients
__attribute__((aligned(16))) float fft_components[N_SAMPLES * 2];              // Real and imaginary part of sampled data_sampled [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) indexed_float_type magnitudes_indexed[N_SAMPLES]; // FFT output for plotting

void fft_init()
{
    ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in FFT init: %s", esp_err_to_name(ret));
    }
}

void fft_apply_window_on_fft_complex(float *data_sampled, float *window, float *fft_components)
{
    // Prepare window coefficients
    dsps_wind_hann_f32(window, N_SAMPLES);
    // dsps_wind_blackman_f32(window, N_SAMPLES);

    // Prepare fft_components array
    for (int i = 0; i < N_SAMPLES; i++)
    {
        // [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
        fft_components[2 * i] = data_sampled[i] * window[i]; // Real part (@ i-th index)
        fft_components[2 * i + 1] = 0;                       // Imaginary part (@ i + 1 index)
    }
}

/**
 * @brief Run DFFT and calculate real and imaginary componenty of the signal
 *
 * Calculate re and im parts of the sampled signal
 * and store results into fft_components array of length N_SAMPLES*2
 */
void fft_calculate_re_im(float *fft_components, size_t arr_len)
{
    // Perform FFT
    dsps_fft2r_fc32(fft_components, arr_len);
    // Bit reverse
    dsps_bit_rev_fc32(fft_components, arr_len);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(fft_components, arr_len);
}

/**
 * @brief Calculate magnitudes from FFT results
 *
 * First run fft_calculate_im_re, then convert components to magnitudes (sqrtf scale)
 *
 * @param magnitudes_indexed: pointer to array of structs with index and magnitude value
 * @param arr_len: length of magnitudes_indexed array
 */
void fft_calculate_magnitudes(indexed_float_type *magnitudes_indexed, size_t arr_len)
{
    for (int i = 0; i < arr_len / 2; i++)
    {
        magnitudes_indexed[i].index = i;
        magnitudes_indexed[i].value = sqrtf(((fft_components[i * 2] * fft_components[i * 2]) + (fft_components[i * 2 + 1] * fft_components[i * 2 + 1])) / N_SAMPLES);
    }
}

/**
 * @brief Sort the base_array of indexed_float_type in descending order.
 *
 * @param base_array: The array you wish to sort (magnitudes_indexed)
 * @param arr_len: length of the base_array
 */
void fft_sort_magnitudes(indexed_float_type *base_array, size_t arr_len)
{
    qsort(base_array, arr_len / 2, sizeof(indexed_float_type), compare_indexed_floats_descending);
}

/**
 * @brief Debug plot the magnitudes of the magnitudes_indexed
 *
 * First calculate real and imaginary coponents, then run fft_calculate_magnitudes.
 * After that you can plot magnitudes as a simple asci char plot.
 *
 * @param arr_len: length of magnitudes_indexed array
 * @param min: plot scale min
 * @param max: plot scale max
 */
void fft_plot_magnitudes(size_t arr_len, int min, int max)
{
    // Display FFT output
    printf("Plot mag:\n");
    fft_calculate_magnitudes(magnitudes_indexed, N_SAMPLES);

    // construct tmp magnitudes float array
    float magnitudes[arr_len];
    for (size_t i = 0; i < arr_len; i++)
    {
        magnitudes[i] = magnitudes_indexed[i].value;
    }

    dsps_view(magnitudes, arr_len / 2, 64, 16, min, max, '|');
}

/**
 * @brief Comparison function of two indexed_float_type structs
 *
 * Compares struct_a.value and struct_b.value in a way,
 * that qsort() will sort the array in descending order.
 *
 * @param a
 * @param b
 * @return int
 */
int compare_indexed_floats_descending(const void *a, const void *b)
{
    float a_val = (const float)((const indexed_float_type *)a)->value;
    float b_val = (const float)((const indexed_float_type *)b)->value;

    return (a_val > b_val) - (a_val < b_val);
}

/**
 * @brief Calculate how many elements to print out depending on the percentile
 *
 * If we have an array of size 100, then 95th percentile index is i = 95,
 * therefore in a sorted array first (100-95)=5 elements are 95th percentile.
 *
 * @param percentile: float < 100
 * @param arr_len: array length
 * @return size_t
 */
size_t fft_percentile_n_components(float percentile, size_t arr_len)
{
    return arr_len - ((percentile / 100) * arr_len);
}

/**
 * @brief Prepare uart buffer with the most significant fft components
 *
 * @param data_buffer: buffer that will be filled with components
 * @param buff_len: length of the buffer
 * @param magnitudes: magnitudes array
 * @param fft_components: real and imaginary fft components
 */
void fft_prepare_uart_data_buffer(uint8_t *data_buffer, size_t buff_len, indexed_float_type *magnitudes, float *fft_components)
{
    size_t _index = 0;
    float *_value;

    for (size_t i = 0; i < buff_len; i++)
    {
        _index = magnitudes[i].index;
        _value = &magnitudes[i].value;

        // Copy [magnitude, real, imaginary] parts to data buffer
        memcpy(&data_buffer[i * 3 * sizeof(float)], _value, sizeof(float));
        memcpy(&data_buffer[(i * 3 + 1) * sizeof(float)], &fft_components[_index * 2], sizeof(float) * 2);
    }
}

/**
 * @brief Prepare metadata that will be sent over uart
 * 
 * We need send the number of samples that were used in fft calculation and how many most significant components were sent over.
 * 
 * @param metadata_buffer: pointer to buffer array that will be prepared
 * @param n_samples: number of samples used in fft calculation
 * @param n_components: number of most significant components that will get sent over uart
 */
void fft_prepare_metadata_buffer(uint8_t *metadata_buffer, size_t n_samples, size_t n_components)
{
    memcpy(&metadata_buffer[0], &n_components, sizeof(size_t));
    memcpy(&metadata_buffer[sizeof(size_t)], &n_samples, sizeof(size_t));
}

/**
 * @brief Send the first n components (magnitude, re, im) of DFFT calculation
 *
 * @param percentile: float < 100
 * @param arr_len: array length
 */
void fft_send_percentiles_over_uart(float percentile, size_t arr_len)
{
    // Sort the magnitudes array
    qsort(magnitudes_indexed, arr_len, sizeof(indexed_float_type), compare_indexed_floats_descending);
    // Calculate how many most significant components to print out
    size_t n_components = fft_percentile_n_components(percentile, arr_len / 2);

    // Prepare the data_buffer for n largest elements.
    // Each element consists of [magnitude, real, imaginary], therefore *3
    uint8_t data_buffer[sizeof(float) * n_components * 3];
    fft_prepare_uart_data_buffer(data_buffer, n_components, magnitudes_indexed, fft_components);

    uint8_t metadata_buffer[sizeof(size_t) * 2];
    fft_prepare_metadata_buffer(metadata_buffer, N_SAMPLES, n_components);

    // Send over the metadata
    uart_write_bytes(uart_num, (const char *)metadata_buffer, sizeof(size_t)*2);

    // Prepare buffer for 3 floats: magnitude, real, imaginary
    uart_write_bytes(uart_num, (const char *)data_buffer, sizeof(float) * 3 * n_components);
}
