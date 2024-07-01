#include "my_fft.h"

esp_err_t ret;

__attribute__((aligned(16))) float window[N_SAMPLES];                   // Window coefficients
__attribute__((aligned(16))) float data_re_im[N_SAMPLES * 2];           // Real and imaginary part of sampled data_sampled [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) indexed_float_type plot_output[N_SAMPLES]; // FFT output for plotting

void fft_init()
{
    ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in FFT init: %s", esp_err_to_name(ret));
    }
}

void fft_apply_window_on_fft_complex(float *data_sampled, float *window, float *data_re_im)
{
    // Prepare window coefficients
    dsps_wind_hann_f32(window, N_SAMPLES);
    // dsps_wind_blackman_f32(window, N_SAMPLES);

    // Prepare data_re_im array
    for (int i = 0; i < N_SAMPLES; i++)
    {
        // [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
        data_re_im[2 * i] = data_sampled[i] * window[i]; // Real part (@ i-th index)
        data_re_im[2 * i + 1] = 0;                       // Imaginary part (@ i + 1 index)
    }
}

/**
 * @brief Run DFFT and calculate real and imaginary componenty of the signal
 *
 * Calculate re and im parts of the sampled signal
 * and store results into data_re_im array of length N_SAMPLES*2
 */
void fft_calculate_re_im(float *fft_components, uint32_t arr_len)
{
    // Perform FFT
    dsps_fft2r_fc32(fft_components, arr_len);
    // Bit reverse
    dsps_bit_rev_fc32(fft_components, arr_len);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(fft_components, arr_len);
}

/**
 * @brief Calculate decibels from FFT results
 *
 * First run fft_calculate_im_re, then convert components to decibels (log10 scale)
 *
 * @param plot_output: pointer to array of structs with index and decibels value
 * @param arr_len: length of plot_output array
 */
void fft_calculate_decibels(indexed_float_type *plot_output, uint32_t arr_len)
{
    for (int i = 0; i < arr_len / 2; i++)
    {
        plot_output[i].index = i;
        plot_output[i].value = 10 * log10f(((data_re_im[i * 2] * data_re_im[i * 2]) + (data_re_im[i * 2 + 1] * data_re_im[i * 2 + 1])) / N_SAMPLES);
    }
}

/**
 * @brief Calculate magnitudes from FFT results
 *
 * First run fft_calculate_im_re, then convert components to magnitudes (sqrtf scale)
 *
 * @param plot_output: pointer to array of structs with index and magnitude value
 * @param arr_len: length of plot_output array
 */
void fft_calculate_magnitudes(indexed_float_type *plot_output, uint32_t arr_len)
{
    for (int i = 0; i < arr_len / 2; i++)
    {
        plot_output[i].index = i;
        plot_output[i].value = sqrtf(((data_re_im[i * 2] * data_re_im[i * 2]) + (data_re_im[i * 2 + 1] * data_re_im[i * 2 + 1])) / N_SAMPLES);
    }
}

/**
 * @brief Debug plot the decibels of the plot_output
 *
 * First calculate im and re coponents, then run fft_calculate_decibels.
 * After that you can plot decibels as a simple asci char plot.
 *
 * @param arr_len: length of plot_output array
 * @param min: plot scale min
 * @param max: plot scale max
 */
void fft_plot_decibels(int arr_len, int min, int max)
{
    // Display FFT output
    printf("Plot dB:\n");
    fft_calculate_decibels(data_re_im, N_SAMPLES);
    dsps_view(plot_output, arr_len / 2, 64, 16, min, max, '|');
}

/**
 * @brief Debug plot the magnitudes of the plot_output
 *
 * First calculate real and imaginary coponents, then run fft_calculate_magnitudes.
 * After that you can plot magnitudes as a simple asci char plot.
 *
 * @param arr_len: length of plot_output array
 * @param min: plot scale min
 * @param max: plot scale max
 */
void fft_plot_magnitudes(int arr_len, int min, int max)
{
    // Display FFT output
    printf("Plot mag:\n");
    fft_calculate_magnitudes(data_re_im, N_SAMPLES);
    dsps_view(plot_output, arr_len / 2, 64, 16, min, max, '|');
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
 * If we have an array of size 100, then 95th percentile index is i = 95
 * If arr size is 200, then 95th percentile index is i = 190
 *
 * @param percentile
 * @param arr_len
 * @return float
 */
uint32_t fft_number_of_percentile_elements(float percentile, uint32_t arr_len)
{
    return arr_len - ((percentile / 100) * arr_len);
}

/**
 * @brief Send the first n components (magnitude, re, im) of DFFT calculation
 *
 * @param percentile
 * @param arr_len
 */
void fft_send_percentiles_over_uart(float percentile, uint32_t arr_len)
{
    // Calculate how many components to print out
    uint32_t n_elements = fft_number_of_percentile_elements(percentile, arr_len);

    // Prepare data_buffer for n largest elements.
    // Each consists of [magnitude, real, imaginary], therefore *3
    uint8_t data[sizeof(float) * n_elements * 3];

    for (int i = 0; i < n_elements; i++)
    {
        memcpy(&data[i*3], &data_re_im[i*2], sizeof(float)*2);
    }
    // Prepare buffer for 3 floats: magnitude, real, imaginary
    uart_write_bytes(uart_num, (const char *)data, sizeof(float) * 3);
}

void fft_print_percentiles(float *percentiles_output, int arr_len, float percentile)
{
    // Sort the indexed array of pointers
    qsort(plot_output, arr_len / 2, sizeof(float *), compare_indexed_floats_descending);

    float percentile_index = fft_number_of_percentile_elements(percentile, arr_len);

    for (int i = 0; i <= percentile_index; i++)
    {
    }
}
