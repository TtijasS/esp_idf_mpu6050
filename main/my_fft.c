#include "my_fft.h"

esp_err_t ret;

__attribute__((aligned(16))) float fft_window_arr[N_SAMPLES];                      // Window coefficients
__attribute__((aligned(16))) float fft_complex_arr[N_SAMPLES * 2];                 // Real and imaginary part of sampled data_sampled [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) indexed_float_type fft_magnitudes_arr[N_SAMPLES]; // FFT output for plotting

void fft_init()
{
    ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in FFT init: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "FFT init complete");
}

/**
 * @brief Prepare window before constructing complex array
 *
 */
void fft_prepare_window()
{
    dsps_wind_hann_f32(fft_window_arr, N_SAMPLES);
}

/**
 * @brief Merge sampled datan and window coefficients into complex array
 *
 * @param sampled_data_arr: array with sensor data
 * @param window_arr: array with window coefficients
 * @param complex_arr: array that stores real and imaginary parts of the signal
 * @param arr_len: length of the sampled_data_arr
 */
void fft_prepare_complex_arr(float *sampled_data_arr, float *window_arr, float *complex_arr, uint32_t arr_len)
{
    if (sampled_data_arr == NULL || window_arr == NULL || complex_arr == NULL)
    {
        ESP_LOGE(TAG, "Null pointer error in fft_prepare_complex_arr");
        return;
    }
    // Prepare fft_components array
    for (int i = 0; i < arr_len; i++)
    {
        // [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
        complex_arr[2 * i] = sampled_data_arr[i];// * window_arr[i]; // Real part (@ i-th index)
        complex_arr[2 * i + 1] = 0;                               // Imaginary part (@ i + 1 index)
    }
}

/**
 * @brief Run DFFT and calculate real and imaginary componenty of the signal
 *
 * Calculate re and im parts of the sampled signal
 * and store results into fft_components array of length N_SAMPLES*2
 */
void fft_calculate_re_im(float *complex_arr, uint32_t arr_len)
{
    ESP_ERROR_CHECK(dsps_fft2r_fc32(complex_arr, arr_len));

    ESP_ERROR_CHECK(dsps_bit_rev_fc32(complex_arr, arr_len));

    ESP_ERROR_CHECK(dsps_cplx2reC_fc32(complex_arr, arr_len));
}

/**
 * @brief Calculate magnitudes from FFT results
 *
 * First run fft_calculate_im_re, then convert components to magnitudes (sqrtf scale)
 *
 * @param magnitudes_indexed: pointer to array of structs with index and magnitude value
 * @param arr_len: length of magnitudes_indexed array
 */
void fft_calculate_magnitudes(indexed_float_type *indexed_magnitudes_arr, uint32_t arr_len)
{
    if (indexed_magnitudes_arr == NULL)
    {
        ESP_LOGE(TAG, "Null pointer error in fft_calculate_magnitudes");
        return;
    }

    if (arr_len == 0)
    {
        ESP_LOGE(TAG, "Array length is zero in fft_calculate_magnitudes");
        return;
    }

    for (int i = 0; i < arr_len; i++)
    {
        indexed_magnitudes_arr[i].index = i;
        indexed_magnitudes_arr[i].value = sqrtf((fft_complex_arr[i * 2] * fft_complex_arr[i * 2]) + (fft_complex_arr[i * 2 + 1] * fft_complex_arr[i * 2 + 1]) / N_SAMPLES);
    }
}

/**
 * @brief Sort the base_array of indexed_float_type in descending order.
 *
 * @param base_array: The array you wish to sort (magnitudes_indexed)
 * @param arr_len: length of the base_array
 */
void fft_sort_magnitudes(indexed_float_type *base_array, uint32_t arr_len)
{
    qsort(base_array, arr_len, sizeof(indexed_float_type), compare_indexed_float_type_descending);
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

void fft_plot_magnitudes(uint32_t arr_len, int min, int max)
{
    // construct tmp magnitudes float array
    float magnitudes[arr_len];
    for (uint32_t i = 0; i < arr_len; i++)
    {
        magnitudes[i] = fft_magnitudes_arr[i].value;
    }

    dsps_view(magnitudes, arr_len, 64, 16, min, max, '|');
}

/**
 * @brief Compare indexed_float_type structs in descending order
 *
 * @param a: value a
 * @param b: value b
 * @return int: 1 if a < b, -1 if a > b, 0 if a == b
 */
int compare_indexed_float_type_descending(const void *a, const void *b)
{
    float va = ((const indexed_float_type *)a)->value;
    float vb = ((const indexed_float_type *)b)->value;
    return (va < vb) - (va > vb);
}

/**
 * @brief Calculate how many elements to print out depending on the percentile
 *
 * If we have an array of size 100, then 95th percentile index is i = 95,
 * therefore in a sorted array first (100-95)=5 elements are 95th percentile.
 *
 * @param percentile: float < 100
 * @param arr_len: array length
 * @return uint32_t
 */
uint32_t fft_percentile_n_components(float percentile, uint32_t arr_len)
{
    return arr_len - ((percentile / 100) * arr_len);
}

/**
 * @brief Prepare uart buffer with the most significant fft components
 *
 * @param data_buffer: buffer that will be filled with components
 * @param n_fft_components: number of component groups to copy
 * @param magnitudes: magnitudes array
 * @param fft_components: real and imaginary fft components
 */
void fft_prepare_uart_data_buffer(uint8_t *data_buffer, uint32_t n_fft_components, indexed_float_type *magnitudes, float *fft_components)
{
    uint32_t _index = 0;
    float index_as_float = 0.0;

    for (uint32_t i = 0; i < n_fft_components; i++)
    {
        if (i >= MAGNITUDES_SIZE)
        {
            ESP_LOGE(TAG, "Index out of bounds in fft_prepare_uart_data_buffer");
            break;
        }
        _index = magnitudes[i].index;
        index_as_float = (float)_index;

        // Copy [index, real, imaginary] parts to data buffer
        // index is converted to float for standardization reasons
        memcpy(&data_buffer[i * 3 * sizeof(float)], &index_as_float, sizeof(float));
        memcpy(&data_buffer[(i * 3 + 1) * sizeof(float)], &fft_components[_index * 2], sizeof(float) * 2);
        // ESP_LOGI(TAG, "i %.6f, ma %.6f, re %.6f, im %.6f", index_as_float, magnitudes[i].value, fft_components[_index * 2], fft_components[_index * 2 + 1]);
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
void fft_prepare_metadata_buffer(uint8_t *metadata_buffer, uint32_t n_samples, uint32_t n_components)
{
    memcpy(&metadata_buffer[0], &n_samples, sizeof(uint32_t));
    memcpy(&metadata_buffer[sizeof(uint32_t)], &n_components, sizeof(uint32_t));
}

/**
 * @brief Send the first n components (magnitude, re, im) of DFFT calculation
 *
 * @param n_samples: number of samples taken during the vibration sampling process
 * @param n_ms_elements: number of most significant components to send over uart
 */
void fft_send_most_significant_components_over_uart(uint32_t n_samples, uint32_t n_ms_elements)
{
    // Prepare metadata buffer to send sample size and number of msb components
    uint8_t metadata_buffer[sizeof(uint32_t) * 2];
    // fft_prepare_metadata_buffer(metadata_buffer, n_samples, n_ms_elements);
    fft_prepare_metadata_buffer(metadata_buffer, 1024, 1024);

    // Prepare the data_buffer for n largest elements [magnitude, real, imaginary]
    uint8_t data_buffer[(sizeof(float) * 3) * n_ms_elements];
    fft_prepare_uart_data_buffer(data_buffer, n_ms_elements, fft_magnitudes_arr, fft_complex_arr);

    // Send data
    // uart_send_fft_components(metadata_buffer, sizeof(metadata_buffer), data_buffer, sizeof(data_buffer));

    uart_write_bytes(uart_num, "\xfd\xfd\xfd\xfd\xfd", 5); // Start of transmission
    uart_write_bytes(uart_num, (const char *)metadata_buffer, 8);
    uart_write_bytes(uart_num, "\xff\xfd\xfd\xfd\xff", 5); // Start of transmission

    uart_write_bytes(uart_num, "\xfe\xfe\xfe\xfe\xfe", 5); // Start of transmission
    for (int i = 0; i < N_SAMPLES; i++)
    {
        float index_as_float = (float)i;
        uart_write_bytes(uart_num, (const char *)&index_as_float, sizeof(float));
        uart_write_bytes(uart_num, (const char *)&fft_complex_arr[i*2], sizeof(float));
        uart_write_bytes(uart_num, (const char *)&fft_complex_arr[i*2 + 1], sizeof(float));
    }
    uart_write_bytes(uart_num, "\xff\xfe\xfe\xfe\xff", 5); // Start of transmission

    // fft_debug_uart_buffers(metadata_buffer, sizeof(metadata_buffer), data_buffer, sizeof(data_buffer));
}

/**
 * @brief Printing the metadata and data buffer for debugging purposes
 *
 * @param metadata_buffer: buffer with metadata uint8_t values
 * @param metadata_size: size of the metadata buffer
 * @param data_buffer: buffer with data float values
 * @param data_size: size of the data buffer
 */
void fft_debug_uart_buffers(uint8_t *metadata_buffer, uint32_t metadata_size, uint8_t *data_buffer, uint32_t data_size)
{
    // construct back original values and print them
    printf("\n");
    ESP_LOGI(TAG, "Metadata:");
    for (int i = 0; i < metadata_size / sizeof(uint32_t); i++)
    {
        uint32_t tmp_value;
        memcpy(&tmp_value, &metadata_buffer[i * sizeof(uint32_t)], sizeof(uint32_t));
        printf("%lu, ", tmp_value);
    }

    printf("\n");
    ESP_LOGI(TAG, "index, re, im:");
    for (int i = 0; i < data_size / sizeof(float); i++)
    {
        float tmp_value = 0.0;
        memcpy(&tmp_value, &data_buffer[i * sizeof(float)], sizeof(float));
        printf("%.4f, ", tmp_value);
    }
}
