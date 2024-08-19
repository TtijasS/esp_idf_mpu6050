#include "my_fft.h"

esp_err_t ret;

__attribute__((aligned(16))) float fft_window_arr[N_SAMPLES];                  // Window coefficients
__attribute__((aligned(16))) float fft_complex_arr[N_SAMPLES * 2];             // Real and imaginary part of sampled data_sampled [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) indexed_float_type fft_magnitudes_arr[MAGNITUDES_SIZE]; // FFT output for plotting

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
        complex_arr[2 * i] = sampled_data_arr[i]; // * window_arr[i]; // Real part (@ i-th index)
        complex_arr[2 * i + 1] = 0;               // Imaginary part (@ i + 1 index)
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
        indexed_magnitudes_arr[i].value = sqrtf(((fft_complex_arr[i * 2] * fft_complex_arr[i * 2]) + (fft_complex_arr[i * 2 + 1] * fft_complex_arr[i * 2 + 1])) / N_SAMPLES);
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
 * @param percentile: 0 <= float < 100
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
 * @return 0 OK
 * @return -1 NULL params passed
 * @return -2 index out of bounds, i > magnitudes array size
 * @return -3 index out of bounds, i > indices buffer size
 * @return -4 index out of bounds, i > complex buffer size
 */
int fft_prepare_uart_data_buffer(uint8_t *indices_buffer, size_t indices_size, uint8_t *magnitudes_buffer, size_t magnitudes_size, uint8_t *complex_data_buffer, size_t complex_size, uint32_t n_fft_components, indexed_float_type *indexed_magnitudes, float *fft_components)
{
    const char *TAG = "PREP UART DATA";
    if (indices_buffer == NULL || magnitudes_buffer == NULL || complex_data_buffer == NULL || indexed_magnitudes == NULL || fft_components == NULL)
    {
        ESP_LOGE(TAG, "NULL params passed");
        return -1;
    }

    for (uint32_t i = 0; i < n_fft_components; i++)
    {
        if (i >= magnitudes_size)
        {
            ESP_LOGE(TAG, "i >= magnitudes size");
            return -2;
        }
        if (i >= indices_size)
        {
            ESP_LOGE(TAG, "i >= indices buffer size");
            return -3;
        }
        if ((i * 2 + 1) >= complex_size)
        {
            ESP_LOGE(TAG, "i >= complex buffer size");
            return -4;
        }

        // indexed magnitudes are sorted by the value
        // Indices are used for accessing the re and im components
        uint32_t _index = indexed_magnitudes[i].index;
        float _magnitude = indexed_magnitudes[i].value;
        // fft_complex_arr = [re[i], im[i+1], re[i*2],im[i*2+1],...]
        float _re = fft_complex_arr[_index * 2];
        float _im = fft_complex_arr[_index * 2 + 1];

        memcpy(&indices_buffer[i * sizeof(uint32_t)], &_index, sizeof(uint32_t));
        memcpy(&magnitudes_buffer[i * sizeof(float)], &_magnitude, sizeof(float));
        memcpy(&complex_data_buffer[(i * 2) * sizeof(float)], &_re, sizeof(float));
        memcpy(&complex_data_buffer[(i * 2 + 1) * sizeof(float)], &_im, sizeof(float));
        ESP_LOGI(TAG, "i %lu, ma %.4f, re %.4f, im %.4f", _index, _magnitude, _re, _im);
    }
    return 0;
}

/**
 * @brief Prepare metadata that will be sent over uart
 *
 * We need send the number of samples that were used in fft calculation and how many most significant components were sent over.
 *
 * @param metadata_buffer: pointer to buffer array that will be prepared
 * @param n_samples: number of samples used in fft calculation
 * @param n_components: number of most significant components that will get sent over uart
 * @return 0 OK
 */
int fft_prepare_metadata_buffer(uint8_t *metadata_buffer, uint32_t n_samples, uint32_t n_components)
{
    const char *TAG = "META PREP";
    if (metadata_buffer == NULL)
    {
        return -1;
    }
    ESP_LOGE(TAG, "n_samples: %lu, n_components: %lu", n_samples, n_components);
    memcpy(&metadata_buffer[0], &n_samples, sizeof(uint32_t));
    memcpy(&metadata_buffer[sizeof(uint32_t)], &n_components, sizeof(uint32_t));
    return 0;
}

/**
 * @brief Send the first n components (magnitude, re, im) of DFFT calculation
 *
 * @param n_samples: number of samples taken during the vibration sampling process
 * @param n_ms_elements: number of most significant components to send over uart
 * @return 0 OK
 * @return -1 failed to malloc metadata buffer
 * @return -2 failed to prepare metadata buffer
 * @return -3 failed to malloc indices buffer
 * @return -4 failed to malloc magnitudes buffer
 * @return -5 failed to malloc complex buffer
 * @return -6 failed to prepare indices or complex buffer
 * @return -7 failed to UART write buffers
 * @return -8 failed to debug UART buffers
 */
int fft_send_most_significant_components_over_uart(uint32_t n_samples, uint32_t n_ms_elements)
{
    uint8_t *metadata_buffer = NULL;
    uint8_t *indices_buffer = NULL;
    uint8_t *magnitudes_buffer = NULL;
    uint8_t *complex_data_buffer = NULL;

    int error_code = 0;
    // Metadata buffer
    size_t metadata_size = 2 * sizeof(uint32_t);
    metadata_buffer = (uint8_t *)malloc(metadata_size);
    if (metadata_buffer == NULL)
    {
        ESP_LOGI(TAG, "-1");
        error_code = -1;
        goto cleanup;
    }

    if (fft_prepare_metadata_buffer(metadata_buffer, n_samples, n_ms_elements) != 0)
    {
        ESP_LOGI(TAG, "-2");
        error_code = -2;
        goto cleanup;
    }

    // Most significant indices buffer
    size_t indices_size = n_ms_elements * sizeof(uint32_t);
    indices_buffer = (uint8_t *)malloc(indices_size);
    if (indices_buffer == NULL)
    {
        ESP_LOGI(TAG, "-3");
        error_code = -3;
        goto cleanup;
    }

    // Magnitudes
    size_t magnitudes_size = n_ms_elements * sizeof(float);
    magnitudes_buffer = (uint8_t *)malloc(magnitudes_size);
    if (magnitudes_buffer == NULL)
    {
        ESP_LOGI(TAG, "-4");
        error_code = -4;
        goto cleanup;
    }

    // Complex data buffer
    size_t complex_size = 2 * n_ms_elements * sizeof(float);
    complex_data_buffer = (uint8_t *)malloc(complex_size);
    if (complex_data_buffer == NULL)
    {
        ESP_LOGI(TAG, "-5");
        error_code = -5;
        goto cleanup;
    }

    if (fft_prepare_uart_data_buffer(indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, complex_data_buffer, complex_size, n_ms_elements, fft_magnitudes_arr, fft_complex_arr) != 0)
    {

        ESP_LOGI(TAG, "-6");
        error_code = -6;
        goto cleanup;
    }

    // Send data
    int err_c;
    if ((err_c = uart_send_fft_components(metadata_buffer, metadata_size, indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, complex_data_buffer, complex_size)) != 0)
    {
        ESP_LOGI(TAG, "-7; %d", err_c);
        error_code = -7;
        goto cleanup;
    }

    // if (fft_debug_uart_buffers(metadata_buffer, metadata_size, indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, complex_data_buffer, complex_size) != 0)
    // {
    //     ESP_LOGI(TAG, "-8");
    //     error_code = -8;
    //     goto cleanup;
    // }
    cleanup:
    if (metadata_buffer != NULL)
        free(metadata_buffer);
    if (indices_buffer != NULL)
        free(indices_buffer);
    if (magnitudes_buffer != NULL)
        free(magnitudes_buffer);
    if (complex_data_buffer != NULL)
        free(complex_data_buffer);
    return error_code;
}

/**
 * @brief Printing the metadata and data buffer for debugging purposes
 *
 * @param metadata_buffer: buffer with metadata uint8_t values
 * @param metadata_size: size of the metadata buffer
 * @param data_buffer: buffer with data float values
 * @param data_size: size of the data buffer
 * @return 0 OK
 * @return -1 NULL params passed
 */
int fft_debug_uart_buffers(uint8_t *metadata_buffer, size_t metadata_size, uint8_t *indices_buffer, size_t indices_size, uint8_t *magnitudes_buffer, size_t magnitudes_size, uint8_t *complex_data_buffer, size_t complex_size)
{
    if (metadata_buffer == NULL || indices_buffer == NULL || magnitudes_buffer == NULL || complex_data_buffer == NULL)
    {
        return -1;
    }
    uint32_t tmp_u32t;
    float tmp_f;

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
    ESP_LOGI(TAG, "indices:");
    for (int i = 0; i < (indices_size / sizeof(uint32_t)); i++)
    {
        memcpy(&tmp_u32t, &indices_buffer[i * sizeof(uint32_t)], sizeof(uint32_t));
        printf("%lu, ", tmp_u32t);
    }

    printf("\n");
    ESP_LOGI(TAG, "Magnitudes:");
    for (int i = 0; i < (magnitudes_size / sizeof(float)); i++)
    {
        memcpy(&tmp_f, &magnitudes_buffer[i * sizeof(float)], sizeof(float));
        printf("%.4f, ", tmp_f);
    }

    printf("\n");
    ESP_LOGI(TAG, "im, re, im, re, ...:");
    for (int i = 0; i < complex_size / sizeof(float); i++)
    {
        memcpy(&tmp_f, &complex_data_buffer[i * sizeof(float)], sizeof(float));
        printf("%.4f, ", tmp_f);
    }
    return 0;
}
