#include "my_fft.h"

void fft_init()
{
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in FFT init: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "FFT init complete");
}

/**
 * @brief Prepare window before constructing complex array
 *
 * @param window_arr pointer to the window array
 *
 */
void fft_prepare_window(float *window_arr)
{
    dsps_wind_hann_f32(window_arr, N_SAMPLES);
}

/**
 * @brief Merge sampled datan and window coefficients into complex array
 *
 * @param sampled_data_arr: array with sensor data
 * @param window_arr: array with window coefficients
 * @param complex_arr: array that stores real and imaginary parts of the signal
 * @param arr_len: length of the sampled_data_arr
 */
void fft_prepare_complex_arr(float *sampled_data_arr, float *complex_arr, uint32_t arr_len)
{
    if (sampled_data_arr == NULL || complex_arr == NULL)
    {
        ESP_LOGE(TAG, "Null pointer error in fft_prepare_complex_arr");
        return;
    }
    // Prepare fft_complex_arr array
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
 * and store results into fft_complex_arr array of length N_SAMPLES*2
 *
 * @param complex_arr fft complex components array
 * @param n_samples number of data samples (should be half the size of the complex arr)
 */
void fft_calculate_re_im(float *complex_arr, uint32_t n_samples)
{
    ESP_ERROR_CHECK(dsps_fft2r_fc32(complex_arr, n_samples));

    ESP_ERROR_CHECK(dsps_bit_rev_fc32(complex_arr, n_samples));

    ESP_ERROR_CHECK(dsps_cplx2reC_fc32(complex_arr, n_samples));
}

/**
 * @brief Calculate magnitudes from FFT results
 *
 * First run fft_calculate_im_re, then convert components to magnitudes (sqrtf scale)
 *
 * @param magnitudes_indexed array of structs with index and magnitude value
 * @param fft_complex_arr array of fft complex components
 * @param magnitudes_size size of magnitudes array (should be hald of the number of data samples)
 */
void fft_calculate_magnitudes(indexed_float_type *indexed_magnitudes_arr, float *fft_complex_arr, uint32_t magnitudes_size)
{
    if (indexed_magnitudes_arr == NULL)
    {
        ESP_LOGE(TAG, "Null pointer error in fft_calculate_magnitudes");
        return;
    }

    if (magnitudes_size == 0)
    {
        ESP_LOGE(TAG, "Array length is zero in fft_calculate_magnitudes");
        return;
    }

    for (int i = 0; i < magnitudes_size; i++)
    {
        indexed_magnitudes_arr[i].index = i;
        indexed_magnitudes_arr[i].value = sqrtf(((fft_complex_arr[i * 2] * fft_complex_arr[i * 2]) + (fft_complex_arr[i * 2 + 1] * fft_complex_arr[i * 2 + 1])) / N_SAMPLES);
    }
}

/**
 * @brief Sort the indexed_mangitudes of indexed_float_type in descending order.
 *
 * @param indexed_mangitudes array that will get sorted
 * @param magnitudes_size size of the array that will get sorted
 */
void fft_sort_magnitudes(indexed_float_type *indexed_mangitudes, uint32_t magnitudes_size)
{
    qsort(indexed_mangitudes, magnitudes_size, sizeof(indexed_float_type), compare_indexed_float_type_descending);
}

/**
 * @brief Debug plot the magnitudes of the magnitudes_indexed
 *
 * First calculate real and imaginary coponents, then run fft_calculate_magnitudes.
 * After that you can plot magnitudes as a simple asci char plot.
 *
 * @param indexed_magnitudes indexed magnitudes array
 * @param arr_len: length of indexed magnitudes array
 * @param min: plot scale min
 * @param max: plot scale max
 */

void fft_plot_magnitudes(indexed_float_type *indexed_magnitudes, uint32_t arr_len, int min, int max)
{
    // construct tmp magnitudes float array
    float *magnitudes = (float *)malloc(arr_len * sizeof(float));
    for (uint32_t i = 0; i < arr_len; i++)
    {
        magnitudes[i] = indexed_magnitudes[i].value;
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
 * @brief Prepare metadata that will be sent over uart
 *
 * We need send the number of samples that were used in fft calculation and how many most significant components were sent over.
 *
 * @param metadata_buffer: pointer to buffer array that will be prepared
 * @param n_samples: number of samples used in fft calculation
 * @param n_components: number of most significant components that will get sent over uart
 * @return 0 OK
 * @return -1 null pointer passed
 * @return -2 metadata buffer size too small
 */
int fft_prepare_metadata_buffer(uint8_t *metadata_buffer, size_t metadata_size, uint32_t n_samples, uint32_t n_components)
{
    const char *TAG = "META PREP";
    esp_log_level_set(TAG, ESP_LOG_INFO);

    if (metadata_buffer == NULL)
    {
        return -1;
    }
    if (metadata_size < (2 * sizeof(uint32_t)))
    {
        return -2;
    }
    memcpy(&metadata_buffer[0], &n_samples, sizeof(uint32_t));
    memcpy(&metadata_buffer[sizeof(uint32_t)], &n_components, sizeof(uint32_t));
    return 0;
}

/**
 * @brief Prepare most significant magnitudes and their indices to send over uart
 *
 * @param indices_buffer uart buffer for indices
 * @param indices_size size of indices buffer
 * @param magnitudes_buffer uart buffer for magnitudes
 * @param magnitudes_size size of magnitudes buffer
 * @param n_ms_components number of fft components
 * @return 0 OK
 * @return -1 NULL pointers passed
 * @return -2 indices buffer size too small
 * @return -3 magnitudes buffer size too small
 * @return -4 indices_size != magnitudes_size
 */
int fft_prepare_magnitudes_buffer(uint8_t *indices_buffer, size_t indices_size, uint8_t *magnitudes_buffer, size_t magnitudes_size, indexed_float_type *indexed_magnitudes, uint32_t n_ms_components)
{
    if (indices_buffer == NULL || magnitudes_buffer == NULL || indexed_magnitudes == NULL)
    {
        return -1;
    }
    if (indices_size < (4 * n_ms_components))
    {
        return -2;
    }
    if (magnitudes_size < (4 * n_ms_components))
    {
        return -3;
    }
    if (indices_size != magnitudes_size)
    {
        return -4;
    }

    for (uint32_t i = 0; i < n_ms_components; i++)
    {
        memcpy(&indices_buffer[i * sizeof(uint32_t)], &indexed_magnitudes[i].index, sizeof(uint32_t));
        memcpy(&magnitudes_buffer[i * sizeof(float)], &indexed_magnitudes[i].value, sizeof(float));
    }
    return 0;
}

/**
 * @brief Prepare most significant complex components to send over uart
 *
 * @param complex_buffer uart complex data buffer
 * @param complex_size size of uart complex data buffer
 * @param n_ms_components number of most significant fft components to send over uart
 * @param fft_complex_arr fft complex components array (re, im elements)
 * @param indexed_mangitudes indexed magnitudes array
 * @return 0 OK
 * @return -1 NULL pointers passed
 * @return -2 complex buffer size too small
 * @return -3 magnitude index out of range
 */
int fft_prepare_complex_buffer(uint8_t *complex_buffer, size_t complex_size, uint32_t n_ms_components, indexed_float_type *indexed_mangitudes, float *fft_complex_arr)
{
    if (complex_buffer == NULL || fft_complex_arr == NULL || indexed_mangitudes == NULL)
    {
        return -1;
    }
    if (complex_size < (n_ms_components * sizeof(float) * 2))
    {
        return -2;
    }
    for (uint32_t i = 0; i < n_ms_components; i++)
    {
        uint32_t mag_index = indexed_mangitudes[i].index * 2;
        if (mag_index >= N_SAMPLES * 2)
        {
            return -3;
        }
        // memcpy(&complex_buffer[i * 2 * sizeof(float)], &fft_complex_arr[_index], 2 * sizeof(float));
        // re part is at index 2i
        memcpy(&complex_buffer[(2 * i) * sizeof(float)], &fft_complex_arr[mag_index], sizeof(float));
        // im part is at index 2i+1
        memcpy(&complex_buffer[((2 * i) + 1) * sizeof(float)], &fft_complex_arr[mag_index + 1], sizeof(float));
    }
    return 0;
}

/**
 * @brief Send the first n components (magnitude, re, im) of DFFT calculation
 *
 * @param fft_complex_arr fft complex components array (re, im elements)
 * @param indexed_magnitudes indexed magnitudes array
 * @param n_samples number of data samples
 * @param n_ms_elements number of most significant elements
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
int fft_send_ms_components_over_uart(float *fft_complex_arr, indexed_float_type *indexed_magnitudes, uint32_t n_samples, uint32_t n_ms_elements)
{
    int error_code = 0;
    uint8_t *metadata_buffer = NULL;
    uint8_t *indices_buffer = NULL;
    uint8_t *magnitudes_buffer = NULL;
    uint8_t *complex_buffer = NULL;

    // Metadata buffer
    size_t metadata_size = 2 * sizeof(uint32_t);
    metadata_buffer = (uint8_t *)malloc(metadata_size);
    if (metadata_buffer == NULL)
    {
        error_code = -1;
        goto memcleanup;
    }

    // Most significant indices buffer
    size_t indices_size = n_ms_elements * sizeof(uint32_t);
    indices_buffer = (uint8_t *)malloc(indices_size);
    if (indices_buffer == NULL)
    {
        error_code = -2;
        goto memcleanup;
    }

    // Magnitudes
    size_t magnitudes_size = n_ms_elements * sizeof(float);
    magnitudes_buffer = (uint8_t *)malloc(magnitudes_size);
    if (magnitudes_buffer == NULL)
    {
        error_code = -3;
        goto memcleanup;
    }

    // Complex data buffer
    size_t complex_size = 2 * n_ms_elements * sizeof(float);
    complex_buffer = (uint8_t *)malloc(complex_size);
    if (complex_buffer == NULL)
    {
        error_code = -4;
        goto memcleanup;
    }

    if ((error_code = fft_prepare_metadata_buffer(metadata_buffer, metadata_size, n_samples, n_ms_elements)) != 0)
    {
        ESP_LOGI(TAG, "error -5, sub error %d", error_code);
        error_code = -5;
        goto memcleanup;
    }

    if ((error_code = fft_prepare_magnitudes_buffer(indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, indexed_magnitudes, n_ms_elements)) != 0)
    {
        ESP_LOGI(TAG, "error -6, sub error %d", error_code);
        error_code = -6;
        goto memcleanup;
        return -6;
    }

    if ((error_code = fft_prepare_complex_buffer(complex_buffer, complex_size, n_ms_elements, indexed_magnitudes, fft_complex_arr)) != 0)
    {
        ESP_LOGI(TAG, "error -7, sub error %d", error_code);
        error_code = -7;
        goto memcleanup;
    }

    if ((error_code = uart_send_fft_components(metadata_buffer, metadata_size, indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, complex_buffer, complex_size)) != 0)
    {
        ESP_LOGI(TAG, "error -8, sub error %d", error_code);
        error_code = -8;
        goto memcleanup;
    }

    // if ((error_code = fft_debug_uart_buffers(metadata_buffer, metadata_size, indices_buffer, indices_size, magnitudes_buffer, magnitudes_size, complex_buffer, complex_size)) != 0)
    // {
    //     ESP_LOGE(TAG, "error -9, sub error %d", error_code);
    //     error_code = -9;
    //     goto memcleanup;
    // }
memcleanup:
    if (metadata_buffer != NULL)
        free(metadata_buffer);
    if (indices_buffer != NULL)
        free(indices_buffer);
    if (magnitudes_buffer != NULL)
        free(magnitudes_buffer);
    if (complex_buffer != NULL)
        free(complex_buffer);
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
