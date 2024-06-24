#include "my_fft.h"

esp_err_t ret;

__attribute__((aligned(16))) float window[N_SAMPLES];         // Window coefficients
__attribute__((aligned(16))) float data_complex[N_SAMPLES*2]; // Real and imaginary part of sampled data_sampled [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
__attribute__((aligned(16))) float fft_output[N_SAMPLES];     // FFT output for magnitudes

void fft_init()
{
    ret = dsps_fft2r_init_fc32(NULL, N_SAMPLES);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error in FFT init: %s", esp_err_to_name(ret));
    }
}

void fft_apply_window_on_fft_complex(float *data_sampled, float *window, float *data_complex)
{
    // Prepare window coefficients
    dsps_wind_hann_f32(window, N_SAMPLES);

    // Prepare data_complex array
    for (int i = 0; i < N_SAMPLES; i++)
    {
        // [r0, im0, r1, im1, r2, im2, ...r_n_samples, im_n_samples]
        data_complex[2 * i] = data_sampled[i] * window[i]; // Real part (@ i-th index)
        data_complex[2 * i + 1] = 0;               // Imaginary part (@ i + 1 index)
    }
}

void fft_calculate(float *data_complex, int length)
{
    // Perform FFT
    dsps_fft2r_fc32(data_complex, length);
    // Bit reverse
    dsps_bit_rev_fc32(data_complex, length);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(data_complex, length);

    for (int i = 0; i < length / 2; i++) {
        fft_output[i] = 10 * log10f((data_complex[i * 2 + 0] * data_complex[i * 2 + 0] + data_complex[i * 2 + 1] * data_complex[i * 2 + 1]) / N_SAMPLES);
    }
}

void fft_run_with_hann(float *data_sampled, int length)
{
    // Apply window on data_sampled
    fft_apply_window_on_fft_complex(data_sampled, window, data_complex);

    // Perform FFT
    fft_calculate(data_complex, N_SAMPLES);

    // Display FFT output
    dsps_view(fft_output, length / 2, 64, 10, -120, 40, '|');
}
