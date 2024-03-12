#include <arm_math.h>

#define FFT_BUFFER_SIZE 4096
#define FFT_OAE_IDX 122
#define NUM_NF_VALS 3*2 // Has to be a multiple of 2
#define NF_MAXIMUM 10000.0 // Placeholder Value. TODO: Update based on histograms of actual values and sensitivity.


void oae_algorithm(float32_t* accumulator, uint32_t* num_successful, const int32_t* sample_buffer, arm_rfft_fast_instance_f32* rfft, float32_t* window_lut) {
    static float32_t converted_buffer[FFT_BUFFER_SIZE];
    static float32_t windowed_buffer[FFT_BUFFER_SIZE];
    static float32_t fft_output[FFT_BUFFER_SIZE*2];
    static float32_t fft_output_abs[FFT_BUFFER_SIZE];
    float32_t average_noise_floor;

    ADCout_to_float32(sample_buffer, converted_buffer, FFT_BUFFER_SIZE);

    arm_mult_f32(window_lut, converted_buffer, windowed_buffer, FFT_BUFFER_SIZE);
    
    
    arm_rfft_fast_f32(rfft,converted_buffer, fft_output, 0);
    
    // Convert to fft magnitudes
    arm_cmplx_mag_f32(fft_output, fft_output_abs, FFT_BUFFER_SIZE);

    // There will be a more elegant way to execute this
    for (int i = 5; i<NUM_NF_VALS/2; i++) {
        average_noise_floor = fft_output_abs[FFT_OAE_IDX + i];
    }
    for (int i = -NUM_NF_VALS/2-5; i< (-NUM_NF_VALS/2); i++) {
        average_noise_floor = fft_output_abs[FFT_OAE_IDX + i];
    }

    if (average_noise_floor > NF_MAXIMUM) {
        exit();
    } else {
        *accumulator += fft_output_abs[FFT_OAE_IDX];
        *num_successful += 1;
    }
}

float32_t convert_ADCout_to_float32(int32_t adc_value) {
    // From: https://stackoverflow.com/questions/37468430/convert-24bit-twos-complement-to-float-32t
    // I would like someone to doublecheck this.
        return adc_value >> 8;
}

void ADCbuff_to_float32(int32_t* pSrc, float32_t* pDst, uint32_t numSamples) {
    for (int i = 0; i < numSamples; i++){
        pDst[i] = pSrc[i] >> 8;
    }


}
