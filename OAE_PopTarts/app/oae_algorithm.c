/*
 * oae_algorithm.c
 *
 *  Created on: February, 2024
 *      Author: benjipugh
 */


#include "oae_algorithm.h"
#include <stdlib.h>
#include <arm_math.h>
#include <stdbool.h>

oae_data_t* setup_oae_data(){
    oae_data_t* oae_data = (oae_data_t*) malloc(sizeof(oae_data_t));

    #if FFT_BUFFER_SIZE != 2048
        #error Currently is only able to deal with 2048 sized fft buffer
    #endif
    arm_rfft_fast_init_f32(&(oae_data->fft), 2048);
    oae_data->num_sub_nf_threshold = 0;
    oae_data->oae_accumulator = 0;
    arm_hamming_f32(oae_data->window_lut, FFT_BUFFER_SIZE);
    return oae_data;
}

void oae_algorithm(oae_data_t *oae_data, int32_t* sample_buffer) {
    static float32_t windowed_buffer[FFT_BUFFER_SIZE];
    static float32_t fft_output[FFT_BUFFER_SIZE];
    float32_t * converted_buffer = fft_output;
    
    float32_t * fft_output_abs = windowed_buffer;
    float32_t * fft_output_dB = fft_output;
    float32_t average_noise_floor;

    ADCbuff_to_float32(sample_buffer, converted_buffer, FFT_BUFFER_SIZE);

    arm_mult_f32(oae_data->window_lut, converted_buffer, windowed_buffer, FFT_BUFFER_SIZE);
    
    arm_rfft_fast_f32(&(oae_data->fft), converted_buffer, fft_output, 0);
    
    // Convert to fft magnitudes
    arm_cmplx_mag_f32(fft_output+1, fft_output_abs, FFT_BUFFER_SIZE-2); // Since the FFT outputs 
    // Convert to dB. This is sort of a worst case scenario since this is performing log on the entire set of samples
    arm_vlog_f32(fft_output_abs, fft_output_dB, FFT_BUFFER_SIZE/2);
    arm_scale_f32(fft_output_dB, SCALE_DB, fft_output_dB, FFT_BUFFER_SIZE/2);

    // There will be a more elegant way to execute this
    for (int i = 5; i<NUM_NF_VALS/2; i++) {
        average_noise_floor = fft_output_dB[FFT_OAE_IDX + i];
    }
    for (int i = -NUM_NF_VALS/2-5; i< (-NUM_NF_VALS/2); i++) {
        average_noise_floor = fft_output_dB[FFT_OAE_IDX + i];
    }

    if (average_noise_floor > NF_MAXIMUM) {
        return;
    } else {
        oae_data->oae_accumulator += fft_output_abs[FFT_OAE_IDX];
        oae_data->nf_accumulator += average_noise_floor;
        oae_data->num_sub_nf_threshold += 1;
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

//void interpolate_error_samples()

void switch_pingpong_buffer(pingpong_buffers_t * bufs, int32_t ** DMA_addr){
    // Switch the pingpong buffer
    if(bufs->isA){
        *DMA_addr = bufs->bufB;
    } else {
        *DMA_addr = bufs->bufA;
    }
    bufs->isA = !bufs->isA;
}


void ADC_half_transfer(pingpong_buffers_t * bufs){
    // Expects that bufA and bufB are the length of a full fft buffer (FFT_BUFFER_SIZE)
    // Sample buffer is the output.
    if (bufs->isA)   {
        memcpy(bufs->sample_buffer, bufs->bufB+FFT_BUFFER_SIZE/2, FFT_BUFFER_SIZE/2);
        memcpy(bufs->sample_buffer+FFT_BUFFER_SIZE/2, bufs->bufA, FFT_BUFFER_SIZE/2);
    } else {
        memcpy(bufs->sample_buffer, bufs->bufA+FFT_BUFFER_SIZE/2, FFT_BUFFER_SIZE/2);
        memcpy(bufs->sample_buffer+FFT_BUFFER_SIZE/2, bufs->bufB, FFT_BUFFER_SIZE/2);
    }
    
}


void ADC_full_transfer(pingpong_buffers_t * bufs, int32_t ** DMA_addr){
    // Gets called when
    // Expects that bufA and bufB are the length of a full fft buffer (FFT_BUFFER_SIZE)
    // Sample buffer is the output.
    if (bufs->isA) {
        memcpy(bufs->sample_buffer,  bufs->bufA, FFT_BUFFER_SIZE/2);
    }
    else {
        memcpy(bufs->sample_buffer,  bufs->bufB, FFT_BUFFER_SIZE/2);
    }
    switch_pingpong_buffer(bufs, DMA_addr);
}

