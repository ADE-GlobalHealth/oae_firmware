/*
 * oae_algorithm.h
 *
 *  Created on: July 3, 2024
 *      Author: benjipugh
 */


#ifndef OAE_ALGORITHM_H_
#define OAE_ALGORITHM_H_

#include <arm_math.h>
#include <stdbool.h>


#define FFT_BUFFER_SIZE 2048
#define FFT_OAE_IDX 122 // Do change to proper index value of current oae signal. This is the array index we expect to see the OAE.
#define NUM_NF_VALS 3*2 // Has to be a multiple of 2
#define NF_MAXIMUM 10000.0 // Placeholder Value. TODO: Update based on histograms of actual values and sensitivity.
#define SCALE_DB 8.685889638 // Scale to the natural log of the magnitude required to obtain decibels.


// Currently unused datatype of different DMA transfer statuses
typedef enum {
    HALF_TRANSFER,
    FULL_TRANSFER
    } DMA_status;


// Struct to hold the ADC DMA pingpong buffers.
// Currently unused.
typedef struct {
    int32_t bufA[FFT_BUFFER_SIZE];
    int32_t bufB[FFT_BUFFER_SIZE];
    int32_t sample_buffer[FFT_BUFFER_SIZE];
    bool isA;
} pingpong_buffers_t;


// Struct for holding values needed for the oae algorithm
// Used for tracking the results of the algorithm as it runs many times.
typedef struct {
    float32_t window_lut[FFT_BUFFER_SIZE];
    arm_rfft_fast_instance_f32 fft;
    int32_t num_sub_nf_threshold;
    int32_t num_total_tests;
    float32_t oae_accumulator;
    float32_t nf_accumulator;
    int start_time;		// measure how long the algorithm takes (in msec)
    int time1;
    int time2;
    int time3;
    int time4;
    int time5;

} oae_data_t;


oae_data_t* setup_oae_data(void);

void oae_algorithm(oae_data_t *oae_data, int32_t* sample_buffer);

float32_t convert_ADCout_to_float32(int32_t adc_value);

void ADCbuff_to_float32(int32_t* pSrc, float32_t* pDst, uint32_t numSamples);

void switch_pingpong_buffer(pingpong_buffers_t *bufs, int32_t **DMA_addr);

void ADC_half_transfer(pingpong_buffers_t *bufs);

void ADC_full_transfer(pingpong_buffers_t *bufs, int32_t **DMA_addr);



#endif /* OAE_ALGORITHM_H_ */
