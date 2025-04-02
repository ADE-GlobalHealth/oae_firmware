/*
 * oae_serial.h
 *
 *  Created on: Mar 24, 2025
 *  	Olin College of Engineering
 *  	Author: Mike Deeds
 *
 *  Overview:
 *    This module implements a packet protocol that can run over a serial UART or USB CDC.
 *    It is designed primarily as a command/response system initiated by the host, though
 *    debug / status / event messages may be sent any time by the embedded device.
 *
 *  OAE serial packet format:
 *    Byte 0: Header: 0x7E
 *    Byte 1: Command
 *    Byte 2: Payload_size
 *    Bytes 3 to Payload_size + 3
 *    Byte N: checksum (sum of all bytes, truncated to 8 bits)
 *
 */

#ifndef INC_OAE_SERIAL_H_
#define INC_OAE_SERIAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#define SER_MAX_PACKET_LEN 254
#define SER_MAX_PAYLOAD_LEN 250

#define SER_PACKET_HEADER 0x7E

#define ADC_BUFFER_SIZE			4096
#define ADC_SAMPLES_PER_PACKET 	64		// 24 bits per sample * 64 samples/packet = 192 byte payload size
#define ADC_PACKETS_PER_BUFFER  (ADC_BUFFER_SIZE / ADC_SAMPLES_PER_PACKET)

#define OAE_SERIAL_PROTOCOL_VERSION "v1.1"

typedef struct {
    uint8_t header;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[SER_MAX_PAYLOAD_LEN];
    uint8_t checksum;
} SerialPacket_t;

typedef struct {
    int rx_packet_valid_count;
    int rx_packet_err_count;
    int rx_packet_err;
    int tx_packet_count;
    int tx_packet_err_count;
    int rx_time_usec;
    int rsp_ack_time;
    int command_start_time;
    int command_turnaround_time;
} SerialStats_t;

typedef enum {
    CMD_NOP 		= 0,
    CMD_PING 		= 1,
    CMD_STATUS 		= 2,
	CMD_ADC_BUF_REQ 	= 3,
	CMD_ADC_BUF_START 	= 4,
	CMD_ADC_BUF 		= 5,
	CMD_ADC_BUF_END 	= 6,
    CMD_OK = 7,
} PacketCommand_t;

typedef enum  {
    RSP_PING 	= 101,
    RSP_ACK 	= 102,
    RSP_NAK 	= 103,
    RSP_ERR 	= 104,
    RSP_TEXT 	= 105,
	RSP_ADC_BUF_START 	= 106,
	RSP_ADC_BUF 		= 107,
	RSP_ADC_BUF_END		= 108,
    RSP_U8 				= 109,
    RSP_U32 			= 110,
    RSP_INVALID 		= 111,
} PacketResponse_t;

void oae_serial_init(void);
void oae_fill_adc_buffer(uint32_t *ADCBuf);
uint32_t oae_fill_adc_data_payload(uint32_t ADCBuf_starting_index, uint32_t *ADCBuf, uint32_t num_samples, uint8_t *payload_buf);
bool oae_serial_send(PacketCommand_t command, uint8_t payload_size, uint8_t *payload);
bool oae_serial_receive(uint8_t rx_char);
void oae_process_rx_packet(void);

int8_t RX_USB_CDC_Data(uint8_t* Buf, uint32_t *Len);

#ifdef __cplusplus
}
#endif


#endif /* INC_OAE_SERIAL_H_ */
