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
#define ADC_BYTES_PER_SAMPLE  	3		// 24 bits per sample * 64 samples/packet = 192 byte payload size
#define ADC_SAMPLES_PER_PACKET 	64
#define ADC_PACKETS_PER_BUFFER  (ADC_BUFFER_SIZE / ADC_SAMPLES_PER_PACKET)

#define OAE_SERIAL_PROTOCOL_VERSION "v1.2"

typedef struct {
    uint8_t header;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[SER_MAX_PAYLOAD_LEN];
    uint8_t checksum;
} SerialPacket_t;

// Host commands:
typedef enum {
    CMD_NOP 			= 0,		// No payload, no response expected
    CMD_PING 			= 1,		// Ping, no payload, RSP_PING response expected
    CMD_STATUS 			= 2,		// Request status from OAE, no payload, multiple RSP_TEXT responses expected
	CMD_ADC_BUF_REQ 	= 3,		// Payload: 1 byte: U8, Request ADC buffer # from OAE
	CMD_ADC_BUF_START 	= 4,		// Payload: 192 bytes: (64) 24 bit ADC samples (the first packet of 64 ADC samples), RSP_ACK or RSP_ERR response expected
	CMD_ADC_BUF 		= 5,		// Payload: 192 bytes: (64) 24 bit ADC samples (packet of 64 ADC samples), no response expected (there will be 62 of these packets in a 4096 sample buffer)
	CMD_ADC_BUF_END 	= 6,		// Payload: 192 bytes: (64) 24 bit ADC samples (the final packet of 64 ADC samples), RSP_ACK or RSP_ERR response expected
	CMD_I2C_RD		 	= 7,		// Payload: 2 bytes: U8 I2C device address, U8 I2C register address, RSP_U8 response expected (I2C read data)
	CMD_I2C_WR			= 8,		// Payload: 3 bytes: U8 I2C device address, U8 I2C register address, U8 I2C write data, RSP_ACK or RSP_ERR response expected
	CMD_START			= 9, 		// Payload: 1 byte: U8, which command to start, RSP_ACK or RSP_ERR response expected
	CMD_STOP			= 10,		// Payload: 1 byte: U8, which command to stop, RSP_ACK or RSP_ERR response expected
    CMD_OK 				= 11,		// No payload
} PacketCommand_t;

// OAE Embedded device responses:
typedef enum  {
    RSP_PING 			= 101,		// Ping response, no payload
    RSP_ACK 			= 102,		// No payload
    RSP_NAK 			= 103,		// No payload
    RSP_ERR 			= 104,		// Payload: Up to 250 bytes, text string
    RSP_TEXT 			= 105,		// Payload: Up to 250 bytes, text string
	RSP_ADC_BUF_START 	= 106,		// Payload: 192 bytes: (64) 24 bit ADC samples (the first packet of 64 ADC samples)
	RSP_ADC_BUF 		= 107,		// Payload: 192 bytes: (64) 24 bit ADC samples (packet of 64 ADC samples, there will be 62 of these packets in a 4096 sample buffer)
	RSP_ADC_BUF_END		= 108,		// Payload: 192 bytes: (64) 24 bit ADC samples (the final packet of 64 ADC samples)
    RSP_U8 				= 109,		// Payload: 1 byte:  U8
    RSP_U32 			= 110,		// Payload: 4 bytes: U32
	RSP_EVENT			= 111,		// Payload: 1 byte:  U8 (event number)
    RSP_INVALID 		= 112,		// No payload (Command from host was not recognized)
} PacketResponse_t;

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

void oae_serial_init(void);
bool oae_start_command(uint8_t command_num);
bool oae_stop_command(uint8_t command_num);
void oae_fill_adc_buffer(uint32_t *ADCBuf);
uint32_t oae_fill_adc_data_payload(uint32_t ADCBuf_starting_index, uint32_t *ADCBuf, uint32_t num_samples, uint8_t *payload_buf);
uint32_t oae_receive_adc_data_payload(uint32_t ADCBuf_starting_index, uint32_t *ADCBuf, uint32_t num_samples, uint8_t *payload_buf);
bool oae_serial_send(PacketCommand_t command, uint8_t payload_size, uint8_t *payload);
bool oae_serial_receive(uint8_t rx_char);
void oae_process_rx_packet(void);

int8_t RX_USB_CDC_Data(uint8_t* Buf, uint32_t *Len);

#define ERR_STR_INVALID_PAYLOAD_SIZE "ERR: payload_size invalid"

#ifdef __cplusplus
}
#endif


#endif /* INC_OAE_SERIAL_H_ */
