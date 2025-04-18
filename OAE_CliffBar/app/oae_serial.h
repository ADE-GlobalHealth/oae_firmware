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

#define SERIAL_BUFFER_MAX_SIZE		4096

#define BYTES_PER_U16  			2
#define BYTES_PER_U24  			3
#define BYTES_PER_S32  			4
#define BYTES_PER_F32  			4

#define U16_SAMPLES_PER_PACKET 	64			// 16 bits per sample * 64 samples/packet = 128 byte payload size
#define U24_SAMPLES_PER_PACKET 	64			// 24 bits per sample * 64 samples/packet = 192 byte payload size
#define F32_SAMPLES_PER_PACKET 	32			// 32 bits per sample * 32 samples/packet = 128 byte payload size
#define S32_SAMPLES_PER_PACKET 	32			// 32 bits per sample * 32 samples/packet = 128 byte payload size

#define U24_PACKETS_PER_BUFFER  (SERIAL_BUFFER_MAX_SIZE / U24_SAMPLES_PER_PACKET)
#define F32_PACKETS_PER_BUFFER  (SERIAL_BUFFER_MAX_SIZE / F32_SAMPLES_PER_PACKET)
#define S32_PACKETS_PER_BUFFER  (SERIAL_BUFFER_MAX_SIZE / S32_SAMPLES_PER_PACKET)

#define OAE_SERIAL_PROTOCOL_VERSION "v1.3"

typedef struct {
    uint8_t header;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[SER_MAX_PAYLOAD_LEN];
    uint8_t checksum;
} SerialPacket_t;

typedef enum {
	BUF_TYPE_NONE		= 0,
	BUF_TYPE_U24		= 1,		// 24 bit unsigned int
	BUF_TYPE_S32		= 2,		// 32 bit signed int
	BUF_TYPE_F32		= 3			// 32 bit float
} BufferDataType_t;

typedef struct {
	uint32_t	U32_Buffer[SERIAL_BUFFER_MAX_SIZE];
	int32_t		S32_Buffer[SERIAL_BUFFER_MAX_SIZE];
	float		F32_Buffer[SERIAL_BUFFER_MAX_SIZE];
	uint32_t	U32_BufSize;
	uint32_t	S32_BufSize;
	uint32_t	F32_BufSize;
} SerialBuffers_t;

// Host commands:
typedef enum {
    CMD_NOP 			= 0,		// No payload, no response expected
    CMD_PING 			= 1,		// Ping, no payload, RSP_PING response expected
    CMD_STATUS 			= 2,		// Request status from OAE, no payload, multiple RSP_TEXT responses expected
	CMD_BUF_REQ 		= 3,		// Payload: 1 byte: U8, Request buffer # from OAE
	CMD_BUF_START 		= 4,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data. First packet of the buffer. RSP_ACK or RSP_ERR response expected
	CMD_BUF 			= 5,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data. No response expected (there will be 62 of these packets in a 4096 sample buffer)
	CMD_BUF_END 		= 6,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data. Last packet of the buffer. RSP_ACK or RSP_ERR response expected
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
	RSP_BUF_START 		= 106,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data. First packet of the buffer
	RSP_BUF 			= 107,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data.
	RSP_BUF_END			= 108,		// Payload: byte 0: BufferDataType_t, bytes 1 to N: buffer data. Last packet of the buffer
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
void oae_fill_test_buffer(BufferDataType_t BufType);

uint32_t oae_build_buf_data_payload(BufferDataType_t BufType, uint32_t Buf_starting_index, uint32_t num_samples, uint8_t *payload_buf);
uint32_t oae_receive_buf_data_payload(uint8_t RxCommand, uint8_t payload_size, uint8_t *payload_buf);
bool oae_serial_send(PacketCommand_t command, uint8_t payload_size, uint8_t *payload);
bool oae_serial_send_error(char *error_str);
bool oae_serial_send_buffer(BufferDataType_t BufType);

bool oae_serial_receive(uint8_t rx_char);
void oae_process_rx_packet(void);

int8_t RX_USB_CDC_Data(uint8_t* Buf, uint32_t *Len);

// These functions are in app_main.c:
void w(uint16_t devaddr, uint16_t memaddr, uint8_t data2);
void r(uint16_t devaddr, uint16_t memaddr, uint8_t* data);

#define ERR_STR_INVALID_PAYLOAD_SIZE 	"ERR: payload_size invalid"
#define ERR_STR_INVALID_PARAMETER 		"ERR: calling parameter invalid"

extern SerialBuffers_t SerialBuf;

#ifdef __cplusplus
}
#endif


#endif /* INC_OAE_SERIAL_H_ */
