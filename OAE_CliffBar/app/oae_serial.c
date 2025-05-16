/*
 * oae_serial.c
 *
 *  Created on: Mar 24, 2025
 *  	Olin College of Engineering
 *      Author: Mike Deeds
 *
 *
 */

#include "stm32l4xx_hal.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "oae_serial.h"
#include "oae_algorithm.h"

extern I2C_HandleTypeDef hi2c3;

SerialStats_t SerStats;
SerialPacket_t RxPacket;

// All buffer transfers between the host and OAE use data in these buffers:
SerialBuffers_t SerialBuf;

// This struct manages the OAE algorithm data and debug information:
oae_data_t* oae_data = 0;


void oae_serial_init(void)
{
	SerStats.rx_packet_valid_count = 0;
	SerStats.rx_packet_err_count = 0;
	SerStats.rx_packet_err = 0;
	SerStats.tx_packet_count = 0;
	SerStats.tx_packet_err_count = 0;
	SerStats.rx_time_usec = 0;
	SerStats.rsp_ack_time = 0;
	SerStats.command_start_time = 0;
	SerStats.command_turnaround_time = 0;
	SerialBuf.U32_BufSize = SERIAL_BUFFER_MAX_SIZE;
	SerialBuf.F32_BufSize = SERIAL_BUFFER_MAX_SIZE;
	SerialBuf.S32_BufSize = SERIAL_BUFFER_MAX_SIZE;

	// Fill the buffers with test data:
	oae_fill_test_buffer(BUF_TYPE_U24);
	oae_fill_test_buffer(BUF_TYPE_F32);
	oae_fill_test_buffer(BUF_TYPE_S32);
}

bool oae_algorithm_test(void)
{
	uint8_t	TxBuffer[SER_MAX_PAYLOAD_LEN];
	uint8_t buf_len;

	int t0, t1, t2;
	t0 = HAL_GetTick();

	if (oae_data == 0) {
		// allocate the oae data struct the first time this is called:
		oae_data =  setup_oae_data();
	}
	t1 = HAL_GetTick();
	oae_algorithm(oae_data, SerialBuf.S32_Buffer);
	t2 = HAL_GetTick();
	buf_len = sprintf((char *)TxBuffer, "oae_algorithm_test: %d %d", t1-t0, t2-t1);
    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);

	buf_len = sprintf((char *)TxBuffer, "\t times: %d %d %d %d %d", oae_data->time1 - oae_data->start_time, oae_data->time2 - oae_data->start_time, oae_data->time3 - oae_data->start_time, oae_data->time4 - oae_data->start_time, oae_data->time5 - oae_data->start_time);
    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);

	return true;
}

bool oae_start_command(ActionCommand_t action)
{
	// Start the selected action:
	switch(action)
	{
		case ACTION_OAE_TEST:
			oae_algorithm_test();
			break;
		case ACTION_ADC_SAMPLES:
			;
			break;
		default:
			return false;	// invalid action command
			break;
	}

	return true;
}

bool oae_stop_command(ActionCommand_t action)
{
	// add stop commands here
	return true;
}

// Fill a buffer with test data:
//   BufType determines which buffer is filled and with what kind of data
void oae_fill_test_buffer(BufferDataType_t BufType)
{
	int SampleCount;
	int PacketCount;
	int i = 0;

    union {
        float f;
        uint32_t u;
        int32_t s;
    } sample;

	if (BufType == BUF_TYPE_U24) {
		SerialBuf.U32_BufSize = SERIAL_BUFFER_MAX_SIZE;
		for (PacketCount = 0; PacketCount < U24_PACKETS_PER_BUFFER; PacketCount++) {
			for (SampleCount = 0; SampleCount < U24_SAMPLES_PER_PACKET; SampleCount++) {

				// Create 24 bit test data with the packet number in the upper 8 bits:
				SerialBuf.U32_Buffer[i] = (PacketCount << 16) | (i & 0xFFFF);
				i++;
			}
		}
	} else if (BufType == BUF_TYPE_F32) {
		SerialBuf.F32_BufSize = SERIAL_BUFFER_MAX_SIZE;
		for (PacketCount = 0; PacketCount < F32_PACKETS_PER_BUFFER; PacketCount++) {
			for (SampleCount = 0; SampleCount < F32_SAMPLES_PER_PACKET; SampleCount++) {

				// test data:
				sample.f = 10.0 * (float) i;
				SerialBuf.F32_Buffer[i] = (uint32_t) sample.f;
				i++;
			}
		}
	}
	else if (BufType == BUF_TYPE_S32) {
		SerialBuf.S32_BufSize = SERIAL_BUFFER_MAX_SIZE;
		for (PacketCount = 0; PacketCount < S32_PACKETS_PER_BUFFER; PacketCount++) {
			for (SampleCount = 0; SampleCount < S32_SAMPLES_PER_PACKET; SampleCount++) {

				// test data:
				sample.s = -10 * i;
				SerialBuf.S32_Buffer[i] = (uint32_t) sample.s;
				i++;
			}
		}
	}

}

// Fills a packet payload buffer with data.
//   BufType determines which buffer is filled and with what kind of data
// The function returns the number of bytes in the payload
uint32_t oae_build_buf_data_payload(BufferDataType_t BufType, uint32_t Buf_starting_index, uint32_t num_samples, uint8_t *payload_buf)
{
    union {
        float f;
        uint32_t u;
        int32_t s;
    } sample;

    int payload_offset = 1;	// payload byte 0 holds the data type
    int payload_size = 0;

	if (BufType == BUF_TYPE_U24) {
		payload_size = num_samples * BYTES_PER_U24 + 1;
		if (payload_size > SER_MAX_PAYLOAD_LEN) return 0;	// Size error!

		payload_buf[0] = (uint8_t) BUF_TYPE_U24;
		for (int SampleCount=0; SampleCount<num_samples; SampleCount++) {
			sample.u = SerialBuf.U32_Buffer[Buf_starting_index + SampleCount];
			payload_buf[payload_offset + SampleCount * BYTES_PER_U24 + 0] = (sample.u >> 16) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_U24 + 1] = (sample.u >> 8) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_U24 + 2] = (sample.u) & 0xFF;
		}
	}
	else if (BufType == BUF_TYPE_F32) {
		payload_size = num_samples * BYTES_PER_F32 + 1;
		if (payload_size > SER_MAX_PAYLOAD_LEN) return 0;	// Size error!

		payload_buf[0] = (uint8_t) BUF_TYPE_F32;
		for (int SampleCount=0; SampleCount<num_samples; SampleCount++) {
			sample.f = SerialBuf.F32_Buffer[Buf_starting_index + SampleCount];
			payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 0] = (sample.u >> 24) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 1] = (sample.u >> 16) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 2] = (sample.u >> 8) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 3] = (sample.u) & 0xFF;
		}
	}
	else if (BufType == BUF_TYPE_S32) {
		payload_size = num_samples * BYTES_PER_S32 + 1;
		if (payload_size > SER_MAX_PAYLOAD_LEN) return 0;	// Size error!

		payload_buf[0] = (uint8_t) BUF_TYPE_S32;
		for (int SampleCount=0; SampleCount<num_samples; SampleCount++) {
			sample.s = SerialBuf.S32_Buffer[Buf_starting_index + SampleCount];
			payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 0] = (sample.u >> 24) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 1] = (sample.u >> 16) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 2] = (sample.u >> 8) & 0xFF;
			payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 3] = (sample.u) & 0xFF;
		}
	}
	else return 0;

	return payload_size;
}

// Copy one packet of received  data into the specified Buf
// payload index 0 contains the buffer data type
//   This determines which buffer is filled and with what kind of data
uint32_t oae_receive_buf_data_payload(uint8_t RxCommand, uint8_t payload_size, uint8_t *payload_buf)
{
	static int	buf_sample_index = 0;

    union {
        float f;
        uint32_t u;
        int32_t s;
    } sample;

	uint32_t samples_received;
    int payload_offset = 1;	// payload byte 0 holds the data type, which indicates where to place the data and in which format

    BufferDataType_t BufType;
    BufType = payload_buf[0];


	if (BufType == BUF_TYPE_U24) {
	    if (RxCommand == CMD_BUF_START) {
	    	buf_sample_index = 0;
	    	SerialBuf.U32_BufSize = 0;
	    }
		samples_received = (payload_size-1)/BYTES_PER_U24;
		if ((buf_sample_index + samples_received) > SERIAL_BUFFER_MAX_SIZE) return 0;	// Size error!
		else if (samples_received < 1 || samples_received > U24_SAMPLES_PER_PACKET) return 0;

		for (int SampleCount=0; SampleCount<samples_received; SampleCount++) {
			sample.u = (payload_buf[SampleCount * BYTES_PER_U24 + 0] << 16);
			sample.u |= (payload_buf[SampleCount * BYTES_PER_U24 + 1] << 8);
			sample.u |= payload_buf[SampleCount * BYTES_PER_U24 + 2];
			SerialBuf.U32_Buffer[buf_sample_index + SampleCount] = sample.u;
		}
	    if (RxCommand == CMD_BUF_END) {
	    	buf_sample_index += samples_received;
	    	SerialBuf.U32_BufSize = buf_sample_index;
	    }
	} else if (BufType == BUF_TYPE_S32) {
	    if (RxCommand == CMD_BUF_START) {
	    	buf_sample_index = 0;
	    	SerialBuf.S32_BufSize = 0;
	    }
		samples_received = (payload_size-1)/BYTES_PER_S32;
		if ((buf_sample_index + samples_received) > SERIAL_BUFFER_MAX_SIZE) return 0;	// Size error!
		else if (samples_received < 1 || samples_received > S32_SAMPLES_PER_PACKET) return 0;

		for (int SampleCount=0; SampleCount<samples_received; SampleCount++) {
			sample.u = (payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 0] << 24);
			sample.u |= (payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 1] << 16);
			sample.u |= (payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 2] << 8);
			sample.u |= payload_buf[payload_offset + SampleCount * BYTES_PER_S32 + 3];
			SerialBuf.S32_Buffer[buf_sample_index + SampleCount] = sample.s;
		}
	    if (RxCommand == CMD_BUF_END) {
	    	buf_sample_index += samples_received;
	    	SerialBuf.S32_BufSize = buf_sample_index;
	    }
	} else if (BufType == BUF_TYPE_F32) {
	    if (RxCommand == CMD_BUF_START) {
	    	buf_sample_index = 0;
	    	SerialBuf.F32_BufSize = 0;
	    }
		samples_received = (payload_size-1)/BYTES_PER_F32;
		if ((buf_sample_index + samples_received) > SERIAL_BUFFER_MAX_SIZE) return 0;	// Size error!
		else if (samples_received < 1 || samples_received > F32_SAMPLES_PER_PACKET) return 0;

		for (int SampleCount=0; SampleCount<samples_received; SampleCount++) {
			sample.u = (payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 0] << 24);
			sample.u |= (payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 1] << 16);
			sample.u |= (payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 2] << 8);
			sample.u |= payload_buf[payload_offset + SampleCount * BYTES_PER_F32 + 3];
			SerialBuf.F32_Buffer[buf_sample_index + SampleCount] = sample.f;
		}
	    if (RxCommand == CMD_BUF_END) {
	    	buf_sample_index += samples_received;
	    	SerialBuf.F32_BufSize = buf_sample_index;
	    }

	} else
		return 0;

    if (RxCommand == CMD_BUF) {
    	buf_sample_index += samples_received;
    }

	return samples_received;
}

// This creates a transmit packet and blocks until the data has finished sending.
// Returns true if an error occurred.
bool oae_serial_send(PacketCommand_t command, uint8_t payload_size, uint8_t *payload)
{
	uint8_t tx_packet_buf[SER_MAX_PACKET_LEN];
    uint8_t checksum = 0;
    uint8_t tx_result;
    int index = 0;
    int i;

    if (payload_size > SER_MAX_PAYLOAD_LEN) {
        SerStats.tx_packet_err_count++;
        return true;
    }
    else SerStats.tx_packet_count++;

    tx_packet_buf[index++] = (uint8_t) SER_PACKET_HEADER;
    tx_packet_buf[index++] = (uint8_t) command;
    tx_packet_buf[index++] = (uint8_t) payload_size;
    for (i = 0; i< payload_size; i++) tx_packet_buf[index++] = (uint8_t) payload[i];

    checksum = 0;
    for (i = 0; i< index; i++) checksum += tx_packet_buf[i];
    checksum = (uint8_t) checksum & 0xFF;     // checksum is the sum of all packet bytes
    tx_packet_buf[index++] = checksum;

    do {
    	tx_result = CDC_Transmit_FS((uint8_t *) tx_packet_buf, index);
    } while (tx_result == USBD_BUSY);

    return false;
}

// Send an error response string:
bool oae_serial_send_error(char *error_str)
{
	uint8_t	TxBuffer[SER_MAX_PAYLOAD_LEN];

	uint8_t buf_len = sprintf((char *)TxBuffer, error_str);
    return oae_serial_send(RSP_ERR, buf_len, (uint8_t *) TxBuffer);
}

// Send a data buffer to the host
//   BufType determines which buffer is sent
bool oae_serial_send_buffer(BufferDataType_t BufType)
{
	int buf_len;
	uint32_t samples_per_packet;
	uint32_t packets_per_buffer;
	uint8_t		TxBuffer[SER_MAX_PAYLOAD_LEN];

	if (BufType == BUF_TYPE_U24) {
		samples_per_packet = U24_SAMPLES_PER_PACKET;
		packets_per_buffer = U24_PACKETS_PER_BUFFER;
	} else if (BufType == BUF_TYPE_F32) {
		samples_per_packet = F32_SAMPLES_PER_PACKET;
		packets_per_buffer = F32_PACKETS_PER_BUFFER;
	} else if (BufType == BUF_TYPE_S32) {
		samples_per_packet = S32_SAMPLES_PER_PACKET;
		packets_per_buffer = S32_PACKETS_PER_BUFFER;
	} else
		return false;	// invalid BufType

	buf_len = oae_build_buf_data_payload(BufType, 0, samples_per_packet, TxBuffer);
	oae_serial_send(RSP_BUF_START, buf_len, (uint8_t *) TxBuffer);
	for (int i=1;i<packets_per_buffer-1;i++) {
		buf_len = oae_build_buf_data_payload(BufType, i*samples_per_packet, samples_per_packet, TxBuffer);
		oae_serial_send(RSP_BUF, buf_len, (uint8_t *) TxBuffer);
	}
	buf_len = oae_build_buf_data_payload(BufType, samples_per_packet*(U24_PACKETS_PER_BUFFER-1), samples_per_packet, TxBuffer);
	oae_serial_send(RSP_BUF_END, buf_len, (uint8_t *) TxBuffer);

	return true;
}

// This function is polled by the calling function.
// It will assemble a packet and return true when a valid packet is available.
bool oae_serial_receive(uint8_t rx_char)
{
    static uint8_t current_rx_index = 0;
    static uint8_t current_payload_count = 0;
    static uint8_t checksum;

    if (current_rx_index == 0) {
        if (rx_char == SER_PACKET_HEADER) {
            RxPacket.header = (uint8_t) rx_char;
            current_rx_index++;
            current_payload_count = 0;
            checksum = (uint8_t) rx_char;
        } else {
            SerStats.rx_packet_err = 1;
            SerStats.rx_packet_err_count++;
            current_rx_index=0;
        }
        return false;
    } else if (current_rx_index == 1) {
        RxPacket.command = (uint8_t) rx_char;
        checksum += (uint8_t) rx_char;
        current_rx_index++;
        return false;
    } else if (current_rx_index == 2) {
        RxPacket.payload_size = (uint8_t) rx_char;
        if (RxPacket.payload_size > SER_MAX_PAYLOAD_LEN) {
            current_rx_index = 0;
            SerStats.rx_packet_err = 2;
            SerStats.rx_packet_err_count++;
        } else {
            checksum += (uint8_t) rx_char;
            current_rx_index++;
        }
        return false;
    } else if (current_rx_index == 3 && RxPacket.payload_size == 0) {
        RxPacket.checksum = (uint8_t) rx_char;
        current_rx_index = 0;
        if ((checksum & 0xFF) == RxPacket.checksum) {
            return true;
        }
        else {
            SerStats.rx_packet_err = 3;
            SerStats.rx_packet_err_count++;
            return false;
        }
    } else if (current_rx_index > 2 && current_payload_count < RxPacket.payload_size) {
        RxPacket.payload[current_payload_count++] = (uint8_t) rx_char;
        checksum += (uint8_t) rx_char;
        current_rx_index++;
        return false;
    } else if (current_rx_index == (3 + RxPacket.payload_size)) {
        RxPacket.checksum = (uint8_t) rx_char;
        current_rx_index = 0;
        if ((checksum  & 0xFF) == RxPacket.checksum) {
        	SerStats.rx_packet_valid_count++;
            return true;
        }
        else {
            SerStats.rx_packet_err = 4;
            SerStats.rx_packet_err_count++;
            return false;
        }
    }
    else {
        current_rx_index = 0;
        SerStats.rx_packet_err = 5;
        SerStats.rx_packet_err_count++;
    }
    return false;
}

// This is the main packet receiver function
void oae_process_rx_packet(void)
{
	static int	cmd_buf_packet_count = 0;
	uint8_t		TxBuffer[SER_MAX_PAYLOAD_LEN];
	int 		buf_len, samples_received;
	bool 		ret;

	SerStats.command_start_time = HAL_GetTick();

	switch(RxPacket.command)
	{
		case CMD_PING:
			oae_serial_send(RSP_PING, 0, (uint8_t *) TxBuffer);
			break;
		case CMD_START:
			if (RxPacket.payload_size != 1) {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			else {
				ret = oae_start_command((ActionCommand_t) RxPacket.payload[0]);
				if (ret == true) oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
				else oae_serial_send(RSP_NAK, 0, (uint8_t *) TxBuffer);
			}
			break;
		case CMD_STOP:
			if (RxPacket.payload_size != 1) {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			else {
				oae_stop_command((ActionCommand_t) RxPacket.payload[0]);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			}
			break;
		case CMD_STATUS:
			buf_len = sprintf((char *)TxBuffer, "OAE_SERIAL_PROTOCOL_VERSION: %s command_turnaround_time: %d tx_packet_count: %d tx_packet_err_count: %d", OAE_SERIAL_PROTOCOL_VERSION, SerStats.command_turnaround_time, SerStats.tx_packet_count, SerStats.tx_packet_err_count);
  	        SerStats.rx_packet_err_count = 0;
  	        SerStats.rx_packet_valid_count = 0;
  	        SerStats.tx_packet_count = 0;
  	        SerStats.tx_packet_err_count = 0;
		    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
		    break;
		case CMD_BUF_REQ:

			if (RxPacket.payload_size != 1) {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			else {
				BufferDataType_t BufType = RxPacket.payload[0];
				ret = oae_serial_send_buffer(BufType);
				if (ret == false) {
					oae_serial_send_error(ERR_STR_INVALID_PARAMETER);
				}
			}
			break;

		case CMD_BUF_START:
			cmd_buf_packet_count = 0;
			samples_received =  oae_receive_buf_data_payload(CMD_BUF_START, RxPacket.payload_size, RxPacket.payload);
			if (samples_received > 0) {
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			break;
		case CMD_BUF:
			cmd_buf_packet_count++;
			samples_received =  oae_receive_buf_data_payload(CMD_BUF, RxPacket.payload_size, RxPacket.payload);
			if (samples_received > 0) {
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			break;
		case CMD_BUF_END:
			cmd_buf_packet_count++;
			samples_received =  oae_receive_buf_data_payload(CMD_BUF_END, RxPacket.payload_size, RxPacket.payload);
			if (samples_received > 0) {
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			break;
		case CMD_I2C_RD:
			if (RxPacket.payload_size != 2) {
				oae_serial_send_error(ERR_STR_INVALID_PAYLOAD_SIZE);
			}
			else {
				uint8_t i2c_device_address = RxPacket.payload[0];
				uint8_t i2c_register_address = RxPacket.payload[1];
				uint8_t i2c_rd_data;

				HAL_I2C_Mem_Read(&hi2c3, i2c_device_address,
						i2c_register_address, 1, &i2c_rd_data, 1, HAL_MAX_DELAY);
				TxBuffer[0] = i2c_rd_data;
			    oae_serial_send(RSP_U8, 1, (uint8_t *) TxBuffer);
			}
			break;

		case CMD_I2C_WR:
			if (RxPacket.payload_size != 3) {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			else {
				uint8_t i2c_device_address = RxPacket.payload[0];
				uint8_t i2c_register_address = RxPacket.payload[1];
				uint8_t i2c_wr_data = RxPacket.payload[2];

				HAL_I2C_Mem_Write(&hi2c3, i2c_device_address,
						i2c_register_address, 1, &i2c_wr_data, 1, HAL_MAX_DELAY);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			}
			break;

		default:
		    oae_serial_send(RSP_INVALID, 0, (uint8_t *) TxBuffer);
			break;
	}
	SerStats.command_turnaround_time = HAL_GetTick() - SerStats.command_start_time;

}

// Copy USB RX data from the receive buffer (USB_RxBuffers) to a user buffer
// Return the user buffer and buffer length to the calling function.
// Increment the USB_RxBuffers.pos_process pointer to mark the buffer as available.

int8_t RX_USB_CDC_Data(uint8_t* Buf, uint32_t *Len)
{
    if (USB_RxBuffers.IsCommandDataReceived==0) return 0; //no data received

    int index=USB_RxBuffers.pos_process;
    *Len=USB_RxBuffers.CommandsLens[index];     //return the length
    memcpy(Buf,USB_RxBuffers.UserRxBufferFS[index],*Len);

    //check if all data were processed.
    USB_RxBuffers.pos_process++;
    if (USB_RxBuffers.pos_process>=MAX_RX_BUFFERS) //reach the last buffer, need to rewind to 0
    {
        USB_RxBuffers.pos_process=0;
    }
    if (USB_RxBuffers.pos_process==USB_RxBuffers.pos_receive)
    	USB_RxBuffers.IsCommandDataReceived=0;

    return 1;
}
