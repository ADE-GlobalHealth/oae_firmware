/*
 * oae_serial.c
 *
 *  Created on: Mar 24, 2025
 *  	Olin College of Engineering
 *      Author: Mike Deeds
 *
 *
 */

#include "usbd_cdc_if.h"
#include "oae_serial.h"

SerialStats_t SerStats;
SerialPacket_t RxPacket;

uint32_t	ADC_Buffer[ADC_BUFFER_SIZE];

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

	oae_fill_adc_buffer(ADC_Buffer);
}

bool oae_start_command(uint8_t command_num)
{
	// add start commands here
	return true;
}

bool oae_stop_command(uint8_t command_num)
{
	// add stop commands here
	return true;
}

// Fill the ADC buffer with test data:
void oae_fill_adc_buffer(uint32_t *ADCBuf)
{
	int SampleCount;
	int PacketCount;
	int i = 0;

	for (PacketCount = 0; PacketCount < ADC_PACKETS_PER_BUFFER; PacketCount++) {
		for (SampleCount = 0; SampleCount < ADC_SAMPLES_PER_PACKET; SampleCount++) {

			// Create 24 bit test data with the packet number in the upper 8 bits:
			ADCBuf[i] = (PacketCount << 16) | (i & 0xFFFF);
			i++;
		}
	}
}

// Fills a packet payload buffer with 24 bit ADC test data.
// The function returns the number of bytes in the payload
uint32_t oae_fill_adc_data_payload(uint32_t ADCBuf_starting_index, uint32_t *ADCBuf, uint32_t num_samples, uint8_t *payload_buf)
{
	uint32_t sample;

	if ((num_samples * ADC_BYTES_PER_SAMPLE) > SER_MAX_PAYLOAD_LEN) return 0;	// Size error!

	for (int SampleCount=0; SampleCount<num_samples; SampleCount++) {

		sample = ADCBuf[ADCBuf_starting_index + SampleCount];
		payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 0] = (sample >> 16) & 0xFF;
		payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 1] = (sample >> 8) & 0xFF;
		payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 2] = (sample) & 0xFF;
	}
	return (num_samples * ADC_BYTES_PER_SAMPLE);
}

// Copy one packet of received ADC data into the specified ADCBuf
uint32_t oae_receive_adc_data_payload(uint32_t ADCBuf_starting_index, uint32_t *ADCBuf, uint32_t num_samples, uint8_t *payload_buf)
{
	uint32_t sample;

	if ((ADCBuf_starting_index + num_samples) > ADC_BUFFER_SIZE) return 0;	// Size error!

	for (int SampleCount=0; SampleCount<num_samples; SampleCount++) {

		sample = (payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 0] << 16);
		sample |= (payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 1] << 8);
		sample |= payload_buf[SampleCount * ADC_BYTES_PER_SAMPLE + 2];
		ADCBuf[ADCBuf_starting_index + SampleCount] = sample;
	}
	return (num_samples * ADC_BYTES_PER_SAMPLE);
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

void oae_process_rx_packet(void)
{
	static int	cmd_adc_buf_packet_count = 0;
	uint8_t		TxBuffer[APP_RX_DATA_SIZE];
	int 		buf_len;

	SerStats.command_start_time = HAL_GetTick();

	switch(RxPacket.command)
	{
		case CMD_PING:
			oae_serial_send(RSP_PING, 0, (uint8_t *) TxBuffer);
			break;
		case CMD_START:
			if (RxPacket.payload_size != 1) {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			else {
				oae_start_command(RxPacket.payload[0]);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			}
			break;
		case CMD_STOP:
			if (RxPacket.payload_size != 1) {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			else {
				oae_stop_command(RxPacket.payload[0]);
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
		case CMD_ADC_BUF_REQ:

			if (RxPacket.payload_size != 1) {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			else {
				if (RxPacket.payload[0] == 0) {
					oae_fill_adc_buffer(ADC_Buffer);		// send back test data, otherwise send back the current contents of the buffer
				}

				buf_len = oae_fill_adc_data_payload(0, ADC_Buffer, ADC_SAMPLES_PER_PACKET, TxBuffer);
				oae_serial_send(RSP_ADC_BUF_START, buf_len, (uint8_t *) TxBuffer);
				for (int i=1;i<ADC_PACKETS_PER_BUFFER-1;i++) {
					buf_len = oae_fill_adc_data_payload(i*ADC_SAMPLES_PER_PACKET, ADC_Buffer, ADC_SAMPLES_PER_PACKET, TxBuffer);
					oae_serial_send(RSP_ADC_BUF, buf_len, (uint8_t *) TxBuffer);
				}
				buf_len = oae_fill_adc_data_payload(ADC_SAMPLES_PER_PACKET*(ADC_PACKETS_PER_BUFFER-1), ADC_Buffer, ADC_SAMPLES_PER_PACKET, TxBuffer);
				oae_serial_send(RSP_ADC_BUF_END, buf_len, (uint8_t *) TxBuffer);
			}
			break;

		case CMD_ADC_BUF_START:
			cmd_adc_buf_packet_count = 0;
			if (RxPacket.payload_size == ADC_SAMPLES_PER_PACKET * ADC_BYTES_PER_SAMPLE) {
				oae_receive_adc_data_payload(cmd_adc_buf_packet_count * ADC_SAMPLES_PER_PACKET, ADC_Buffer, ADC_SAMPLES_PER_PACKET, RxPacket.payload);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			break;
		case CMD_ADC_BUF:
			if (RxPacket.payload_size == ADC_SAMPLES_PER_PACKET * ADC_BYTES_PER_SAMPLE) {
				cmd_adc_buf_packet_count++;
				oae_receive_adc_data_payload(cmd_adc_buf_packet_count * ADC_SAMPLES_PER_PACKET, ADC_Buffer, ADC_SAMPLES_PER_PACKET, RxPacket.payload);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
			}
			break;
		case CMD_ADC_BUF_END:
			if (RxPacket.payload_size == ADC_SAMPLES_PER_PACKET * ADC_BYTES_PER_SAMPLE) {
				cmd_adc_buf_packet_count++;
				oae_receive_adc_data_payload(63 * ADC_SAMPLES_PER_PACKET, ADC_Buffer, ADC_SAMPLES_PER_PACKET, RxPacket.payload);
				oae_serial_send(RSP_ACK, 0, (uint8_t *) TxBuffer);
			} else {
				buf_len = sprintf((char *)TxBuffer, ERR_STR_INVALID_PAYLOAD_SIZE);
			    oae_serial_send(RSP_TEXT, buf_len, (uint8_t *) TxBuffer);
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
    //Buf[*Len]= '\0'; //testing only. make sure there is ending char in the returned command string
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
