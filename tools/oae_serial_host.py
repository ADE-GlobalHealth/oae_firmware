# oae_serial_host.py
#
# Olin College of Engineering
#   3/24/2025  Mike Deeds
#
# Overview:
#   This module implements the oae_serial packet protocol that can run over a serial UART or USB CDC.
#   It is designed primarily as a command/response system initiated by the host, though
#   debug / status / event messages may be sent any time by the embedded device.
#
# OAE serial packet format:
#   Byte 0: Header: 0x7E
#   Byte 1: Command
#   Byte 2: Payload_size
#   Bytes 3 to Payload_size + 3
#   Byte N: checksum (sum of all bytes, truncated to 8 bits)
#
#

import sys
import threading
import os
import datetime
import time
from time import sleep
import serial
import serial.tools.list_ports as port_list
import numpy as np

OAE_SERIAL_PROTOCOL_VERSION = "v1.3"

PACKET_HEADER_BYTE = 0x7E

BUF_TYPE_NONE 		= 0		# undefined
BUF_TYPE_U24		= 1		# 24 bit unsigned int
BUF_TYPE_S32		= 2		# 32 bit signed int
BUF_TYPE_F32		= 3		# 32 bit float

SERIAL_BUFFER_MAX_SIZE = 4096
    
BYTES_PER_U16  			= 2
BYTES_PER_U24  			= 3
BYTES_PER_S32  			= 4
BYTES_PER_F32  			= 4
                        
U16_SAMPLES_PER_PACKET 	= 64			# 16 bits per sample * 64 samples/packet = 128 byte payload size
U24_SAMPLES_PER_PACKET 	= 64			# 24 bits per sample * 64 samples/packet = 192 byte payload size
F32_SAMPLES_PER_PACKET 	= 32			# 32 bits per sample * 32 samples/packet = 128 byte payload size
S32_SAMPLES_PER_PACKET 	= 32			# 32 bits per sample * 32 samples/packet = 128 byte payload size
                         
U24_PACKETS_PER_BUFFER  = (SERIAL_BUFFER_MAX_SIZE / U24_SAMPLES_PER_PACKET)
F32_PACKETS_PER_BUFFER  = (SERIAL_BUFFER_MAX_SIZE / F32_SAMPLES_PER_PACKET)
S32_PACKETS_PER_BUFFER  = (SERIAL_BUFFER_MAX_SIZE / S32_SAMPLES_PER_PACKET)

# Host commands:
CMD_NOP             = 0     # No payload, no response expected
CMD_PING            = 1     # Ping, no payload, RSP_PING response expected
CMD_STATUS          = 2     # Request status from OAE, no payload, multiple RSP_TEXT responses expected
CMD_BUF_REQ 	    = 3     # Payload: 1 byte: BUF_TYPE
CMD_BUF_START 	    = 4     # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer. RSP_ACK or RSP_ERR response expected
CMD_BUF 		    = 5     # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. No response expected (there will be 62 of these packets in a 4096 sample buffer)
CMD_BUF_END		    = 6     # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer. RSP_ACK or RSP_ERR response expected
CMD_I2C_RD		 	= 7	    # Payload: 2 bytes: U8 I2C device address, U8 I2C register address, RSP_U8 response expected (I2C read data)
CMD_I2C_WR			= 8     # Payload: 3 bytes: U8 I2C device address, U8 I2C register address, U8 I2C write data, RSP_ACK or RSP_ERR response expected
CMD_START			= 9     # Payload: 1 byte: U8, which command to start, RSP_ACK or RSP_ERR response expected
CMD_STOP			= 10    # Payload: 1 byte: U8, which command to stop, RSP_ACK or RSP_ERR response expected
CMD_OK              = 11    # No payload

# OAE Embedded device responses:
RSP_PING            = 101   # Ping response, no payload
RSP_ACK             = 102   # No payload
RSP_NAK             = 103   # No payload
RSP_ERR             = 104   # Payload: Up to 250 bytes, text string
RSP_TEXT            = 105   # Payload: Up to 250 bytes, text string
RSP_BUF_START 	    = 106   # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer
RSP_BUF 		    = 107   # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data.
RSP_BUF_END		    = 108   # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer
RSP_U8 		        = 109   # Payload: 1 byte:  U8
RSP_U32 			= 110   # Payload: 4 bytes: U32
RSP_EVENT			= 111   # Payload: 1 byte:  U8 (event number)
RSP_INVALID 		= 112   # No payload (Command from host was not recognized)

ACTION_OAE_TEST 	= 1		# Run the OAE test once (does not require a stop command)

class oae_serial_host:
    RxQ = []
    RxPayload = []
    RxSilent = False
    RxData_u8 = np.empty(4, dtype=np.uint8)
    RxDataValid = False
    RxPayload_u8 = []
    RxPacketIndex = 0
    RxPayloadSize = 0
    RxCommand = 0
    CurrentTxCommand = CMD_NOP
    TxCommandsActive = 0
    RxChecksum = 0
    ValidRXPacket = False
    RxAudioBufferStartTime = 0
    TxAudioBufferStartTime = 0
    CmdTime = 0
    RspTime = 0
    Buffer_PacketCount = 0
    Buffer_SamplesReceived = 0
    Buffer_TotalSamplesReceived = 0
    
    ADC_RxBuf = []
    RxPacketTimes = []
    RxAudioBuffer = []
    TxAudioBuffer = []
        

    def __init__(self, comport):
        self.logfile_open = False
        self.ConsolePrint = True
        self.open_logfile()

        if "COM" in comport:        # Windows
            self.isSerial = True
            self.serial_init(comport)
        elif "/dev" in comport:     # Linux
            self.isSerial = True
            self.serial_init(comport)
        else:
            self.isSerial = False

    def open_logfile(self):
        if self.logfile_open:
            self.close_logfile()

        logs_dir = os.path.join(os.getcwd(), "logs")
        os.makedirs(logs_dir, exist_ok=True)

        # Create a unique logfile based on the current date + time:
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_name = f"logs\\{current_time}.log"
        print(f"\tOpening log file: {self.log_file_name}")
        self.log = open(self.log_file_name, "a")
        self.logfile_open = True
        self.log.write(f"OAE serial host log file: {self.log_file_name}\n")

    def writeLog(self, text):
        if self.ConsolePrint:
            print(text)
        t2 = text + '\n'
        if self.log:
            self.log.write(t2)
            self.log.flush() # Ensure data is written to disk immediately            

    def closeLog(self):
        if self.log:
            self.log.close()
            
    def save_audio_buffer(self, AudioBuffer, filename_prefix = "Audio_buffer"):
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"logs\\{filename_prefix}_{current_time}.txt"
        self.writeLog(f"\tSaving audio buffer: {filename}")
        f = open(filename, "w")
        for i in range(len(AudioBuffer)):
            sampleText = f"{hex(AudioBuffer[i])}"
            t2 = sampleText + '\n'

            f.write(t2)
            if False:
                if i < 10:
                    self.writeLog(f"\t{i}: {sampleText}")
                elif i > 4090:
                    self.writeLog(f"\t{i}: {sampleText}")
        f.close()
        

    def serial_init(self, port, baudrate=921600, timeout=.01):
        """Initialize serial connection."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.writeLog(f"Connected to {port} at {baudrate} baud.")

        except serial.SerialException as e:
            self.writeLog(f"Error: Unable to open port {port} - {e}")
            sys.exit(1)

        # Python thread switch interval defaults to 5msec.
        sys.setswitchinterval(.001)
        threadSwitchTime = sys.getswitchinterval()
        self.writeLog(f"threadSwitchTime: {threadSwitchTime}")

        # Start a thread to read from the serial port
        thread = threading.Thread(target=self.read_from_port)
        thread.daemon = True
        thread.start()

    # Function to read from the serial port and process incoming data
    #   This is run in a thread
    def read_from_port(self):
        while True:
            if self.ser._port_handle and self.ser.in_waiting > 0:
                # Read up to 64 bytes
                byte_data = self.ser.read(64)
                for i in range(len(byte_data)):
                #if len(byte_data) > 0:                
                    unsigned_byte = byte_data[i]
                    #unsigned_byte = int.from_bytes(byte_data, byteorder='big', signed=False) & 0xFF
                    #self.writeLog(f"read_from_port: {unsigned_byte.hex()}")
                    self.RxQ.append(unsigned_byte)

            while len(self.RxQ) > 0:
                self.build_rx_packet()
                if self.ValidRXPacket:
                    self.RxPacketTimes.append(time.perf_counter())
                    self.process_rx_response()

    def command_name(self, command):
        if command == CMD_NOP:
            return "CMD_NOP"
        elif command == CMD_PING:
            return "CMD_PING"
        elif command == CMD_STATUS:
            return "CMD_STATUS"
        elif command == CMD_BUF_REQ:
            return "CMD_BUF_REQ"
        elif command == CMD_BUF_START:
            return "CMD_BUF_START"
        elif command == CMD_BUF:
            return "CMD_BUF"
        elif command == CMD_BUF_END:
            return "CMD_BUF_END"
        elif command == CMD_I2C_RD:
            return "CMD_I2C_RD"
        elif command == CMD_I2C_WR:
            return "CMD_I2C_WR"
        elif command == CMD_START:
            return "CMD_START"
        elif command == CMD_STOP:
            return "CMD_STOP"
        elif command == CMD_OK:
            return "CMD_OK"
        elif command == RSP_PING:
            return "RSP_PING"
        elif command == RSP_ACK:
            return "RSP_ACK"
        elif command == RSP_NAK:
            return "RSP_NAK"
        elif command == RSP_ERR:
            return "RSP_ERR"
        elif command == RSP_TEXT:
            return "RSP_TEXT"
        elif command == RSP_U8:
            return "RSP_U8"
        elif command == RSP_U32:
            return "RSP_U32"
        elif command == RSP_EVENT:
            return "RSP_EVENT"
        elif command == RSP_BUF_START:
            return "RSP_BUF_START"
        elif command == RSP_BUF:
            return "RSP_BUF"
        elif command == RSP_BUF_END:
            return "RSP_BUF_END"
        elif command == RSP_INVALID:
            return "RSP_INVALID"
        else:
            return f"INVALID Command {command}"

    def build_rx_packet(self):
        if len(self.RxQ) == 0:
            return
        elif self.ValidRXPacket:
            return  # Wait until the current packet has been processed.
        else:
            rx_byte = self.RxQ.pop(0)
            #self.writeLog(f"pop: {hex(rx_byte)} RxPacketIndex: {self.RxPacketIndex} ")
            if (self.RxPacketIndex == 0) & (rx_byte & 0xFF == 0x7E):
                self.RxChecksum = rx_byte
                #self.writeLog(f"\tPacket Header: {hex(rx_byte)}  computed checksum: {self.RxChecksum}")
                self.RspPktHeaderTime = time.perf_counter()
                self.RxPacketIndex = 1
            elif self.RxPacketIndex == 1:
                # self.writeLog(f"\tPacket command: {rx_byte} type: {type(rx_byte)}")
                self.RxCommand = rx_byte
                self.RxChecksum += rx_byte
                #self.writeLog(f"\tRxCommand: {self.RxCommand}")
                self.RxPacketIndex += 1
            elif self.RxPacketIndex == 2:
                self.RxPayloadSize = rx_byte
                self.RxChecksum += rx_byte
                #if self.RxPayloadSize > 0:
                #    self.writeLog(f"\tRxPayloadSize: {self.RxPayloadSize}  computed checksum: {self.RxChecksum}")
                self.RxPacketIndex += 1
            elif (self.RxPayloadSize > 0) & (self.RxPacketIndex > 2) & (self.RxPacketIndex < (self.RxPayloadSize + 3)):
                if self.RxPacketIndex == 3:
                    self.RxPayload.clear()
                    self.RxPayload_u8.clear()

                if self.RxCommand == RSP_U8:
                    self.RxPayload_u8.append(rx_byte)
                elif self.RxCommand == RSP_BUF_START:
                    self.RxPayload_u8.append(rx_byte)
                elif self.RxCommand == RSP_BUF:
                    self.RxPayload_u8.append(rx_byte)
                elif self.RxCommand == RSP_BUF_END:
                    self.RxPayload_u8.append(rx_byte)
                elif rx_byte != 0x0:  # don't save null characters
                    self.RxPayload.append(chr(rx_byte))
                self.RxChecksum += rx_byte
                self.RxPacketIndex += 1
            elif self.RxPacketIndex == self.RxPayloadSize + 3:
                if (self.RxChecksum) & 0xFF == rx_byte:
                    self.ValidRXPacket = True
                    self.PktRxTime = time.perf_counter()
                    self.RxPacketIndex = 0
                else:
                    self.writeLog(f"Received invalid packet: RxCommand: {self.RxCommand} RxPayloadSize: {self.RxPayloadSize} computed checksum: {self.RxChecksum}")
                    self.RxPacketIndex = 0
            #else:
            #    self.writeLog(f"\tRx invalid byte: {hex(rx_byte)} {chr(rx_byte)}")
            
    def process_rx_buffer_payload(self):
                
        if self.Buffer_SamplesReceived != U24_SAMPLES_PER_PACKET:
            self.writeLog(f"\tErr: rx_audio_buffer_payload: RxPayloadSize: {self.RxPayloadSize} Buffer_SamplesReceived: {self.Buffer_SamplesReceived} U24_SAMPLES_PER_PACKET: {U24_SAMPLES_PER_PACKET}");
            
        for i in range(self.Buffer_SamplesReceived):
            AudioSample = (self.RxPayload_u8[i*BYTES_PER_U24] << 16) | (self.RxPayload_u8[i*BYTES_PER_U24 + 1] << 8)  | self.RxPayload_u8[i*BYTES_PER_U24 + 2]
            self.RxAudioBuffer.append(AudioSample)
                

    def process_rx_response(self):
        self.RspTime = time.perf_counter()
        self.RoundTripTime = int((self.RspTime - self.CmdTime) * 1e6)

        if self.RxPayloadSize > 0:
            if self.RxCommand == RSP_U32:
                data_u32 = self.RxPayload_u8[0]
                self.RxData_u8[0] = self.RxPayload_u8[0]
                for j in range(3):
                    data_u32 = data_u32 << 8
                    data_u32 = data_u32 | self.RxPayload_u8[j + 1]
                    self.RxData_u8[j + 1] = self.RxPayload_u8[j + 1]
                self.RxDataValid = True
                if self.RxSilent == False:
                    self.writeLog(f"\t{self.command_name(self.RxCommand)} Payload: {hex(data_u32)} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec")
            elif self.RxCommand == RSP_U8:
                Data = self.RxPayload_u8[0]
                self.writeLog(f"\t{self.command_name(self.RxCommand)} Data: {hex(Data)} PktRxTime: {self.PktRxTime} sec")
            elif self.RxCommand == RSP_EVENT:
                EventNumber = self.RxPayload_u8[0]
                self.writeLog(f"\t{self.command_name(self.RxCommand)} Event Number: {EventNumber} PktRxTime: {self.PktRxTime} sec")
            elif self.RxCommand == RSP_BUF_START:
                self.Buffer_PacketCount = 1
                self.RxAudioBufferStartTime = time.perf_counter()
                self.RxAudioBuffer = []                
                self.Buffer_SamplesReceived = int((self.RxPayloadSize-1)/BYTES_PER_U24)                    
                if self.RxSilent == False:
                    self.writeLog(f"\t{self.command_name(self.RxCommand)} # Buffer_SamplesReceived: {self.Buffer_SamplesReceived} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec")
                    
                self.Buffer_TotalSamplesReceived = self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()
                
            elif self.RxCommand == RSP_BUF:
                self.Buffer_PacketCount += 1
                self.Buffer_SamplesReceived = int((self.RxPayloadSize-1)/BYTES_PER_U24)                    
                self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()
                
            elif self.RxCommand == RSP_BUF_END:
                self.Buffer_PacketCount += 1
                self.Buffer_SamplesReceived = int((self.RxPayloadSize-1)/BYTES_PER_U24)                    
                self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()
                if self.RxSilent == False:
                    elapsedTime = time.perf_counter() - self.RxAudioBufferStartTime
                    self.writeLog(f"\t{self.command_name(self.RxCommand)} Buffer_PacketCount: {self.Buffer_PacketCount} # samples: {self.Buffer_TotalSamplesReceived} RoundTripTime: {self.RoundTripTime} usec elapsedTime: {elapsedTime} sec")
                    if False:       # debug packet timing
                        for i in range(len(self.RxPacketTimes)):
                            if i==0:
                                self.writeLog(f"\t\tpacket {i} time: {self.RxPacketTimes[i]} delay: {self.RxPacketTimes[0] - self.CmdTime}")
                            else:
                                self.writeLog(f"\t\tpacket {i} time: {self.RxPacketTimes[i]} delay: {self.RxPacketTimes[i] - self.RxPacketTimes[i-1]}")
                self.save_audio_buffer(self.RxAudioBuffer)
                        
            else:
                if self.RxCommand == RSP_ERR:
                    if self.TxCommandsActive > 0: self.TxCommandsActive -= 1
                
                payloadStr = ''.join(self.RxPayload)
                self.writeLog(f"\t{self.command_name(self.RxCommand)} {payloadStr}")
                self.writeLog(f"\t\tRoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec")
        else:
            if self.RxCommand == RSP_ACK:
                if self.TxCommandsActive > 0: self.TxCommandsActive -= 1
                
            if self.CurrentTxCommand != CMD_BUF:
                self.writeLog(f"\t{self.command_name(self.RxCommand)} no payload. RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec ")

        self.ValidRXPacket = False

    # send a packet as a byte array:
    def send_TxPacket(self, TxCommand, TxPayload):
        TxPacket = []
        Checksum = 0
        self.CurrentTxCommand = TxCommand
        if TxCommand != CMD_BUF:
            self.writeLog(f"send_TxPacket: {self.command_name(TxCommand)}")
        TxPacket.append(PACKET_HEADER_BYTE)
        Checksum += PACKET_HEADER_BYTE
        TxPacket.append(TxCommand)
        Checksum += TxCommand
        PayloadSize = len(TxPayload)
        TxPacket.append(PayloadSize)
        Checksum += PayloadSize
        if PayloadSize > 0:
            for p in TxPayload:
                p_u8 = p & 0xFF
                TxPacket.append(p_u8)
                Checksum += p_u8

        Checksum = Checksum & 0xFF  # truncate to 8 bits
        TxPacket.append(Checksum)
        self.CmdTime = time.perf_counter()
        self.ser.write(TxPacket)
                
        
    def upload_buffer_packet(self, Buf_Command, TxAudioBuffer, SampleIndexOffset):

        TxPayload = []
        TxPayload.append(BUF_TYPE_U24)       
        sampleCount = 0
        for i in range(U24_SAMPLES_PER_PACKET):
            sample = TxAudioBuffer[SampleIndexOffset + sampleCount]
            sampleCount += 1
            TxPayload.append((sample >> 16) & 0xFF)
            TxPayload.append((sample >> 8) & 0xFF)
            TxPayload.append(sample & 0xFF)
            
        self.send_TxPacket(Buf_Command, TxPayload)

    def upload_test_buffer(self):
    
        AudioPacketsPerBuffer =  int(SERIAL_BUFFER_MAX_SIZE / U24_SAMPLES_PER_PACKET)
        TxAudioBuf = []
        for i in range(AudioPacketsPerBuffer):
            for j in range(U24_SAMPLES_PER_PACKET):
                TxAudioBuf.append((i & 0xFF) | ((i * U24_SAMPLES_PER_PACKET + j) & 0xFFFF)<<8)

        print(f"upload_test_buffer AudioPacketsPerBuffer: {AudioPacketsPerBuffer} TxAudioBuf[4095]: {hex(TxAudioBuf[4095])}")
        self.TxCommandsActive = 0
            
        SampleIndex = 0
        TxAudioBufferStartTime = time.perf_counter()
        self.writeLog(f"\tCMD_BUF_START time: {TxAudioBufferStartTime}")
        self.upload_buffer_packet(CMD_BUF_START, TxAudioBuf, SampleIndex)   
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        for i in range(AudioPacketsPerBuffer - 2):
            while self.TxCommandsActive > 5:
                elapsedTime = time.perf_counter() - TxAudioBufferStartTime
                if elapsedTime > 2.0:
                    self.writeLog(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                    break
                else:
                    sleep(0.001)
                
            self.upload_buffer_packet(CMD_BUF, TxAudioBuf, SampleIndex)   
            self.TxCommandsActive += 1
            SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                self.writeLog(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)
            
        self.upload_buffer_packet(CMD_BUF_END, TxAudioBuf, SampleIndex)    
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                self.writeLog(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)
        
        self.writeLog(f"\tCMD_BUF_END SampleIndex: {SampleIndex} elapsedTime: {elapsedTime}")

    def i2c_rd(self, device_addr, device_register_addr):
        TxPayload = []
        TxPayload.append(device_addr)            
        TxPayload.append(device_register_addr)            
        self.send_TxPacket(CMD_I2C_RD, TxPayload)

    def i2c_wr(self, device_addr, device_register_addr, i2c_wr_data):
        TxPayload = []
        TxPayload.append(device_addr)            
        TxPayload.append(device_register_addr)            
        TxPayload.append(i2c_wr_data)            
        self.send_TxPacket(CMD_I2C_WR, TxPayload)

    def command_response(self, command, TxPayload = None):
        EmptyPayload = []
        if command == CMD_BUF_REQ:
            TxPayload = []
            TxPayload.append(BUF_TYPE_U24)                           
            self.RxPacketTimes = []
            self.send_TxPacket(CMD_BUF_REQ, TxPayload)
            self.writeLog(f"\tCMD_BUF_REQ time: {time.perf_counter()}")
        elif command == CMD_BUF_START:
            self.upload_test_buffer()
        else:
            if TxPayload == None:
                self.send_TxPacket(command, EmptyPayload)
            else: 
                self.send_TxPacket(command, TxPayload)

    def print_menu(self):
        self.writeLog("Menu: ")
        self.writeLog("\t1) \tCMD_PING ")
        self.writeLog("\t2) \tCMD_STATUS ")
        self.writeLog("\t3) \tCMD_BUF Upload (host test pattern)")
        self.writeLog("\t4) \tCMD_BUF Request 0 (oae test pattern)")
        self.writeLog("\t5) \tCMD_BUF Request 1 (current oae buffer)")
        self.writeLog("\t6) \tCMD_START ACTION_OAE_TEST")
        self.writeLog("\t7) \tCMD_STOP ")
        self.writeLog("\t8) \tCMD_I2C_RD ")
        self.writeLog("\t9) \tCMD_I2C_WR ")
        self.writeLog("\t? or h) Print this menu")
        self.writeLog("\tq) \tQuit")

    def serial_user_interface(self):
        self.print_menu()
        while True:
            user_input = input("")
            if len(user_input) == 0:
                continue
            elif user_input[0] == '1':
                self.command_response(CMD_PING)
            elif user_input[0] == '2':
                self.command_response(CMD_STATUS)
            elif user_input[0] == '3':
                self.command_response(CMD_BUF_START)
            elif user_input[0] == '4':
                BufferNum = 0
                TxPayload = []
                TxPayload.append(BufferNum)            
                self.command_response(CMD_BUF_REQ, TxPayload)
            elif user_input[0] == '5':
                BufferNum = 1
                TxPayload = []
                TxPayload.append(BufferNum)            
                self.command_response(CMD_BUF_REQ, TxPayload)
            elif user_input[0] == '6':
                CommandNum = ACTION_OAE_TEST
                TxPayload = []
                TxPayload.append(CommandNum)            
                self.command_response(CMD_START, TxPayload)
            elif user_input[0] == '7':
                CommandNum = 0
                TxPayload = []
                TxPayload.append(CommandNum)            
                self.command_response(CMD_STOP, TxPayload)
            elif user_input[0] == '8':
                self.i2c_rd(device_addr = 0x9C, device_register_addr = 0x70)                
            elif user_input[0] == '9':
                self.i2c_wr(device_addr = 0x9C, device_register_addr = 0x70, i2c_wr_data = 0x75)                
            elif user_input[0] == 'h':
                self.print_menu()
            elif user_input[0] == '?':
                self.print_menu()
            elif user_input[0] == 'q':
                return
            else:
                self.writeLog(f"Invalid command: {user_input}")

def main():

    print(f"OAE serial protocol version: {OAE_SERIAL_PROTOCOL_VERSION}")

    if len(sys.argv) != 2:
        print("\tUsage: python -m oae_serial_host <COM Port>")
        print("\tLog and adc data files are saved to the logs/ subdirectory.")

        ports = list(port_list.comports())

        if not ports:
            print("No serial ports found.")
            return
        else:
            print("Available Serial Ports:")
            for port in ports:
                print(f"- {port.device} ({port.description})")
            return

    oae = oae_serial_host(sys.argv[1])
    oae.serial_user_interface()
    oae.closeLog()

if __name__ == "__main__":
    main()