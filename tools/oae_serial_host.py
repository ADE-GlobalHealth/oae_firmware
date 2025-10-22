# This module implements the oae_serial packet protocol that can run over a serial
# UART or USB CDC.
# It is designed primarily as a command/response system initiated by the host, though
# debug / status / event messages may be sent any time by the embedded device.
#
# OAE serial packet format:
#   Byte 0: Header: 0x7E
#   Byte 1: Command
#   Byte 2: Payload_size
#   Bytes 3 to Payload_size + 3
#   Byte N: checksum (sum of all bytes, truncated to 8 bits)

import argparse
import datetime
import os
import sys
import threading
import time
from enum import IntEnum
from time import sleep
import logging

import numpy as np
import serial
import serial.tools.list_ports as port_list

OAE_DEVICE_NAME = "Global Health OAE Device"  # defined in usbd_desc.c, used to automatically connect

PACKET_HEADER_BYTE = 0x7E

BUF_TYPE_NONE = 0  # undefined
BUF_TYPE_U24 = 1  # 24 bit unsigned int
BUF_TYPE_S32 = 2  # 32 bit signed int
BUF_TYPE_F32 = 3  # 32 bit float

SERIAL_BUFFER_MAX_SIZE = 4096

BYTES_PER_U16 = 2
BYTES_PER_U24 = 3
BYTES_PER_S32 = 4
BYTES_PER_F32 = 4

U16_SAMPLES_PER_PACKET = (
    64  # 16 bits per sample * 64 samples/packet = 128 byte payload size
)
U24_SAMPLES_PER_PACKET = (
    64  # 24 bits per sample * 64 samples/packet = 192 byte payload size
)
F32_SAMPLES_PER_PACKET = (
    32  # 32 bits per sample * 32 samples/packet = 128 byte payload size
)
S32_SAMPLES_PER_PACKET = (
    32  # 32 bits per sample * 32 samples/packet = 128 byte payload size
)

U24_PACKETS_PER_BUFFER = SERIAL_BUFFER_MAX_SIZE / U24_SAMPLES_PER_PACKET
F32_PACKETS_PER_BUFFER = SERIAL_BUFFER_MAX_SIZE / F32_SAMPLES_PER_PACKET
S32_PACKETS_PER_BUFFER = SERIAL_BUFFER_MAX_SIZE / S32_SAMPLES_PER_PACKET


class Command(IntEnum):
    """
    Host commands.

    Host commands flow from the host computer to the OAE embedded device.
    """

    def __str__(self):
        return "CMD_" + self.name

    NOP = 0  # No payload, no response expected
    PING = 1  # Ping, no payload, RSP_PING response expected
    STATUS = (
        2  # Request status from OAE, no payload, multiple RSP_TEXT responses expected
    )
    BUF_REQ = 3  # Payload: 1 byte: BUF_TYPE
    BUF_START = 4  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer. RSP_ACK or RSP_ERR response expected
    BUF = 5  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. No response expected (there will be 62 of these packets in a 4096 sample buffer)
    BUF_END = 6  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer. RSP_ACK or RSP_ERR response expected
    I2C_RD = 7  # Payload: 2 bytes: U8 I2C device address, U8 I2C register address, RSP_U8 response expected (I2C read data)
    I2C_WR = 8  # Payload: 3 bytes: U8 I2C device address, U8 I2C register address, U8 I2C write data, RSP_ACK or RSP_ERR response expected
    STOP = 9  # Payload: 1 byte: U8, which command to stop, RSP_ACK or RSP_ERR response expected
    OK = 10  # No payload
    OAE_TEST = 11  # Run the OAE test once (does not require a stop command)


class Response(IntEnum):
    """
    OAE device responses.

    Responses flow from the OAE device to the host computer.
    """

    def __str__(self):
        return "RSP_" + self.name

    PING = 101  # Ping response, no payload
    ACK = 102  # No payload
    NAK = 103  # No payload
    ERR = 104  # Payload: Up to 250 bytes, text string
    TEXT = 105  # Payload: Up to 250 bytes, text string
    BUF_START = 106  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer
    BUF = 107  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data.
    BUF_END = 108  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer
    U8 = 109  # Payload: 1 byte:  U8
    U32 = 110  # Payload: 4 bytes: U32
    EVENT = 111  # Payload: 1 byte:  U8 (event number)
    INVALID = 112  # No payload (Command from host was not recognized)
    LOG_DEBUG = 113  # Payload: Up to 250 bytes, text string
    LOG_INFO = 114  # Payload: Up to 250 bytes, text string
    LOG_WARNING = 115  # Payload: Up to 250 bytes, text string
    LOG_ERROR = 116  # Payload: Up to 250 bytes, text string
    LOG_CRITICAL = 117  # Payload: Up to 250 bytes, text string


class oae_serial_host:
    RxQ = []
    RxPayload = []
    RxSilent = False
    RxData_u8 = np.empty(4, dtype=np.uint8)
    RxDataValid = False
    RxPayload_u8 = []
    RxPacketIndex = 0
    RxPayloadSize = 0
    RxResponse: Response
    CurrentTxCommand = Command.NOP
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

        if "COM" in comport or "/dev" in comport:  # Windows
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
        with open(self.log_file_name, "a") as self.log:
            self.logfile_open = True
            self.log.write(f"OAE serial host log file: {self.log_file_name}\n")

    def write_log(self, text):
        if self.ConsolePrint:
            print(text)
        t2 = text + "\n"
        if self.log:
            self.log.write(t2)
            self.log.flush()  # Ensure data is written to disk immediately

    def closeLog(self):
        if self.log:
            self.log.close()

    def save_audio_buffer(self, AudioBuffer, filename_prefix="Audio_buffer"):
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"logs\\{filename_prefix}_{current_time}.txt"
        self.write_log(f"\tSaving audio buffer: {filename}")
        with open(filename, "w") as f:
            for sample in AudioBuffer:
                f.write(str(hex(sample)) + "\n")

    def serial_init(self, port, baudrate=921600, timeout=0.01):
        """Initialize serial connection."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.write_log(f"Connected to {port} at {baudrate} baud.")

        except serial.SerialException as e:
            self.write_log(f"Error: Unable to open port {port} - {e}")
            sys.exit(1)

        # Python thread switch interval defaults to 5msec.
        sys.setswitchinterval(0.001)
        threadSwitchTime = sys.getswitchinterval()
        self.write_log(f"threadSwitchTime: {threadSwitchTime}")

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
                    # if len(byte_data) > 0:
                    unsigned_byte = byte_data[i]
                    # unsigned_byte = int.from_bytes(byte_data, byteorder='big', signed=False) & 0xFF
                    # self.write_log(f"read_from_port: {unsigned_byte.hex()}")
                    self.RxQ.append(unsigned_byte)

            while len(self.RxQ) > 0:
                self.build_rx_packet()
                if self.ValidRXPacket:
                    self.RxPacketTimes.append(time.perf_counter())
                    self.process_rx_response()

    def build_rx_packet(self):
        if len(self.RxQ) == 0:
            return
        elif self.ValidRXPacket:
            return  # Wait until the current packet has been processed.
        else:
            rx_byte = self.RxQ.pop(0)
            # self.write_log(f"pop: {hex(rx_byte)} RxPacketIndex: {self.RxPacketIndex} ")
            if (self.RxPacketIndex == 0) & (rx_byte & 0xFF == 0x7E):
                self.RxChecksum = rx_byte
                # self.write_log(f"\tPacket Header: {hex(rx_byte)}  computed checksum: {self.RxChecksum}")
                self.RspPktHeaderTime = time.perf_counter()
                self.RxPacketIndex = 1
            elif self.RxPacketIndex == 1:
                # self.write_log(f"\tPacket command: {rx_byte} type: {type(rx_byte)}")
                self.RxResponse = Response(rx_byte)
                self.RxChecksum += rx_byte
                # self.write_log(f"\tRxResponse: {self.RxResponse}")
                self.RxPacketIndex += 1
            elif self.RxPacketIndex == 2:
                self.RxPayloadSize = rx_byte
                self.RxChecksum += rx_byte
                # if self.RxPayloadSize > 0:
                #    self.write_log(f"\tRxPayloadSize: {self.RxPayloadSize}  computed checksum: {self.RxChecksum}")
                self.RxPacketIndex += 1
            elif (
                (self.RxPayloadSize > 0)
                & (self.RxPacketIndex > 2)
                & (self.RxPacketIndex < (self.RxPayloadSize + 3))
            ):
                if self.RxPacketIndex == 3:
                    self.RxPayload.clear()
                    self.RxPayload_u8.clear()

                if (
                    self.RxResponse == Response.U8
                    or self.RxResponse == Response.BUF_START
                    or self.RxResponse == Response.BUF
                    or self.RxResponse == Response.BUF_END
                ):
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
                    self.write_log(
                        f"Received invalid packet: RxResponse: {self.RxResponse} RxPayloadSize: {self.RxPayloadSize} computed checksum: {self.RxChecksum}"
                    )
                    self.RxPacketIndex = 0
            #    host_logger.debug(f"\tRx invalid byte: {hex(rx_byte)} {chr(rx_byte)}")

    def process_rx_buffer_payload(self):
        if self.Buffer_SamplesReceived != U24_SAMPLES_PER_PACKET:
            self.write_log(
                f"\tErr: rx_audio_buffer_payload: RxPayloadSize: {self.RxPayloadSize} Buffer_SamplesReceived: {self.Buffer_SamplesReceived} U24_SAMPLES_PER_PACKET: {U24_SAMPLES_PER_PACKET}"
            )

        for i in range(self.Buffer_SamplesReceived):
            AudioSample = (
                (self.RxPayload_u8[i * BYTES_PER_U24] << 16)
                | (self.RxPayload_u8[i * BYTES_PER_U24 + 1] << 8)
                | self.RxPayload_u8[i * BYTES_PER_U24 + 2]
            )
            self.RxAudioBuffer.append(AudioSample)

    def process_rx_response(self):
        self.RspTime = time.perf_counter()
        self.RoundTripTime = int((self.RspTime - self.CmdTime) * 1e6)

        if self.RxPayloadSize > 0:
            if self.RxResponse == Response.U32:
                data_u32 = self.RxPayload_u8[0]
                self.RxData_u8[0] = self.RxPayload_u8[0]
                for j in range(3):
                    data_u32 = data_u32 << 8
                    data_u32 = data_u32 | self.RxPayload_u8[j + 1]
                    self.RxData_u8[j + 1] = self.RxPayload_u8[j + 1]
                self.RxDataValid = True
                if not self.RxSilent:
                    self.write_log(
                        f"\t{self.RxResponse} Payload: {hex(data_u32)} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec"
                    )
            elif self.RxResponse == Response.U8:
                Data = self.RxPayload_u8[0]
                self.write_log(
                    f"\t{self.RxResponse} Data: {hex(Data)} PktRxTime: {self.PktRxTime} sec"
                )
            elif self.RxResponse == Response.EVENT:
                EventNumber = self.RxPayload_u8[0]
                self.write_log(
                    f"\t{self.RxResponse} Event Number: {EventNumber} PktRxTime: {self.PktRxTime} sec"
                )
            elif self.RxResponse == Response.BUF_START:
                self.Buffer_PacketCount = 1
                self.RxAudioBufferStartTime = time.perf_counter()
                self.RxAudioBuffer = []
                self.Buffer_SamplesReceived = int(
                    (self.RxPayloadSize - 1) / BYTES_PER_U24
                )
                if not self.RxSilent:
                    self.write_log(
                        f"\t{self.RxResponse} # Buffer_SamplesReceived: {self.Buffer_SamplesReceived} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec"
                    )

                self.Buffer_TotalSamplesReceived = self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()

            elif self.RxResponse == Response.BUF:
                self.Buffer_PacketCount += 1
                self.Buffer_SamplesReceived = int(
                    (self.RxPayloadSize - 1) / BYTES_PER_U24
                )
                self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()

            elif self.RxResponse == Response.BUF_END:
                self.Buffer_PacketCount += 1
                self.Buffer_SamplesReceived = int(
                    (self.RxPayloadSize - 1) / BYTES_PER_U24
                )
                self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                self.process_rx_buffer_payload()
                if not self.RxSilent:
                    elapsedTime = time.perf_counter() - self.RxAudioBufferStartTime
                    self.write_log(
                        f"\t{self.RxResponse} Buffer_PacketCount: {self.Buffer_PacketCount} # samples: {self.Buffer_TotalSamplesReceived} RoundTripTime: {self.RoundTripTime} usec elapsedTime: {elapsedTime} sec"
                    )
                    if False:  # debug packet timing
                        for i in range(len(self.RxPacketTimes)):
                            if i == 0:
                                self.write_log(
                                    f"\t\tpacket {i} time: {self.RxPacketTimes[i]} delay: {self.RxPacketTimes[0] - self.CmdTime}"
                                )
                            else:
                                self.write_log(
                                    f"\t\tpacket {i} time: {self.RxPacketTimes[i]} delay: {self.RxPacketTimes[i] - self.RxPacketTimes[i - 1]}"
                                )
                self.save_audio_buffer(self.RxAudioBuffer)

            else:
                if self.RxResponse == Response.ERR and self.TxCommandsActive > 0:
                    self.TxCommandsActive -= 1

                payloadStr = "".join(self.RxPayload)
                self.write_log(f"\t{self.RxResponse} {payloadStr}")
                self.write_log(
                    f"\t\tRoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec"
                )
        else:
            if self.RxResponse == Response.ACK and self.TxCommandsActive > 0:
                self.TxCommandsActive -= 1

            if self.CurrentTxCommand != Command.BUF:
                self.write_log(
                    f"\t{self.RxResponse} no payload. RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} sec "
                )

        self.ValidRXPacket = False

    # send a packet as a byte array:
    def send_TxPacket(self, TxCommand, TxPayload):
        TxPacket = []
        Checksum = 0
        self.CurrentTxCommand = TxCommand
        if TxCommand != Command.BUF:
            self.write_log(f"send_TxPacket: {TxCommand}")
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
        for sampleCount, _ in enumerate(range(U24_SAMPLES_PER_PACKET)):
            sample = TxAudioBuffer[SampleIndexOffset + sampleCount]
            TxPayload.append((sample >> 16) & 0xFF)
            TxPayload.append((sample >> 8) & 0xFF)
            TxPayload.append(sample & 0xFF)

        self.send_TxPacket(Buf_Command, TxPayload)

    def upload_test_buffer(self):
        AudioPacketsPerBuffer = int(SERIAL_BUFFER_MAX_SIZE / U24_SAMPLES_PER_PACKET)
        TxAudioBuf = []
        for i in range(AudioPacketsPerBuffer):
            for j in range(U24_SAMPLES_PER_PACKET):
                TxAudioBuf.append(
                    (i & 0xFF) | ((i * U24_SAMPLES_PER_PACKET + j) & 0xFFFF) << 8
                )

        print(
            f"upload_test_buffer AudioPacketsPerBuffer: {AudioPacketsPerBuffer} TxAudioBuf[4095]: {hex(TxAudioBuf[4095])}"
        )
        self.TxCommandsActive = 0

        SampleIndex = 0
        TxAudioBufferStartTime = time.perf_counter()
        self.write_log(f"\tCMD_BUF_START time: {TxAudioBufferStartTime}")
        self.upload_buffer_packet(Command.BUF_START, TxAudioBuf, SampleIndex)
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        for _ in range(AudioPacketsPerBuffer - 2):
            while self.TxCommandsActive > 5:
                elapsedTime = time.perf_counter() - TxAudioBufferStartTime
                if elapsedTime > 2.0:
                    self.write_log(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                    break
                else:
                    sleep(0.001)

            self.upload_buffer_packet(Command.BUF, TxAudioBuf, SampleIndex)
            self.TxCommandsActive += 1
            SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                self.write_log(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)

        self.upload_buffer_packet(Command.BUF_END, TxAudioBuf, SampleIndex)
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                self.write_log(f"\tErr: CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)

        self.write_log(
            f"\tCMD_BUF_END SampleIndex: {SampleIndex} elapsedTime: {elapsedTime}"
        )

    def i2c_rd(self, device_addr, device_register_addr):
        TxPayload = []
        TxPayload.append(device_addr)
        TxPayload.append(device_register_addr)
        self.send_TxPacket(Command.I2C_RD, TxPayload)

    def i2c_wr(self, device_addr, device_register_addr, i2c_wr_data):
        TxPayload = []
        TxPayload.append(device_addr)
        TxPayload.append(device_register_addr)
        TxPayload.append(i2c_wr_data)
        self.send_TxPacket(Command.I2C_WR, TxPayload)

    def command_response(self, command, TxPayload=None):
        EmptyPayload = []
        if command == Command.BUF_REQ:
            TxPayload = []
            TxPayload.append(BUF_TYPE_U24)
            self.RxPacketTimes = []
            self.send_TxPacket(Command.BUF_REQ, TxPayload)
            self.write_log(f"\tCMD_BUF_REQ time: {time.perf_counter()}")
        elif command == Command.BUF_START:
            self.upload_test_buffer()
        else:
            if TxPayload is None:
                self.send_TxPacket(command, EmptyPayload)
            else:
                self.send_TxPacket(command, TxPayload)

    def print_menu(self):
        self.write_log("Menu: ")
        self.write_log("\t1) \tCMD_PING ")
        self.write_log("\t2) \tCMD_STATUS ")
        self.write_log("\t3) \tCMD_BUF Upload (host test pattern)")
        self.write_log("\t4) \tCMD_BUF Request 0 (oae test pattern)")
        self.write_log("\t5) \tCMD_BUF Request 1 (current oae buffer)")
        self.write_log("\t6) \tCMD_STOP ")
        self.write_log("\t7) \tCMD_I2C_RD ")
        self.write_log("\t8) \tCMD_I2C_WR ")
        self.write_log("\t? or h) Print this menu")
        self.write_log("\tq) \tQuit")

    def serial_user_interface(self):
        self.print_menu()
        while True:
            user_input = input("")
            if len(user_input) == 0:
                continue
            elif user_input[0] == "1":
                self.command_response(Command.PING)
            elif user_input[0] == "2":
                self.command_response(Command.STATUS)
            elif user_input[0] == "3":
                self.command_response(Command.BUF_START)
            elif user_input[0] == "4":
                BufferNum = 0
                TxPayload = []
                TxPayload.append(BufferNum)
                self.command_response(Command.BUF_REQ, TxPayload)
            elif user_input[0] == "5":
                BufferNum = 1
                TxPayload = []
                TxPayload.append(BufferNum)
                self.command_response(Command.BUF_REQ, TxPayload)
            elif user_input[0] == "6":
                CommandNum = 0
                TxPayload = []
                TxPayload.append(CommandNum)
                self.command_response(Command.STOP, TxPayload)
            elif user_input[0] == "7":
                self.i2c_rd(device_addr=0x9C, device_register_addr=0x70)
            elif user_input[0] == "8":
                self.i2c_wr(
                    device_addr=0x9C, device_register_addr=0x70, i2c_wr_data=0x75
                )
            elif user_input[0] == "h" or user_input[0] == "?":
                self.print_menu()
            elif user_input[0] == "q":
                return
            else:
                self.write_log(f"Invalid command: {user_input}")


def find_default_port(device_name: str) -> str:
    """Find the default port based on a device name.

    Args:
      device_name (str): The default name of a connected device

    Returns:
      The path to the port of the connected device.
    """
    try:
        default_port = str(
            next(
                port.device
                for port in port_list.comports()
                if port.description == device_name
            )
        )
        return default_port
    except StopIteration:
        print(f"Default device {device_name} not available. Alternative serial ports:")

        ports = port_list.comports()
        if not ports:
            print("No serial ports found")
        else:
            for port in ports:
                print(f"- {port.device} ({port.description})")
        exit()


def main():
    parser = argparse.ArgumentParser(
        prog="OAE serial protocol version 1.3",
        description="Usage: uv run oae_serial_host <COM Port>",
    )

    parser.add_argument("--port")
    args = parser.parse_args()

    if not args.port:
        args.port=find_default_port(OAE_DEVICE_NAME)

    oae = oae_serial_host(args.port)
    oae.serial_user_interface()
    oae.closeLog()


if __name__ == "__main__":
    main()
