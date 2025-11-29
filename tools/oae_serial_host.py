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
import logging
import os
import sys
import threading
import time
from enum import IntEnum
from time import sleep

import numpy as np
import serial
import serial.tools.list_ports as port_list

OAE_DEVICE_NAME = "Global Health OAE Device"  # defined in usbd_desc.c, used to automatically connect

LOGGING_LEVEL = logging.DEBUG
host_logger = logging.getLogger("host")
device_logger = logging.getLogger("device")

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
    RESET = 1  # No payload, RESET response expected
    PING = 2  # Ping, no payload, RSP_PING response expected
    STATUS = (
        3  # Request status from OAE, no payload, multiple RSP_TEXT responses expected
    )
    BUF_REQ = 4  # Payload: 1 byte: BUF_TYPE
    BUF_START = 5  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer. RSP_ACK or RSP_ERR response expected
    BUF = 6  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. No response expected (there will be 62 of these packets in a 4096 sample buffer)
    BUF_END = 7  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer. RSP_ACK or RSP_ERR response expected
    I2C_RD = 8  # Payload: 2 bytes: U8 I2C device address, U8 I2C register address, RSP_U8 response expected (I2C read data)
    I2C_WR = 9  # Payload: 3 bytes: U8 I2C device address, U8 I2C register address, U8 I2C write data, RSP_ACK or RSP_ERR response expected
    STOP = 10  # Payload: 1 byte: U8, which command to stop, RSP_ACK or RSP_ERR response expected
    OK = 11  # No payload
    OAE_TEST = 12  # Run the OAE test once (does not require a stop command)


class Response(IntEnum):
    """
    OAE device responses.

    Responses flow from the OAE device to the host computer.
    """

    def __str__(self):
        return "RSP_" + self.name

    HEARTBEAT = 0  # Heartbeat response, no payload
    PING = 1  # Ping response, no payload
    ACK = 2  # No payload
    NAK = 3  # No payload
    ERR = 4  # Payload: Up to 250 bytes, text string
    TEXT = 5  # Payload: Up to 250 bytes, text string
    BUF_START = 6  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. First packet of the buffer
    BUF = 7  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data.
    BUF_END = 8  # Payload: byte 0: BUF_TYPE, bytes 1 to N: buffer data. Last packet of the buffer
    U8 = 9  # Payload: 1 byte:  U8
    U32 = 10  # Payload: 4 bytes: U32
    EVENT = 11  # Payload: 1 byte:  U8 (event number)
    INVALID = 12  # No payload (Command from host was not recognized)
    LOG_DEBUG = 13  # Payload: Up to 250 bytes, text string
    LOG_INFO = 14  # Payload: Up to 250 bytes, text string
    LOG_WARNING = 15  # Payload: Up to 250 bytes, text string
    LOG_ERROR = 16  # Payload: Up to 250 bytes, text string
    LOG_CRITICAL = 17  # Payload: Up to 250 bytes, text string


class oae_serial_host:
    # connection management
    kill_receive_thread = threading.Event()

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
        self.comport = comport
        self.serial_init(self.comport)
        self.reset_device()

    def save_audio_buffer(self, AudioBuffer, filename_prefix="Audio_buffer"):
        current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = (
            f"logs/{filename_prefix}_{current_time}.txt"
        )
        host_logger.info(f"\tSaving audio buffer: {filename}")
        with open(filename, "w") as f:
            for sample in AudioBuffer:
                f.write(str(hex(sample)) + "\n")

    def serial_init(self, port, baudrate=921600, timeout=0.01):
        """Initialize serial connection."""
        host_logger.info("Initializing serial port")
        serial_exception = None
        current_time = time.monotonic()
        while (current_time - time.monotonic() < timeout):
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1e-6)
                host_logger.info(f"Connected to {port} at {baudrate} baud")
                serial_exception = None
                break
            except serial.SerialException as error:
                serial_exception = error
                time.sleep(1e-6)

        if serial_exception:
            host_logger.error(f"Unable to open port {port} - {serial_exception}")

        # Python thread switch interval defaults to 5msec.
        sys.setswitchinterval(0.001)
        threadSwitchTime = sys.getswitchinterval()
        host_logger.debug(f"threadSwitchTime: {threadSwitchTime}")

        # Start a thread to read from the serial port
        self.kill_receive_thread.clear()
        self.receive_thread = threading.Thread(target=self.read_from_port)
        self.receive_thread.daemon = True
        self.receive_thread.start()

    # Function to read from the serial port and process incoming data
    #   This is run in a thread
    def read_from_port(self):
        self.connection_checked = False
        while not self.kill_receive_thread.is_set():
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    # Read up to 64 bytes
                    byte_data = self.ser.read(64)
                    for unsigned_byte in byte_data:
                        self.RxQ.append(unsigned_byte)

                while len(self.RxQ) > 0:
                    self.build_rx_packet()
                    if self.ValidRXPacket:
                        self.RxPacketTimes.append(time.perf_counter())
                        self.process_rx_response()
            except OSError:
                print("Unable to check receive buffer, device may be disconnected.")
                raise RuntimeError("Unable to check receive buffer, device may be disconnected.") from None
            time.sleep(1e-6)

    def build_rx_packet(self):
        if len(self.RxQ) == 0:
            return
        elif self.ValidRXPacket:
            return  # Wait until the current packet has been processed.
        else:
            rx_byte = self.RxQ.pop(0)
            if (self.RxPacketIndex == 0) & (rx_byte & 0xFF == 0x7E):
                self.RxChecksum = rx_byte
                self.RspPktHeaderTime = time.perf_counter()
                self.RxPacketIndex = 1
            elif self.RxPacketIndex == 1:
                self.RxResponse = Response(rx_byte)
                self.RxChecksum += rx_byte
                self.RxPacketIndex += 1
            elif self.RxPacketIndex == 2:
                self.RxPayloadSize = rx_byte
                self.RxChecksum += rx_byte
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
                    host_logger.warning(
                        f"Received invalid packet: RxResponse: {self.RxResponse} RxPayloadSize: {self.RxPayloadSize} computed checksum: {self.RxChecksum}"
                    )
                    self.RxPacketIndex = 0

    def process_rx_buffer_payload(self):
        if self.Buffer_SamplesReceived != U24_SAMPLES_PER_PACKET:
            host_logger.error(
                f"rx_audio_buffer_payload: RxPayloadSize: {self.RxPayloadSize} Buffer_SamplesReceived: {self.Buffer_SamplesReceived} U24_SAMPLES_PER_PACKET: {U24_SAMPLES_PER_PACKET}"
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
        self.RoundTripTime = round(((self.RspTime - self.CmdTime) * 1e3),2)

        if self.RxPayloadSize > 0:
            match self.RxResponse:
                case Response.U32:
                    data_u32 = self.RxPayload_u8[0]
                    self.RxData_u8[0] = self.RxPayload_u8[0]
                    for j in range(3):
                        data_u32 = data_u32 << 8
                        data_u32 = data_u32 | self.RxPayload_u8[j + 1]
                        self.RxData_u8[j + 1] = self.RxPayload_u8[j + 1]
                    self.RxDataValid = True
                    if not self.RxSilent:
                        host_logger.debug(
                            f"{self.RxResponse} Payload: {hex(data_u32)} RoundTripTime: {self.RoundTripTime}ms"
                        )
                case Response.U8:
                    Data = self.RxPayload_u8[0]
                    host_logger.debug(
                        f"{self.RxResponse} Data: {hex(Data)}"
                    )
                case Response.EVENT:
                    EventNumber = self.RxPayload_u8[0]
                    host_logger.debug(
                        f"{self.RxResponse} Event Number: {EventNumber}"
                    )
                case Response.BUF_START:
                    self.Buffer_PacketCount = 1
                    self.RxAudioBufferStartTime = time.perf_counter()
                    self.RxAudioBuffer = []
                    self.Buffer_SamplesReceived = int(
                        (self.RxPayloadSize - 1) / BYTES_PER_U24
                    )
                    if not self.RxSilent:
                        host_logger.debug(
                            f"{self.RxResponse} # Buffer_SamplesReceived: {self.Buffer_SamplesReceived} RoundTripTime: {self.RoundTripTime}ms"
                        )

                    self.Buffer_TotalSamplesReceived = self.Buffer_SamplesReceived
                    self.process_rx_buffer_payload()

                case Response.BUF:
                    self.Buffer_PacketCount += 1
                    self.Buffer_SamplesReceived = int(
                        (self.RxPayloadSize - 1) / BYTES_PER_U24
                    )
                    self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                    self.process_rx_buffer_payload()

                case Response.BUF_END:
                    self.Buffer_PacketCount += 1
                    self.Buffer_SamplesReceived = int(
                        (self.RxPayloadSize - 1) / BYTES_PER_U24
                    )
                    self.Buffer_TotalSamplesReceived += self.Buffer_SamplesReceived
                    self.process_rx_buffer_payload()
                    if not self.RxSilent:
                        elapsedTime = time.perf_counter() - self.RxAudioBufferStartTime
                        host_logger.debug(
                            f"{self.RxResponse} Buffer_PacketCount: {self.Buffer_PacketCount} # samples: {self.Buffer_TotalSamplesReceived} RoundTripTime: {self.RoundTripTime}ms elapsedTime: {elapsedTime} sec"
                        )
                    self.save_audio_buffer(self.RxAudioBuffer)
                case Response.LOG_DEBUG:
                    device_logger.debug("".join(self.RxPayload))
                case Response.LOG_INFO:
                    device_logger.info("".join(self.RxPayload))
                case Response.LOG_WARNING:
                    device_logger.warning("".join(self.RxPayload))
                case Response.LOG_ERROR:
                    device_logger.error("".join(self.RxPayload))
                case Response.LOG_CRITICAL:
                    device_logger.critical("".join(self.RxPayload))
                case _:
                    if self.RxResponse == Response.ERR and self.TxCommandsActive > 0:
                        self.TxCommandsActive -= 1

                    payloadStr = "".join(self.RxPayload)
                    host_logger.debug(f"{self.RxResponse} {payloadStr}")
                    host_logger.debug(
                        f"RoundTripTime: {self.RoundTripTime}ms"
                    )
        else:
            if self.RxResponse == Response.ACK and self.TxCommandsActive > 0:
                self.TxCommandsActive -= 1

            if self.CurrentTxCommand != Command.BUF:
                host_logger.debug(
                    f"{self.RxResponse} no payload. RoundTripTime: {self.RoundTripTime}ms"
                )

        self.ValidRXPacket = False

    # send a packet as a byte array:
    def send_TxPacket(self, TxCommand, TxPayload):
        TxPacket = []
        Checksum = 0
        self.CurrentTxCommand = TxCommand
        if TxCommand != Command.BUF:
            host_logger.debug(f"send_TxPacket: {TxCommand}")
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
        host_logger.debug(f"CMD_BUF_START time: {TxAudioBufferStartTime}")
        self.upload_buffer_packet(Command.BUF_START, TxAudioBuf, SampleIndex)
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        for _ in range(AudioPacketsPerBuffer - 2):
            while self.TxCommandsActive > 5:
                elapsedTime = time.perf_counter() - TxAudioBufferStartTime
                if elapsedTime > 2.0:
                    host_logger.error(f"CMD_BUF Timeout: {elapsedTime}")
                    break
                else:
                    sleep(0.001)

            self.upload_buffer_packet(Command.BUF, TxAudioBuf, SampleIndex)
            self.TxCommandsActive += 1
            SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                host_logger.error(f"CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)

        self.upload_buffer_packet(Command.BUF_END, TxAudioBuf, SampleIndex)
        self.TxCommandsActive += 1
        SampleIndex += U24_SAMPLES_PER_PACKET

        while self.TxCommandsActive > 0:
            elapsedTime = time.perf_counter() - TxAudioBufferStartTime
            if elapsedTime > 2.0:
                host_logger.error(f"CMD_BUF Timeout: {elapsedTime}")
                break
            else:
                sleep(0.001)

        host_logger.debug(
            f"CMD_BUF_END SampleIndex: {SampleIndex} elapsedTime: {elapsedTime}"
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
            host_logger.debug(f"CMD_BUF_REQ time: {time.perf_counter()}")
        elif command == Command.BUF_START:
            self.upload_test_buffer()
        else:
            if TxPayload is None:
                self.send_TxPacket(command, EmptyPayload)
            else:
                self.send_TxPacket(command, TxPayload)

    def reset_device(self) -> None:
        """"""
        host_logger.info("Resetting device")
        self.command_response(Command.RESET)
        self.kill_receive_thread.set()
        self.receive_thread.join()
        
        # close serial port
        host_logger.info("Closing serial port")
        self.ser.close()

        # reopen serial port
        time.sleep(1)
        self.serial_init(port=self.comport, timeout=5)

    def print_menu(self):
        print("# Menu: ")
        print("# \t0) \tCMD_RESET ")
        print("# \t1) \tCMD_PING ")
        print("# \t2) \tCMD_STATUS ")
        print("# \t3) \tCMD_BUF Upload (host test pattern)")
        print("# \t4) \tCMD_BUF Request 0 (oae test pattern)")
        print("# \t5) \tCMD_BUF Request 1 (current oae buffer)")
        print("# \t6) \tCMD_STOP ")
        print("# \t7) \tCMD_I2C_RD ")
        print("# \t8) \tCMD_I2C_WR ")
        print("# \t? or h) Print this menu")
        print("# \tq) \tQuit")

    def serial_user_interface(self):
        self.print_menu()
        while True:
            user_input = input("")
            if len(user_input) == 0:
                continue
            elif user_input[0] == "0":
                self.reset_device()
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
                host_logger.warning(f"Invalid command: {user_input}")


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
        host_logger.error(
            f"Default device {device_name} not available. Alternative serial ports:"
        )

        ports = port_list.comports()
        if not ports:
            host_logger.error("No serial ports found")
        else:
            for port in ports:
                host_logger.info(f"- {port.device} ({port.description})")
        exit()


def main():
    # host and device logging configuration
    os.makedirs("tools/logs", exist_ok=True)
    formatter = logging.Formatter(
        fmt="%(asctime)s %(name)-6s %(levelname)-8s %(filename)s:%(lineno)d - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler = logging.FileHandler(f"tools/logs/{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    file_handler.setFormatter(formatter)
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setFormatter(formatter)
    host_logger.addHandler(stdout_handler)
    host_logger.addHandler(file_handler)
    host_logger.setLevel(level=LOGGING_LEVEL)
    device_logger.addHandler(stdout_handler)
    device_logger.addHandler(file_handler)
    device_logger.setLevel(level=LOGGING_LEVEL)

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
