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
import serial
import serial.tools.list_ports as port_list
import numpy as np

VERSION = "v1.1"

PACKET_HEADER_BYTE = 0x7E
CMD_NOP     = 0
CMD_PING    = 1
CMD_STATUS  = 2
CMD_ADC_BUF_REQ 	= 3
CMD_ADC_BUF_START 	= 4
CMD_ADC_BUF 		= 5
CMD_ADC_BUF_END		= 6
CMD_OK              = 7

RSP_PING    = 101
RSP_ACK     = 102
RSP_NAK     = 103
RSP_ERR     = 104
RSP_TEXT    = 105
RSP_ADC_BUF_START 	= 106
RSP_ADC_BUF 		= 107
RSP_ADC_BUF_END		= 108
RSP_U8 		        = 109
RSP_U32 			= 110
RSP_INVALID 		= 111

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
    RxChecksum = 0
    ValidRXPacket = False
    StartTime = 0
    CmdTime = 0
    RspTime = 0
    ADC_PacketCount = 0
    ADC_SamplesReceived = 0
    ADC_RxBuf = []

    def __init__(self, comport):
        self.logfile_open = False
        self.open_logfile()
        self.StartTime = time.perf_counter()

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

    def writeLog(self, text, ConsolePrint = True):
        if ConsolePrint:
            print(text)
        t2 = text + '\n'
        if self.log:
            self.log.write(t2)

    def closeLog(self):
        if self.log:
            self.log.close()

    def serial_init(self, port, baudrate=115200, timeout=1):
        """Initialize serial connection."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Connected to {port} at {baudrate} baud.")

        except serial.SerialException as e:
            print(f"Error: Unable to open port {port} - {e}")
            sys.exit(1)

        # Python thread switch interval defaults to 5msec.
        sys.setswitchinterval(.001)
        threadSwitchTime = sys.getswitchinterval()
        print(f"threadSwitchTime: {threadSwitchTime}")

        # Start a thread to read from the serial port
        thread = threading.Thread(target=self.read_from_port)
        thread.daemon = True
        thread.start()

    # Function to read from the serial port and process incoming data
    #   This is run in a thread
    def read_from_port(self):
        while True:
            if self.ser._port_handle and self.ser.in_waiting > 0:
                try:
                    # Read a single byte
                    byte_data = self.ser.read(1)
                    unsigned_byte = int.from_bytes(byte_data, byteorder='big', signed=False) & 0xFF
                    #print(f"read_from_port: {unsigned_byte.hex()}")
                    self.RxQ.append(unsigned_byte)

                except Exception as e:
                    print(f"Error: {e}")
            else:
                self.build_rx_packet()
                if self.ValidRXPacket:
                    self.process_rx_response()

    def command_name(self, command):
        if command == CMD_NOP:
            return "CMD_NOP"
        elif command == CMD_PING:
            return "CMD_PING"
        elif command == CMD_STATUS:
            return "CMD_STATUS"
        elif command == CMD_ADC_BUF_REQ:
            return "CMD_ADC_BUF_REQ"
        elif command == CMD_ADC_BUF_START:
            return "CMD_ADC_BUF_START"
        elif command == CMD_ADC_BUF:
            return "CMD_ADC_BUF"
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
        elif command == RSP_ADC_BUF_START:
            return "RSP_ADC_BUF_START"
        elif command == RSP_ADC_BUF:
            return "RSP_ADC_BUF"
        elif command == RSP_ADC_BUF_END:
            return "RSP_ADC_BUF_END"
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
            #print(f"pop: {hex(rx_byte)} RxPacketIndex: {self.RxPacketIndex} ")
            if (self.RxPacketIndex == 0) & (rx_byte & 0xFF == 0x7E):
                self.RxChecksum = rx_byte
                #print(f"\tPacket Header: {hex(rx_byte)}  computed checksum: {self.RxChecksum}")
                self.RspPktHeaderTime = time.perf_counter()
                self.RxPacketIndex = 1
            elif self.RxPacketIndex == 1:
                # print(f"\tPacket command: {rx_byte} type: {type(rx_byte)}")
                self.RxCommand = rx_byte
                self.RxChecksum += rx_byte
                #print(f"\tRxCommand: {self.RxCommand}")
                self.RxPacketIndex += 1
            elif self.RxPacketIndex == 2:
                self.RxPayloadSize = rx_byte
                self.RxChecksum += rx_byte
                #if self.RxPayloadSize > 0:
                #    print(f"\tRxPayloadSize: {self.RxPayloadSize}  computed checksum: {self.RxChecksum}")
                self.RxPacketIndex += 1
            elif (self.RxPayloadSize > 0) & (self.RxPacketIndex > 2) & (self.RxPacketIndex < (self.RxPayloadSize + 3)):
                if self.RxPacketIndex == 3:
                    self.RxPayload.clear()
                    self.RxPayload_u8.clear()

                if self.RxCommand == RSP_U8:
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
                    print(f"Received invalid packet: RxCommand: {self.RxCommand} RxPayloadSize: {self.RxPayloadSize} computed checksum: {self.RxChecksum}")
                    self.RxPacketIndex = 0
            #else:
            #    print(f"\tRx invalid byte: {hex(rx_byte)} {chr(rx_byte)}")

    def process_rx_response(self):
        self.RspTime = time.perf_counter()
        self.GlobalTime = int((self.RspTime - self.StartTime) * 1e6)
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
                    print(f"\t{self.command_name(self.RxCommand)} Payload: {hex(data_u32)} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} usec")
            elif self.RxCommand == RSP_ADC_BUF_START:
                self.ADC_PacketCount = 1
                if self.RxSilent == False:
                    print(f"\t{self.command_name(self.RxCommand)} # samples: {self.RxPayloadSize / 3} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} usec")
                self.ADC_SamplesReceived = self.RxPayloadSize / 3
            elif self.RxCommand == RSP_ADC_BUF:
                self.ADC_PacketCount += 1
                self.ADC_SamplesReceived += self.RxPayloadSize / 3
            elif self.RxCommand == RSP_ADC_BUF_END:
                self.ADC_PacketCount += 1
                self.ADC_SamplesReceived += self.RxPayloadSize / 3
                if self.RxSilent == False:
                    print(f"\t{self.command_name(self.RxCommand)} ADC_PacketCount: {self.ADC_PacketCount} # samples: {self.ADC_SamplesReceived} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} usec")
            else:
                payloadStr = ''.join(self.RxPayload)
                print(f"\t{self.command_name(self.RxCommand)} {payloadStr}")
                print(f"\t\tRoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} usec")
        else:
            print(f"\t{self.command_name(self.RxCommand)} RoundTripTime: {self.RoundTripTime} usec PktRxTime: {self.PktRxTime} usec ")

        self.ValidRXPacket = False

    # send a packet as a byte array:
    def send_TxPacket(self, TxCommand, TxPayload):
        TxPacket = []
        Checksum = 0
        print(f"send_TxPacket: {self.command_name(TxCommand)}")
        TxPacket.append(PACKET_HEADER_BYTE)
        Checksum += PACKET_HEADER_BYTE
        TxPacket.append(TxCommand)
        Checksum += TxCommand
        PayloadSize = len(TxPayload)
        TxPacket.append(PayloadSize)
        Checksum += PayloadSize
        if PayloadSize > 0:
            for p in TxPayload:
                TxPacket.append(p)
                Checksum += p

        Checksum = Checksum & 0xFF  # truncate to 8 bits
        TxPacket.append(Checksum)
        self.CmdTime = time.perf_counter()
        self.ser.write(TxPacket)

    def command_response(self, command):
        TxPayload = []
        if command == CMD_PING:
            self.send_TxPacket(CMD_PING, TxPayload)
        elif command == CMD_ADC_BUF_REQ:
            self.send_TxPacket(CMD_ADC_BUF_REQ, TxPayload)
        elif command == CMD_STATUS:
            self.send_TxPacket(CMD_STATUS, TxPayload)

    def print_menu(self):
        print("Menu: ")
        print("\t1) \tCMD_PING ")
        print("\t2) \tCMD_STATUS ")
        print("\t3) \tCMD_ADC_BUF_REQ ")
        print("\t? or h) Print this menu")
        print("\tq) \tQuit")

    def serial_user_interface(self):
        self.print_menu()
        while True:
            user_input = input("")
            if user_input[0] == '1':
                self.command_response(CMD_PING)
            elif user_input[0] == '2':
                self.command_response(CMD_STATUS)
            elif user_input[0] == '3':
                self.command_response(CMD_ADC_BUF_REQ)
            elif user_input[0] == 'h':
                self.print_menu()
            elif user_input[0] == '?':
                self.print_menu()
            elif user_input[0] == 'q':
                return
            else:
                print(f"Invalid command: {user_input}")

def main():

    print(f"OAE serial host version: {VERSION}")

    if len(sys.argv) != 2:
        print("\tUsage: python -m oae_serial_host <COM Port>")
        print("\tlog files are saved to the logs/ subdirectory.")

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