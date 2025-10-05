# siyi_rc.py  -- MAVProxy module for SIYI-SDK RC data over serial

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import serial, struct, threading, time

# RC channel, min pwm, and max pwm
RC_TOTAL_CH = 16
RC_MIN = 1000
RC_MAX = 2000

# For print()
RED    = '\033[31m'
GREEN  = '\033[32m'
YELLOW = '\033[33m'
BLUE   = '\033[34m'
RESET  = '\033[0m'

# Mark byte for parsing
STX           = b'\x55\x66'
CTRL_NEED_ACK = b'\x00'
CTRL_ACK_PACK = b'\x01'
START_SEQ     = b'\x00\x00'

# Size of header packet and crc in byte
STX_SIZE     = 2
CTRL_SIZE    = 1
DATALEN_SIZE = 2
SEQ_SIZE     = 2
CMD_ID_SIZE  = 1
CRC_SIZE     = 2
HEADER_SIZE  = STX_SIZE + CTRL_SIZE + DATALEN_SIZE + SEQ_SIZE + CMD_ID_SIZE + CRC_SIZE
DATA_SIZE    = 32

# Index in bytearray
STX_IDX     = 0
CTRL_IDX    = 2
DATALEN_IDX = 3
SEQ_IDX     = 5
CMD_ID_IDX  = 7
DATA_IDX    = 8

# Command-id
# Todo: Handle all cmd
CMD_ID_REQUEST_SYSTEM_SETTING      = b'\x16'
CMD_ID_REQUEST_HW_ID               = b'\x40'
CMD_ID_REQUEST_CHANNEL_DATA        = b'\x42'
CMD_ID_REQUEST_DATALINK_STATUS     = b'\x43'
CMD_ID_REQUEST_IMG_STATUS          = b'\x44'
CMD_ID_REQUEST_FW_VERSION          = b'\x47'
CMD_ID_REQUEST_ALL_CHANNEL_MAP     = b'\x48'
CMD_ID_REQUEST_CHANNEL_MAP         = b'\x49'
CMD_ID_REQUEST_ALL_CHANNEL_REVERSE = b'\x4B'
CMD_ID_REQUEST_CHANNEL_REVERSE     = b'\x4C'
CMD_ID_SEND_CHANNEL_MAP_TO_GROUND     = b'\x4A'
CMD_ID_SEND_SYSTEM_SETTING_TO_GROUND  = b'\x17'
CMD_ID_SEND_CHANNEL_REVERSE_TO_GROUND = b'\x4D'

# frequency in Hz -> map option number
OUTPUT_FREQUENCY = {
    "0"   : 0,     # OFF
    "2"   : 1,
    "4"   : 2,
    "5"   : 3,
    "10"  : 4,
    "20"  : 5,
    "50"  : 6,
    "100" : 7,
}

# CRC16 Table
CRC16_TAB = [
    0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
    0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
    0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
    0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
    0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
    0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
    0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
    0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
    0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
    0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
    0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
    0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
    0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
    0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
    0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
    0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
    0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
    0x1080,0x0101,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
    0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
    0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
    0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
    0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
    0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
    0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
    0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
    0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
    0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
    0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
    0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
    0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
    0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
    0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
]

def crc16_ccitt(data: bytes, init_val=0):
    crc = init_val
    for b in data:
        temp = (crc >> 8) & 0xFF           # high byte
        crc = ((crc << 8) ^ CRC16_TAB[b ^ temp]) & 0xFFFF
    return crc

class SiyiRCModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SiyiRCModule, self).__init__(mpstate, "siyi_rc", "SIYI RC from SDK over serial")
        self.add_command('siyirc', self.cmd_siyirc_stream, 'SIYI remote rc streaming')

        self.siyirc_settings = mp_settings.MPSettings(
            [ ('port', str, '/dev/ttyACM0'),
              ('baudrate', int, 115200),
              ('timeout', int, 5),
              ('frequency', int, 100),
              ('verbose', int, 0)]
            )
        self.add_completion_function('(SERIALSETTING)', self.siyirc_settings.completion)

        self.ser = None
        self.rc_override_value = {i: RC_MIN for i in range(RC_TOTAL_CH)}
        self.rc_read_period = mavutil.periodic_event(20)
        self.thread = threading.Thread(target=self.reader)
        self.thread.start()

    # --------------------- I/O helpers --------------------------------
    def send_request(self, pkt_type, payload=b''):
        """Send a framed request"""
        frame = bytearray(STX)
        frame.extend(CTRL_ACK_PACK)
        frame.extend(len(payload).to_bytes(2, 'little'))
        frame.extend(START_SEQ)
        frame.extend(pkt_type)
        frame.extend(payload)
        frame.extend(crc16_ccitt(frame).to_bytes(2, 'little'))
        self.ser.write(frame)

    # ------------------------------------------------------------------
    def request_rc_stream(self, rate='4'):
        options = OUTPUT_FREQUENCY.get(rate)
        if options is not None:
            self.send_request(CMD_ID_REQUEST_CHANNEL_DATA, struct.pack('<B', options))
            print(f"Request RC stream: {rate} Hz")
            return True
        return False

    # ------------------------------------------------------------------
    def parse_packet(self, pkt: bytes):
        if len(pkt) < DATA_SIZE:
            return None, None, "too short"
        if pkt[STX_IDX:STX_SIZE] != STX:
            print(pkt[STX_IDX:STX_SIZE])
            return None, None, "bad STX"

        # Print raw packet if needed
        if self.siyirc_settings.verbose > 1:
            print("Raw Packet:")
            for i, b in enumerate(pkt):
                print(RED+f"{b:02X} "+RESET, end="")
                if (i + 1) % 8 == 0:
                    print()
            print("\n")

        ctrl      = pkt[CTRL_IDX]
        data_len  = pkt[DATALEN_IDX] | (pkt[DATALEN_IDX+1] << 8)
        seq       = pkt[SEQ_IDX] | (pkt[SEQ_IDX+1] << 8)
        cmd_id    = pkt[CMD_ID_IDX]
        total_len = HEADER_SIZE + data_len

        if len(pkt) < total_len:
            return None, None, "incomplete"

        data     = pkt[DATA_IDX:DATA_IDX + data_len]
        crc_recv = struct.unpack("<H", pkt[DATA_IDX+data_len:total_len])[0]

        crc_calc = crc16_ccitt(pkt[:total_len-CRC_SIZE])
        if crc_calc != crc_recv:
            return None, None, f"CRC mismatch (calc=0x{crc_calc:04X}, recv=0x{crc_recv:04X})"

        packet_info = {
          "STX"     : f"0x{STX.hex()}",
          "CTRL_raw": ctrl,
          "need_ack": ctrl == CTRL_NEED_ACK[0],
          "ack_pack": ctrl == CTRL_ACK_PACK[0],
          "Data_len": data_len,
          "SEQ"     : seq,
          "CMD_ID"  : cmd_id,
          "CRC16"   : f"0x{crc_recv:04X}",
        }

        result_data = {}
        if data_len == DATA_SIZE:
            channels = struct.unpack("<16h", data)
            for i, v in enumerate(channels, 1):
                result_data[i] = v

        return packet_info, result_data, None

    # ------------------------------------------------------------------
    def reader(self):
        """Background thread: read & parse SIYI packets"""
        while True:
            try:
                if not self.ser:
                    continue
                data = self.ser.read(128)
                index = data.find(STX)
                info, parsed_data, err = self.parse_packet(pkt=data[index:])
                # Update RCin value
                if parsed_data:
                    self.rc_override_value = parsed_data
                # For debugging
                if self.siyirc_settings.verbose:
                    if err:
                        print(RED+f"Error: {err}"+RESET)
                    else:
                        # Print RC channels compactly
                        print("RC Channels:")
                        for ch, val in parsed_data.items():
                            print(BLUE+f"CH{ch:02}: "+RESET+GREEN+f"{val:4}"+RESET, end="   ")
                            if ch % 4 == 0:  # 4 per row
                                print()
                        # Print info packet in one line
                        print("Info Packet:")
                        for k, v in info.items():
                            print(BLUE+f"{k}: "+RESET+YELLOW+f"{v}"+RESET, end="   ")
                        print("\n")

            except Exception as e:
                self.console.error(f"siyi_rc read error: {e}")
                continue

    # ------------------------------------------------------------------
    def serial_close(self):
        try:
            self.ser.close()
            self._running = False
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            pass

    # ------------------------------------------------------------------
    def serial_connect(self):
        try:
            self.ser = serial.Serial(
                port=self.siyirc_settings.port,
                baudrate=self.siyirc_settings.baudrate,
                timeout=self.siyirc_settings.timeout,
            )
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            pass

    # ------------------------------------------------------------------
    def serial_status(self):
        if self.ser and self.ser.isOpen():
            print("The serial port is open.")
        elif self.ser:
            print("The serial port is not open (though the object was created).")
        else:
            print("The serial port object could not be created.")

    # ------------------------------------------------------------------
    def cmd_siyirc_stream(self, args):
        '''rc streaming commands'''
        usage = "Usage: siyirc <start|stop|set|status>"
        if len(args) < 1:
            print(usage)
            return

        if args[0] == "start":
            self.serial_connect()
            self.request_rc_stream(rate=self.siyirc_settings.frequency)
        elif args[0] == "stop":
            self.request_rc_stream(rate=0)
            self.serial_close()
        elif args[0] == "set":
            self.siyirc_settings.command(args[1:])
        elif args[0] == "status":
            # todo: add rc-stream status
            self.serial_status()
        else:
            print(usage)

    # ------------------------------------------------------------------
    def idle_task(self):
        if self.rc_read_period.trigger():
            if self.ser:
                if self.module('rc') is not None:
                    [self.module('rc').set_override_chan(ch-1, v) for ch, v in self.rc_override_value.items()]

def init(mpstate):
    return SiyiRCModule(mpstate)
