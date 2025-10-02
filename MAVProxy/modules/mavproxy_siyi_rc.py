# siyi_rc.py  -- MAVProxy module for SIYI-SDK RC data over serial

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import serial, struct, threading, time

STX = b'\x55\x66'
CTRL_NEED_ACK = b'\x00'
CTRL_ACK_PACK = b'\x01'
START_SEQ = b'\x00\x00'

CMD_ID_REQUEST_SYSTEM_SETTING = b'\x16'
CMD_ID_REQUEST_HW_ID = b'\x40'
CMD_ID_REQUEST_CHANNEL_DATA = b'\x42'
CMD_ID_REQUEST_DATALINK_STATUS = b'\x43'
CMD_ID_REQUEST_IMG_STATUS = b'\x44'
CMD_ID_REQUEST_FW_VERSION = b'\x47'
CMD_ID_REQUEST_ALL_CHANNEL_MAP = b'\x48'
CMD_ID_REQUEST_CHANNEL_MAP = b'\x49'
CMD_ID_REQUEST_ALL_CHANNEL_REVERSE = b'\x4B'
CMD_ID_REQUEST_CHANNEL_REVERSE = b'\x4C'

CMD_ID_SEND_CHANNEL_MAP_TO_GROUND = b'\x4A'
CMD_ID_SEND_SYSTEM_SETTING_TO_GROUND = b'\x17'
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
        self.add_command('siyiserial', self.cmd_serial, 'SIYI remote serial control')
        self.add_command('siyirc', self.cmd_rc_stream, 'SIYI remote rc streaming')

        self.serial_settings = mp_settings.MPSettings(
            [ ('port', str, '/dev/ttyACM0'),
              ('baudrate', int, 57600),
              ('timeout', int, 500),
              ('verbose', bool, False)]
            )
        self.add_completion_function('(SERIALSETTING)', self.serial_settings.completion)
        self.connected = False

        self.ser = None
        self._running = True
        self._rx_thread = threading.Thread(target=self.reader, daemon=True)
        self._rx_thread.start()

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

    def request_rc_stream(self, rate='4'):
        options = OUTPUT_FREQUENCY.get(rate)
        if options is not None:
            self.send_request(CMD_ID_REQUEST_CHANNEL_DATA, struct.pack('<B', options))
            print(f"Request RC stream: {rate} Hz")
            return True
        return False

    # ------------------------------------------------------------------
    def reader(self):
        """Background thread: read & parse SIYI packets"""
        if not self.ser:
            # todo:
            return
        while self._running:
            try:
                stx = self.ser.read(2)
                if stx != STX:
                    continue
                length_b = self.ser.read(1)
                ptype_b  = self.ser.read(1)
                if not length_b or not ptype_b:
                    continue

                plen = length_b[0]
                ptype = ptype_b[0]
                payload = self.ser.read(plen)
                crc = self.ser.read(1)
                # todo: parse all data
                print(payload)
            except Exception as e:
                self.console.error(f"siyi_rc read error: {e}")

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
                port=self.serial_settings.port,
                baudrate=self.serial_settings.baudrate,
                timeout=self.serial_settings.timeout,
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

    def cmd_serial(self, args):
        '''serial control commands'''
        usage = "Usage: serial <status|set|connect|close|>"
        if len(args) < 1:
            print(usage)
            return
        if args[0] == "status":
            self.serial_status()
        elif args[0] == "set":
            self.serial_settings.command(args[1:])
        elif args[0] == "close":
            self.serial_close()
        elif args[0] == "connect":
            self.serial_connect()
        else:
            print(usage)

    def cmd_rc_stream(self, args):
        '''rc streaming commands'''
        usage = "Usage: siyirc <start|stop>"
        if len(args) < 1:
            print(usage)
            return
        elif not self.ser:
            print("Serial port is not opened")
            return

        if args[0] == "start":
            if len(args) > 1:
                if not self.request_rc_stream(rate=args[1]):
                    print("Usage: siyi_rc start <FREQUENCY> \n Available FREQUENCY: ", end=' ')
                    print(*list(OUTPUT_FREQUENCY.keys()), sep=", ", end=" Hz.\n")
                    return
        elif args[0] == "stop":
            self.request_rc_stream(rate_options=0)
        else:
            print(usage)

def init(mpstate):
    return SiyiRCModule(mpstate)
