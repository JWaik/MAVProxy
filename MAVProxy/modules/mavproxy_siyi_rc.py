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
        frame.extend(CTRL_NEED_ACK)
        frame.append(len(payload))
        frame.extend(START_SEQ)
        frame.extend(pkt_type)
        frame.extend(payload)
        # TODO: Crc calculation
        # frame.extend(sum(frame) & b'\xFF) # crc
        # self.ser.write(frame)
        # Or, for a more formatted output with spaces:
        hex_string = " ".join([f"{byte:02x}" for byte in frame])
        print(hex_string)

    def request_rc_stream(self, rate='4'):
        options = OUTPUT_FREQUENCY.get(rate)
        print(f"option {options}")
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
