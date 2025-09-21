# siyi_rc.py  -- MAVProxy module for SIYI-SDK RC data over serial

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from pymavlink import mavutil
import serial
import threading

STX = b'\x55\x66'
CTRL_NEED_ACK = b'\x00'
CTRL_ACK_PACK = b'\x01'
START_SEQ = b'\x00\x00'

CMD_ID_REQUEST_SYSTEM_SETTING = 0x16
CMD_ID_REQUEST_HW_ID = 0x40
CMD_ID_REQUEST_CHANNEL_DATA = 0x42
CMD_ID_REQUEST_DATALINK_STATUS = 0x43
CMD_ID_REQUEST_IMG_STATUS = 0x44
CMD_ID_REQUEST_FW_VERSION = 0x47
CMD_ID_REQUEST_ALL_CHANNEL_MAP = 0x48
CMD_ID_REQUEST_CHANNEL_MAP = 0x49
CMD_ID_REQUEST_ALL_CHANNEL_REVERSE = 0x4B
CMD_ID_REQUEST_CHANNEL_REVERSE = 0x4C

CMD_ID_SEND_CHANNEL_MAP_TO_GROUND = 0x4A
CMD_ID_SEND_SYSTEM_SETTING_TO_GROUND = 0x17
CMD_ID_SEND_CHANNEL_REVERSE_TO_GROUND = 0x4D

class SiyiRCModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super(SiyiRCModule, self).__init__(mpstate, "siyi_rc", "SIYI RC from SDK over serial")
        self.add_command('siyi_serial', self.cmd_serial, 'SIYI remote serial control')

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

    # ------------------------------------------------------------------
    def reader(self):
        """Background thread: read & parse SIYI packets"""
        if not self.ser:
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

def init(mpstate):
    return SiyiRCModule(mpstate)
