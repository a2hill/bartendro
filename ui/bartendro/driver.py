import collections
from time import sleep, localtime, time
import serial
from struct import pack, unpack
from router import pack7

__author__ = 'austinhill'

BAUD_RATE = 9600
DEFAULT_TIMEOUT = 2  # in seconds

MAX_DISPENSERS = 15
SHOT_TICKS = 20

RAW_PACKET_SIZE = 10
PACKET_SIZE = 8

PACKET_ACK_OK = 0
PACKET_CRC_FAIL = 1
PACKET_ACK_TIMEOUT = 2
PACKET_ACK_INVALID = 3
PACKET_ACK_INVALID_HEADER = 4
PACKET_ACK_HEADER_IN_PACKET = 5
PACKET_ACK_CRC_FAIL = 6

PACKET_PING = 3
PACKET_SET_MOTOR_SPEED = 4
PACKET_TICK_DISPENSE = 5
PACKET_TIME_DISPENSE = 6
PACKET_LED_OFF = 7
PACKET_LED_IDLE = 8
PACKET_LED_DISPENSE = 9
PACKET_LED_DRINK_DONE = 10
PACKET_IS_DISPENSING = 11
PACKET_LIQUID_LEVEL = 12
PACKET_UPDATE_LIQUID_LEVEL = 13
PACKET_ID_CONFLICT = 14
PACKET_LED_CLEAN = 15
PACKET_SET_CS_THRESHOLD = 16
PACKET_SAVED_TICK_COUNT = 17
PACKET_RESET_SAVED_TICK_COUNT = 18
PACKET_GET_LIQUID_THRESHOLDS = 19
PACKET_SET_LIQUID_THRESHOLDS = 20
PACKET_FLUSH_SAVED_TICK_COUNT = 21
PACKET_TICK_SPEED_DISPENSE = 22
PACKET_PATTERN_DEFINE = 23
PACKET_PATTERN_ADD_SEGMENT = 24
PACKET_PATTERN_FINISH = 25
PACKET_SET_MOTOR_DIRECTION = 26
PACKET_GET_VERSION = 27
PACKET_COMM_TEST = 0xFE

MOTOR_DIRECTION_FORWARD = 1
MOTOR_DIRECTION_BACKWARD = 0

DISPENSER_DEFAULT_VERSION = 2

DEST_BROADCAST = 0xFF


def crc16_update(crc, a):
    crc ^= a
    for i in xrange(0, 8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xA001
        else:
            crc = (crc >> 1)
    return crc


class DeviceDriver(object):
    '''This object interacts with the bartendro router controller.'''

    def __init__(self, device):
        self.device = device
        self.ser = None
        self.msg = ""
        self.ret = 0
        self.dispenser_version = DISPENSER_DEFAULT_VERSION
        self.startup_log = ""
        self.debug_levels = [200, 180, 120]

        # dispenser_ids are the ids the dispensers have been assigned. These are logical ids
        # used for dispenser communication.
        self.dispenser_id = -1

    def get_startup_log(self):
        return self.startup_log

    def get_dispenser_version(self):
        return self.dispenser_version

    def reset(self):
        """Reset the hardware. Do this if there is shit going wrong. All motors will be stopped
           and reset."""

        self.close()
        self.open()

    def set_timeout(self, timeout):
        self.ser.timeout = timeout

    def open(self):
        """Open the serial connection to the router"""

        self._clear_startup_log()

        try:
            self.ser = serial.Serial(self.device,
                                     BAUD_RATE,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     timeout=.01)
        except serial.serialutil.SerialException, e:
            self.log("Unable to ope serial connection to pump: " + str(e))

        self.log("Serial connection open")

        # This primes the communication line.
        self.ser.write(chr(170) + chr(170) + chr(170))
        sleep(.001)

        while True:
            self.ser.flushInput()
            self.ser.write("???")
            data = self.ser.read(3)
            ll = ""
            for ch in data:
                ll += "%02X " % ord(ch)
            if len(data) == 3:
                if data[0] != data[1] or data[0] != data[2]:
                    self.log("  %s -- inconsistent" % ll)
                    continue
                dev_id = ord(data[0])
                self.dispenser_id = dev_id
                self.log("  %s -- Found dispenser with pump id %02X" % (ll, dev_id))
                break
            elif len(data) > 1:
                self.log("  %s -- Did not receive 3 characters back. Trying again." % ll)
                sleep(.5)
            else:
                break

        self.set_timeout(DEFAULT_TIMEOUT)
        self.ser.write(chr(255))

        self.dispenser_version = self.query_dispenser_version(0)
        if self.dispenser_version < 0:
            self.dispenser_version = DISPENSER_DEFAULT_VERSION

        self.log("Detected dispensers version %d." % self.dispenser_version)

    def close(self):
        self.ser.close()
        self.ser = None

    def log(self, msg):
        t = localtime()
        print("%d-%d-%d %d:%02d %s" % (t.tm_year, t.tm_mon, t.tm_mday, t.tm_hour, t.tm_min, msg))

    def make_shot(self):
        self._send_packet32(0, PACKET_TICK_DISPENSE, 90)
        return True

    def ping(self, dispenser):
        return self._send_packet32(dispenser, PACKET_PING, 0)

    def start(self, dispenser):
        return self._send_packet8(dispenser, PACKET_SET_MOTOR_SPEED, 255, True)

    def set_motor_direction(self, dispenser, direction):
        return self._send_packet8(dispenser, PACKET_SET_MOTOR_DIRECTION, direction)

    def stop(self, dispenser):
        return self._send_packet8(dispenser, PACKET_SET_MOTOR_SPEED, 0)

    def dispense_time(self, dispenser, duration):
        return self._send_packet32(dispenser, PACKET_TIME_DISPENSE, duration)

    def dispense_ticks(self, dispenser, ticks, speed=255):
        ret = self._send_packet16(dispenser, PACKET_TICK_SPEED_DISPENSE, ticks, speed)

        # if it fails, re-try once.
        if not ret:
            self.log("*** dispense command failed. re-trying once.")
            ret = self._send_packet16(dispenser, PACKET_TICK_SPEED_DISPENSE, ticks, speed)

        return ret

    def comm_test(self):
        return self._send_packet8(0, PACKET_COMM_TEST, 0)

    def is_dispensing(self, dispenser):
        """
        Returns a tuple of (dispensing, is_over_current)
        """

        # Sometimes the motors can interfere with communications.
        # In such cases, assume the motor is still running and
        # then assume the caller will again to see if it is still running
        self.set_timeout(.1)
        ret = self._send_packet8(dispenser, PACKET_IS_DISPENSING, 0)
        self.set_timeout(DEFAULT_TIMEOUT)
        if ret:
            ack, value0, value1 = self._receive_packet8_2()
            if ack == PACKET_ACK_OK:
                return value0, value1
            if ack == PACKET_ACK_TIMEOUT:
                return -1, -1
        return True, False

    def query_dispenser_version(self, dispenser):
        if self._send_packet8(dispenser, PACKET_GET_VERSION, 0):
            # set a short timeout, in case its a v2 dispenser
            self.set_timeout(.1)
            ack, ver, dummy = self._receive_packet16(True)
            self.set_timeout(DEFAULT_TIMEOUT)
            if ack == PACKET_ACK_OK:
                return ver
        return -1

    def get_saved_tick_count(self, dispenser):
        if self._send_packet8(dispenser, PACKET_SAVED_TICK_COUNT, 0):
            ack, ticks, dummy = self._receive_packet16()
            if ack == PACKET_ACK_OK:
                return ticks
        return -1

    def flush_saved_tick_count(self):
        return self._send_packet8(DEST_BROADCAST, PACKET_FLUSH_SAVED_TICK_COUNT, 0)

    # -----------------------------------------------
    # Past this point we only have private functions.
    # -----------------------------------------------

    def _send_packet(self, dest, packet):

        self.ser.flushInput()
        self.ser.flushOutput()

        crc = 0
        for ch in packet:
            crc = crc16_update(crc, ord(ch))

        encoded = pack7.pack_7bit(packet + pack("<H", crc))
        if len(encoded) != RAW_PACKET_SIZE:
            self.log("send_packet: Encoded packet size is wrong: %d vs %s" % (len(encoded), RAW_PACKET_SIZE))
            return False

        try:
            t0 = time()
            written = self.ser.write(chr(0xFF) + chr(0xFF) + encoded)
            if written != RAW_PACKET_SIZE + 2:
                self.log("*** send timeout")
                self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
                return False

            if dest == DEST_BROADCAST:
                return True

            ch = self.ser.read(1)
            t1 = time()
            self.log("packet time: %f" % (t1 - t0))
            if len(ch) < 1:
                self.log("*** send packet: read timeout")
                self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
                return False
        except serial.serialutil.SerialException, err:
            self.log("SerialException: %s" % err)
            return False

        ack = ord(ch)
        if ack == PACKET_ACK_OK:
            return True
        if ack == PACKET_CRC_FAIL:
            self.log("*** send_packet: packet ack crc fail")
            self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
            return False
        if ack == PACKET_ACK_TIMEOUT:
            self.log("*** send_packet: ack timeout")
            self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
            return False
        if ack == PACKET_ACK_INVALID:
            self.log("*** send_packet: dispenser received invalid packet")
            self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
            return False
        if ack == PACKET_ACK_INVALID_HEADER:
            self.log("*** send_packet: dispenser received invalid header")
            self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
            return False
        if ack == PACKET_ACK_HEADER_IN_PACKET:
            self.log("*** send_packet: header in packet error")
            self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
            return False

        # if we get an invalid ack code, it might be ok.
        self.log("send_packet: Invalid ACK code %d" % ord(ch))
        self.log("*** dispenser: %d, type: %d" % (dest + 1, ord(packet[1:2])))
        return False

    def _get_dispenser_id(self, dest):
        if dest != DEST_BROADCAST:
            return self.dispenser_id
        else:
            return dest

    def _send_packet8(self, dest, type, val0, val1=0, val2=0, val3=0):
        dispenser_id = self._get_dispenser_id(dest)
        if dispenser_id == 255:
            return False

        return self._send_packet(dest, pack("BBBBBB", dispenser_id, type, val0, val1, val2, val3))

    def _send_packet16(self, dest, type, val0, val1):
        dispenser_id = self._get_dispenser_id(dest)
        if dispenser_id == 255:
            return False

        return self._send_packet(dest, pack("<BBHH", dispenser_id, type, val0, val1))

    def _send_packet32(self, dest, type, val):
        dispenser_id = self._get_dispenser_id(dest)
        if dispenser_id == 255:
            return False

        return self._send_packet(dest, pack("<BBI", dispenser_id, type, val))

    def _receive_packet(self, quiet=False):

        header = 0
        while True:
            ch = self.ser.read(1)
            if len(ch) < 1:
                if not quiet:
                    self.log("receive packet: response timeout")
                return PACKET_ACK_TIMEOUT, ""

            if ord(ch) == 0xFF:
                header += 1
            else:
                header = 0

            if header == 2:
                break

        ack = PACKET_ACK_OK
        raw_packet = self.ser.read(RAW_PACKET_SIZE)
        if len(raw_packet) != RAW_PACKET_SIZE:
            if not quiet:
                self.log("receive packet: timeout")
            ack = PACKET_ACK_TIMEOUT

        if ack == PACKET_ACK_OK:
            packet = pack7.unpack_7bit(raw_packet)
            if len(packet) != PACKET_SIZE:
                ack = PACKET_ACK_INVALID
                if not quiet:
                    self.log("receive_packet: Unpacked length incorrect")

            if ack == PACKET_ACK_OK:
                received_crc = unpack("<H", packet[6:8])[0]
                packet = packet[0:6]

                crc = 0
                for ch in packet:
                    crc = crc16_update(crc, ord(ch))

                if received_crc != crc:
                    if not quiet:
                        self.log("receive_packet: CRC fail")
                    ack = PACKET_ACK_CRC_FAIL

        # Send the response back to the dispenser
        if self.ser.write(chr(ack)) != 1:
            if not quiet:
                self.log("receive_packet: Send ack timeout!")
            ack = PACKET_ACK_TIMEOUT

        if ack == PACKET_ACK_OK:
            return ack, packet
        else:
            return ack, ""

    def _receive_packet8(self, quiet=False):
        ack, packet = self._receive_packet(quiet)
        if ack == PACKET_ACK_OK:
            data = unpack("BBBBBB", packet)
            return ack, data[2]
        else:
            return ack, 0

    def _receive_packet8_2(self, quiet=False):
        ack, packet = self._receive_packet(quiet)
        if ack == PACKET_ACK_OK:
            data = unpack("BBBBBB", packet)
            return ack, data[2], data[3]
        else:
            return ack, 0, 0

    def _receive_packet16(self, quiet=False):
        ack, packet = self._receive_packet(quiet)
        if ack == PACKET_ACK_OK:
            data = unpack("<BBHH", packet)
            return ack, data[2], data[3]
        else:
            return ack, 0, 0

    def _clear_startup_log(self):
        self.startup_log = ""

    def _log_startup(self, txt):
        self.log(txt)
        self.startup_log += "%s\n" % txt
