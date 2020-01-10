import serial
import struct
import threading
import time
import math

class MSPParser:
    def parse_ident(self, payload):
        r = struct.unpack("<BBBI", payload)
        return {
            'version': r[0],
            'multitype': r[1],
            'msp_version': r[2],
            'capability': r[3]
        }

    def parse_status(self, payload):
        r = struct.unpack("<HHHIB", payload)
        return {
            'cycleTime': r[0],
            'i2c_errors_count': r[1],
            'sensor': r[2],
            'flag': r[3],
            'global_conf.currentSet': r[4]
        }

    def parse_raw_imu(self, payload):
        (ax, ay, az) = struct.unpack("<hhh", payload[0:6])
        (gx, gy, gz) = struct.unpack("<hhh", payload[6:12])
        (mx, my, mz) = struct.unpack("<hhh", payload[12:18])
        return {
            'ax': ax, 'ay': ay, 'az': az, 
            'gx': gx, 'gy': gy, 'gz': gz,
            'mx': mx, 'my': my, 'mz': mz
        }

    def parse_servo(self, payload):
        motor = struct.unpack("<8h", payload)
        return { 'motor': motor }

    def parse_rc(self, payload):
        l = math.floor(len(payload) / 2)
        r = struct.unpack("<{}h".format(l))
        return { 'rcData': r }

    def parse(self, cmd, payload):
        data = {}
        if cmd == MSP.IDENT:
            data = self.parse_ident(payload)
        elif cmd == MSP.STATUS:
            data = self.parse_status(payload)
        elif cmd == MSP.RAW_IMU:
            data = self.parse_raw_imu(payload)
        elif cmd == MSP.SERVO:
            data = self.parse_servo(payload)
        elif cmd == MSP.RC:
            data = self.parse_rc(payload)
        else:
            print("Unknown cmd: ", function)
            print("Payload: ", payload)

        if cmd == "":
            return None

        return {'cmd': cmd, 'data': data}


class MSP:
    IDENT = 100
    STATUS = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    BOX = 113
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254
    VTX_CONFIG = 88
    VTX_SET_CONFIG = 89
    EEPROM_WRITE = 250
    REBOOT = 68

    def __init__(self, path="/dev/ttyUSB0", baudrate=115200):
        self.callbacks = {}
        self.ser = serial.Serial(path, baudrate)

        #self.request_queue  = queue.Queue()
        #self.response_queue = queue.Queue()

        self.req_t = threading.Thread(target=self.req_thread)
        self.req_t.daemon = True
        self.req_t.start()

    def req_thread(self):
        while True:
            cmd, size, payload = self.get_message()
            if cmd in self.callbacks:
                callback = self.callbacks[cmd]
                callback(payload)

    def res_thread(self):
        pass

    def get_message(self):
        header = self.ser.read(5)
        if header[0:1] != b'$':
            return None
        if header[1:2] != b'M':
            return None
        direction = header[2]
        size = ord(header[3])
        cmd = ord(header[4])
        payload = self.ser.read(size)
        checksum = self.ser.read(1)
        # FIXME: check checksum
        #self.parse_message(direction, cmd, payload)
        return (cmd, size, payload)
    
    def send_message(self, function, payload):
        header = b'$M<' + ''.join([chr(len(payload)), chr(function)])
        msg = header + payload
        crc = 0
        for b in msg[3:]:
            crc = crc ^ ord(b)
        msg = msg + chr(crc)
        self.ser.write(msg)

    def register_callback(self, cmd, callback):
        self.callbacks[cmd] = callback


# def rc_callback(rcData):
#     print("rcData: ", rcData)
# 
# msp = MSP()
# msp.register_callback(MSP.RC, rc_callback)
# msp.send_message(MSP.RC, b'')
