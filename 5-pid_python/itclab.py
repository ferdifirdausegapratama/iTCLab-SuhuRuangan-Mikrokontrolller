import sys
import time
import numpy as np
try:
    import serial
except ImportError:
    import pip
    pip.main(['install', 'pyserial'])
    import serial
from serial.tools import list_ports


class iTCLab(object):
    def __init__(self, port=None, baud=115200):
        port = self.findPort()
        print('Opening connection')
        self.sp = serial.Serial(port=port, baudrate=baud, timeout=2)
        self.sp.flushInput()
        self.sp.flushOutput()
        time.sleep(3)
        print('iTCLab connected via Arduino on port ' + port)

    def findPort(self):
        found = False
        for port in list(list_ports.comports()):
            if port[2].startswith('USB VID:PID=16D0:0613'):  # Arduino Uno
                port = port[0]
                found = True
            if port[2].startswith('USB VID:PID=1A86:7523'):  # Arduino HDuino
                port = port[0]
                found = True
            if port[2].startswith('USB VID:PID=2341:8036'):  # Arduino Leonardo
                port = port[0]
                found = True
            if port[2].startswith('USB VID:PID=10C4:EA60'):  # Arduino ESP32
                port = port[0]
                found = True
            if port[2].startswith('USB VID:PID=1A86:55D4'):  # ESP32 (variant)
                port = port[0]
                found = True
        if not found:
            print('Arduino COM port not found')
            print('Please ensure that the USB cable is connected')
            print('--- Printing Serial Ports ---')
            for port in list(serial.tools.list_ports.comports()):
                print(port[0] + ' ' + port[1] + ' ' + port[2])
            port = input('Input port: ')
        return port

    def stop(self):
        return self.read('X')

    def version(self):
        return self.read('VER')

    @property
    def T1(self):
        raw_data = self.read('T1')
        try:
            self._T1 = float(raw_data)
        except ValueError:
            print(f"Invalid data received for T1: {raw_data}")
            self._T1 = 0.0
        return self._T1

    @property
    def T2(self):
        raw_data = self.read('T2')
        try:
            self._T2 = float(raw_data)
        except ValueError:
            print(f"Invalid data received for T2: {raw_data}")
            self._T2 = 0.0
        return self._T2

    def LED(self, pwm):
        pwm = max(0.0, min(100.0, pwm)) / 2.0
        self.write('LED', pwm)
        return pwm

    def Q1(self, pwm):
        pwm = max(0.0, min(100.0, pwm))
        self.write('Q1', pwm)
        return pwm

    def Q2(self, pwm):
        pwm = max(0.0, min(100.0, pwm))
        self.write('Q2', pwm)
        return pwm

    def save_txt(self, t, u1, u2, y1, y2, sp1, sp2):
        data = np.vstack((t, u1, u2, y1, y2, sp1, sp2))
        data = data.T
        top = 'Time (sec), Heater 1 (%), Heater 2 (%), ' \
              + 'Temperature 1 (degC), Temperature 2 (degC), ' \
              + 'Set Point 1 (degC), Set Point 2 (degC)'
        np.savetxt('data.txt', data, delimiter=',', header=top, comments='')

    def read(self, cmd):
        cmd_str = self.build_cmd_str(cmd, '')
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except Exception as e:
            print(f"Error sending command {cmd}: {e}")
            return '0.0'

        raw_data = self.sp.readline().decode('UTF-8').strip()
        if raw_data == '':
            print(f"Warning: No data received for command {cmd}. Returning default value.")
            return '0.0'
        print(f"Debug: Received data for {cmd}: {raw_data}")
        return raw_data

    def write(self, cmd, pwm):
        cmd_str = self.build_cmd_str(cmd, (pwm,))
        try:
            self.sp.write(cmd_str.encode())
            self.sp.flush()
        except Exception as e:
            print(f"Error sending command {cmd} with pwm {pwm}: {e}")
            return None
        raw_data = self.sp.readline().decode('UTF-8').strip()
        return raw_data

    def build_cmd_str(self, cmd, args=None):
        if args:
            args = ' '.join(map(str, args))
        else:
            args = ''
        return "{cmd} {args}\n".format(cmd=cmd, args=args)

    def close(self):
        try:
            self.sp.close()
            print('Arduino disconnected successfully')
        except:
            print('Problems disconnecting from Arduino.')
        return True
