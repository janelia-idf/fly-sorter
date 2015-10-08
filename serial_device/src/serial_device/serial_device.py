from __future__ import print_function, division
import os
import serial
import time
import platform
import atexit
import exceptions
import operator


DEBUG = False

class WriteFrequencyError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class SerialDevice(serial.Serial):

    TIMEOUT = 0.05
    WRITE_READ_DELAY = 0.05
    WRITE_WRITE_DELAY = 0.05

    def __init__(self, *args, **kwargs):
        try:
            self.debug = kwargs.pop('debug')
        except KeyError:
            self.debug = DEBUG
        if 'try_ports' in kwargs:
            try_ports = kwargs.pop('try_ports')
        else:
            try_ports = None
        try:
            self.write_read_delay = kwargs.pop('write_read_delay')
        except KeyError:
            self.write_read_delay = self.WRITE_READ_DELAY
        try:
            self.write_write_delay = kwargs.pop('write_write_delay')
        except KeyError:
            self.write_write_delay = self.WRITE_WRITE_DELAY
        try:
            self.device_name = kwargs.pop('device_name')
        except KeyError:
            self.device_name = ''

        if ('port' not in kwargs) or (kwargs['port'] is None):
            kwargs.update({'port': findSerialDevicePort(try_ports=try_ports,debug=self.debug)})
        if 'timeout' not in kwargs:
            kwargs.update({'timeout': self.TIMEOUT})

        super(SerialDevice,self).__init__(*args,**kwargs)
        atexit.register(self._exitSerialDevice)
        self.time_write_prev = time.time()

    def writeCheckFreq(self,cmd_str,delay_write=False):

        '''Use instead of self.write when you want to ensure that
        serial write commands do not happen too
        frequently. delay_write=True waits and then writes the serial
        command, delay_write=False raises WriteFrequencyError
        Exception if time between method calls is too short. Might
        remove delay_write option if it turns out to be
        unnecessary.'''

        time_now = time.time()
        time_since_write_prev = time_now - self.time_write_prev
        if time_since_write_prev < self.write_write_delay:
            delay_time_needed = self.write_write_delay - time_since_write_prev
            if delay_write:
                time.sleep(delay_time_needed)
            else:
                raise WriteFrequencyError(delay_time_needed)
        try:
            bytes_written = self.write(cmd_str)
            self.time_write_prev = time_now
        except (serial.writeTimeoutError):
            bytes_written = 0
        self.debugPrint('command:', cmd_str)
        self.debugPrint('bytes_written:', bytes_written)
        return bytes_written

    def writeRead(self,cmd_str,use_readline=True,check_write_freq=True):

        '''A simple self.write followed by a self.readline with a
        delay set by write_read_delay when use_readline=True and
        check_write_freq=False. Setting check_write_freq=True ensures
        the write frequency is not too fast for the serial device to
        handle. Setting use_readline=False reads all response
        characters that are available instead of looking for the end
        of line character or timing out.'''

        # First clear garbage.
        chars_waiting = self.inWaiting()
        self.read(chars_waiting)
        response = None
        if check_write_freq:
            bytes_written = self.writeCheckFreq(cmd_str,delay_write=True)
        else:
            bytes_written = self.write(cmd_str)
        if 0 < bytes_written:
            time.sleep(self.write_read_delay)
            if use_readline:
                response = self.readline()
            else:
                chars_waiting = self.inWaiting()
                self.debugPrint('chars_waiting:', chars_waiting)
                response = self.read(chars_waiting)
            self.debugPrint('response:', response)
        return response

    def _exitSerialDevice(self):
        """
        Close the serial connection to provide some clean up.
        """
        self.close()

    def debugPrint(self, *args):
        if self.debug:
            print(*args)

    def getSerialDeviceInfo(self):
        serial_device_info = {'device_name' : self.device_name,
                              'port' : self.port,
                              }
        return serial_device_info

    def getDeviceName(self):
        return self.device_name

    def setDeviceName(self,device_name):
        self.device_name = str(device_name)

# device_names example:
# [{'port':'/dev/ttyACM0',
#   'device_name':'port0'},
#  {'port':'/dev/ttyACM1',
#   'device_name':'port1'}]
class SerialDevices(list):

    def __init__(self,*args,**kwargs):
        if 'debug' in kwargs:
            debug = kwargs['debug']
        else:
            debug = DEBUG
        if 'use_ports' in kwargs:
            use_ports = kwargs.pop('use_ports')
        else:
            use_ports = None
        if 'try_ports' in kwargs:
            try_ports = kwargs.pop('try_ports')
        else:
            try_ports = None
        if 'device_names' in kwargs:
            device_names = kwargs.pop('device_names')
        else:
            device_names = []

        if use_ports is not None:
            serial_device_ports = use_ports
        else:
            serial_device_ports = findSerialDevicePorts(try_ports=try_ports,debug=debug)
        for port in serial_device_ports:
            kwargs.update({'port': port})
            self.appendDevice(*args,**kwargs)

        self.updateDeviceNames(device_names)

    def appendDevice(self,*args,**kwargs):
        self.append(SerialDevice(*args,**kwargs))

    def getSerialDevicesInfo(self):
        serial_devices_info = []
        for dev in self:
            serial_devices_info.append(dev.getSerialDeviceInfo())
        return serial_devices_info

    def updateDeviceNames(self,device_names):
        for name_dict in device_names:
            device_name = name_dict.pop('device_name')
            for device_index in range(len(self)):
                dev = self[device_index]
                match = True
                for key in name_dict.keys():
                    if name_dict[key] != getattr(dev,key):
                        match = False
                if match:
                    dev.device_name = str(device_name)

    def sortByPort(self,*args,**kwargs):
        kwargs['key'] = operator.attrgetter('port')
        self.sort(**kwargs)

    def getByPort(self,port):
        for device_index in range(len(self)):
            dev = self[device_index]
            if dev.port == port:
                return dev

    def sortByDeviceName(self,*args,**kwargs):
        kwargs['key'] = operator.attrgetter('device_name','port')
        self.sort(**kwargs)

    def getByDeviceName(self,device_name):
        dev_list = []
        for device_index in range(len(self)):
            dev = self[device_index]
            if dev.device_name == device_name:
                dev_list.append(dev)
        if len(dev_list) == 1:
            return dev_list[0]
        elif 1 < len(dev_list):
            return dev_list


# ----------------------------------------------------------------------------

def findSerialDevicePorts(try_ports=None, debug=DEBUG):
    serial_device_ports = []
    os_type = platform.system()
    if os_type == 'Linux':
        serial_device_ports = os.listdir('{0}dev'.format(os.path.sep))
        serial_device_ports = [x for x in serial_device_ports if 'ttyUSB' in x or 'ttyACM' in x]
        serial_device_ports = ['{0}dev{0}{1}'.format(os.path.sep,x) for x in serial_device_ports]
    elif os_type == 'Windows':
        import _winreg as winreg
        import itertools

        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        try:
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
        except WindowsError:
            key = None

        if key is not None:
            for i in itertools.count():
                try:
                    val = winreg.EnumValue(key, i)
                    # Only return USBSER devices
                    if 'USBSER' in val[0]:
                        serial_device_ports.append(str(val[1]))
                except EnvironmentError:
                    break

    if try_ports is not None:
        serial_device_ports = list(set(try_ports) & set(serial_device_ports))

    serial_device_ports.sort()
    return serial_device_ports

def findSerialDevicePort(try_ports=None, debug=DEBUG):
    serial_device_ports = findSerialDevicePorts(try_ports)
    if len(serial_device_ports) == 1:
        return serial_device_ports[0]
    elif len(serial_device_ports) == 0:
        raise RuntimeError('Could not find any serial devices. Check connections and permissions.')
    else:
        raise RuntimeError('Found more than one serial device. Specify port.\n' + str(serial_device_ports))
