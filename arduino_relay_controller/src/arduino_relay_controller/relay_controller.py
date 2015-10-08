from __future__ import print_function, division
import atexit
import argparse
import copy
import time

from arduino_device import ArduinoDevice, ArduinoDevices, findArduinoDevicePorts


DEBUG = False
BAUDRATE = 'default'
DEVICE_MODEL_NUMBER = 1761

class ArduinoRelayController(ArduinoDevice):

    def __init__(self,*args,**kwargs):
        kwargs.update({'model_number': DEVICE_MODEL_NUMBER})
        serial_number = None
        if 'debug' not in kwargs:
            kwargs.update({'debug': DEBUG})
        if 'baudrate' not in kwargs:
            kwargs.update({'baudrate': BAUDRATE})
        elif (kwargs['baudrate'] is None) or (kwargs['baudrate'].lower() == 'default'):
            kwargs.update({'baudrate': BAUDRATE})
        if 'serial_number' in kwargs:
            serial_number = kwargs.pop('serial_number')
        if ('port' not in kwargs) or (kwargs['port'] is None):
            port =  findArduinoRelayControllerPort(baudrate=kwargs['baudrate'],
                                                   serial_number=serial_number)
            kwargs.update({'port': port})
        super(ArduinoRelayController,self).__init__(*args,**kwargs)
        atexit.register(self._exitArduinoRelayController)
        dev_info = self.getDevInfo()
        self.relay_count = dev_info['relay_count']
        self.clock_ticks_per_second = dev_info['clock_ticks_per_second']
        self._initClocks()

    def _exitArduinoRelayController(self):
        try:
            self.stopAllRelaysBlink()
        except IOError:
            pass

    def getArduinoRelayControllerInfo(self):
        arduino_relay_controller_info = self.getArduinoDeviceInfo()
        arduino_relay_controller_info.update({'relay_count': self.relay_count})
        arduino_relay_controller_info.update({'clock_ticks_per_second': self.clock_ticks_per_second})

        return arduino_relay_controller_info

    def _initClocks(self):
        self.time0 = time.time()
        self.setTime(0)

    def syncClocks(self):
        # now plus measured offset to account for communication time
        now = time.time() + 0.007
        time_host = long(round(self.clock_ticks_per_second*(now - self.time0)))
        self.setTime(time_host)

    def compareClocks(self):
        now = time.time()
        time_arduino = self.getTime()
        time_host = long(round(self.clock_ticks_per_second*(now - self.time0)))
        time_difference = (time_host - time_arduino)/1000
        time_drift = 1000*time_difference/time_host
        print('arduino time = {0}'.format(time_arduino))
        print('host time = {0}'.format(time_host))
        print('difference = {0}'.format(time_difference))
        print('drift = {0}'.format(time_drift))

# device_names example:
# [{'port':'/dev/ttyACM0',
#   'device_name':'relay_controller0'},
#  {'serial_number':3,
#   'device_name':'relay_controller1'}]
class ArduinoRelayControllers(ArduinoDevices):

    def __init__(self,*args,**kwargs):
        if ('use_ports' not in kwargs) or (kwargs['use_ports'] is None):
            kwargs['use_ports'] = findArduinoRelayControllerPorts(*args,**kwargs)
        super(ArduinoRelayControllers,self).__init__(*args,**kwargs)

    def appendDevice(self,*args,**kwargs):
        self.append(ArduinoRelayController(*args,**kwargs))

    def getArduinoRelayControllerInfo(self):
        arduino_relay_controller_info = []
        for device_index in range(len(self)):
            dev = self[device_index]
            arduino_relay_controller_info.append(dev.getArduinoRelayControllerInfo())
        return arduino_relay_controller_info


def findArduinoRelayControllerPorts(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
    arduino_device_ports = findArduinoDevicePorts(baudrate=baudrate,
                                                  model_number=DEVICE_MODEL_NUMBER,
                                                  serial_number=serial_number,
                                                  try_ports=try_ports,
                                                  debug=debug)

    if type(serial_number) is int:
        serial_number = [serial_number]

    arduino_relay_controller_ports = {}
    for port in arduino_device_ports:
        try:
            dev_serial_number = arduino_device_ports[port]['serial_number']
        except KeyError:
            break
        if (serial_number is None) or (dev_serial_number in serial_number):
            arduino_relay_controller_ports[port] = {'serial_number': dev_serial_number}
    return arduino_relay_controller_ports

def findArduinoRelayControllerPort(baudrate=None, serial_number=None, try_ports=None, debug=DEBUG):
    arduino_relay_controller_ports = findArduinoRelayControllerPorts(baudrate=baudrate,
                                                                     serial_number=serial_number,
                                                                     try_ports=try_ports,
                                                                     debug=debug)
    if len(arduino_relay_controller_ports) == 1:
        return arduino_relay_controller_ports.keys()[0]
    elif len(arduino_relay_controller_ports) == 0:
        arduino_device_ports = findArduinoDevicePorts(baudrate=baudrate,
                                                      model_number=DEVICE_MODEL_NUMBER,
                                                      serial_number=serial_number,
                                                      try_ports=try_ports,
                                                      debug=debug)
        err_str = 'Could not find Arduino relay controllers. Check connections and permissions.\n'
        err_str += 'Tried ports: ' + str(arduino_device_ports)
        raise RuntimeError(err_str)
    else:
        err_str = 'Found more than one Arduino relay controller. Specify port or serial_number.\n'
        err_str += 'Matching ports: ' + str(arduino_relay_controller_ports)
        raise RuntimeError(err_str)

def arduinoRelayControllerCli():
    parser = argparse.ArgumentParser(description='Arduino Relay Controller')
    # parser.add_argument('--debug',
    #                     help='Use the simulated software relay controller instead of the real hardware relay controller',
    #                     action='store_true')
    # parser.add_argument('-d','--device',nargs=1,type=int,default=[0],
    #                     help='device index')
    subparsers = parser.add_subparsers(dest='subparser_name',help='sub-command help')

    # create the parser for the "info" command
    parser_info = subparsers.add_parser('info', help='info help')

    # create the parser for the "relay" command
    parser_relay = subparsers.add_parser('relay', help='relay help')
    parser_relay.add_argument('-r','--relay',nargs=1,type=int,default=[0],
                              help='relay index')
    parser_relay.add_argument('--on',action="store_true",
                              help='turn on relay')
    parser_relay.add_argument('--off',action="store_true",
                              help='turn off relay')

    args = parser.parse_args()

    o = ArduinoRelayController(debug=DEBUG)
    # if not args.debug:
    #     o = ArduinoRelayController()
    # else:
    #     o = ArduinoRelayControllerDebug()
    # device = args.device[0]
    if args.subparser_name == 'relay':
        relay = args.relay[0]
        if args.on:
            o.setRelayOn(relay)
        elif args.off:
            o.setRelayOff(relay)
        else:
            print("Add --on or --off")
    elif args.subparser_name == 'info':
        print(o.getArduinoRelayControllerInfoDict())

# -----------------------------------------------------------------------------------------
if __name__ == '__main__':
    arduinoRelayControllerCli()
