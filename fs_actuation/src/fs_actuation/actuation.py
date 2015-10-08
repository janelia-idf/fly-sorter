from __future__ import division, print_function
import roslib; roslib.load_manifest('fs_actuation')
import rospy

import atexit
import json
import yaml
import time
import random

from arduino_relay_controller import ArduinoRelayControllers
# from arduino_relay_controller import ArduinoRelayController


class Actuation(object):

    def __init__(self,*args,**kwargs):
        self.hardware = rospy.get_param('/fs_actuation/fs_actuation_web_server/hardware')
        self.mode = str(rospy.get_param('/fs_actuation/fs_actuation_web_server/mode')).lower()

        self.parameters = rospy.get_param('/fs_parameters')

        if self.hardware:
            arduino_ports = rospy.get_param('/fs_parameters/serial_port/arduino')
            self.relay_controllers = ArduinoRelayControllers(use_ports=arduino_ports)
            atexit.register(self._exitActuation)

        self.time_puff_last = time.time()
        self.puff_duration = self.parameters['puff']['duration']
        self.puff_offset = self.parameters['puff']['offset']
        self.min_time_between_puffs = self.parameters['puff']['min_time_between']

        self.turntable_parameters = self.parameters['animatics'][2]
        self.turntable_vel = self.turntable_parameters['vel_default']
        self.turntable_pos_dict = self.turntable_parameters['pos_dict']
        self.travel_full_rev = self.turntable_pos_dict['full_revolution']
        self.deg_per_in = 360.0/self.travel_full_rev
        self.pixels_per_deg = self.parameters['camera']['y_pixels_per_degree']
        self.seconds_per_pixel = 1.0/(self.pixels_per_deg*self.deg_per_in*self.turntable_vel)
        self.seconds_travel_per_frame = self.parameters['camera']['y_pixels']*self.seconds_per_pixel
        self.travel_camera_to_origin = self.turntable_pos_dict['full_revolution'] - self.turntable_pos_dict['camera_center']
        self.travel_camera_to_puffer_male = self.travel_camera_to_origin + (self.turntable_pos_dict['puffer_male'] - self.turntable_pos_dict['origin'])
        self.travel_camera_to_puffer_female = self.travel_camera_to_origin + (self.turntable_pos_dict['puffer_female'] - self.turntable_pos_dict['origin'])
        self.travel_camera_to_puffer_unknown = self.travel_camera_to_origin + (self.turntable_pos_dict['puffer_unknown'] - self.turntable_pos_dict['origin'])

    def _exitActuation(self):
        self._stopAllPuffs()

    def getActuationInfo(self):
        pass

    # def _getRelayNumberFromRelay(self,relay):
    #     relay_number = None
    #     relay_str = str(relay).lower()
    #     if relay_str in self.parameters['relay']:
    #         relay_number = self.parameters['relay'][relay_str]
    #     elif int(relay) in self.parameters['relay'].values():
    #         relay_number = int(relay)
    #     else:
    #         raise IOError('Unknown relay value')
    #     return relay_number

    def _getRelayControllerAndRelayNumberFromRelay(self,relay):
        relay_number = None
        if self.hardware:
            relay_str = str(relay).lower()
            if relay_str in self.parameters['relay']:
                serial_number = self.parameters['relay'][relay_str]['serial_number']
                relay_controller = self.relay_controllers.getBySerialNumber(serial_number)
                relay_number = self.parameters['relay'][relay_str]['relay_number']
            else:
                raise IOError('Unknown relay value')
            return relay_controller,relay_number
        else:
            return None,None

    def _startPuffRepeated(self,relay,period,duty_cycle,count):
        rospy.loginfo('relay_controller.startRelayBlink: relay={0}, period={1}, duty_cycle={2}, count={3}'.format(relay,
                                                                                                                  period,
                                                                                                                  duty_cycle,
                                                                                                                  count))
        # relay_number = self._getRelayNumberFromRelay(relay)
        relay_controller,relay_number = self._getRelayControllerAndRelayNumberFromRelay(relay)
        if self.hardware and (relay_number is not None):
            period = int(period)
            duty_cycle = int(duty_cycle)
            count = int(count)
            # self.relay_controller.startRelayBlink(relay_number,period,duty_cycle,count)
            relay_controller.startRelayBlink(relay_number,period,duty_cycle,count)

    def _stopPuffRepeated(self,relay):
        rospy.loginfo('relay_controller.stopRelayBlink: relay={0}'.format(relay))
        # relay_number = self._getRelayNumberFromRelay(relay)
        relay_controller,relay_number = self._getRelayControllerAndRelayNumberFromRelay(relay)
        if self.hardware and (relay_number is not None):
            # self.relay_controller.stopRelayBlink(relay_number)
            relay_controller.stopRelayBlink(relay_number)

    def _setRelayOn(self,relay):
        rospy.loginfo('relay_controller.setRelayOn: relay={0}'.format(relay))
        # relay_number = self._getRelayNumberFromRelay(relay)
        relay_controller,relay_number = self._getRelayControllerAndRelayNumberFromRelay(relay)
        if self.hardware and (relay_number is not None):
            # self.relay_controller.setRelayOn(relay_number)
            relay_controller.setRelayOn(relay_number)

    def _setRelayOff(self,relay):
        rospy.loginfo('relay_controller.setRelayOff: relay={0}'.format(relay))
        # relay_number = self._getRelayNumberFromRelay(relay)
        relay_controller,relay_number = self._getRelayControllerAndRelayNumberFromRelay(relay)
        if self.hardware and (relay_number is not None):
            # self.relay_controller.setRelayOff(relay_number)
            relay_controller.setRelayOff(relay_number)

    def _stopAllPuffs(self):
        if self.hardware:
            for relay_controller in self.relay_controllers:
                relay_controller.removeAllTasks()
                relay_controller.setAllRelaysOff()

    def _addPuff(self,relay,delay,duration):
        rospy.loginfo('relay_controller.addPuff: relay={0}, delay={1}, duration={2}'.format(relay,
                                                                                            delay,
                                                                                            duration))
        # relay_number = self._getRelayNumberFromRelay(relay)
        relay_controller,relay_number = self._getRelayControllerAndRelayNumberFromRelay(relay)
        if self.hardware and (relay_number is not None):
            delay = int(delay)
            duration = int(duration)
            # self.relay_controller.addPulseCentered(relay_number,delay,duration)
            relay_controller.addPulseCentered(relay_number,delay,duration)

    def _toggleLights(self):
        rospy.loginfo('relay_controller.toggleDigitalOutput({0})'.format(self.parameters['digital_output']['lights']['digital_output']))
        if self.hardware:
            # self.relay_controller.toggleDigitalOutput(self.parameters['digital_output']['lights'])
            self.relay_controllers[0].toggleDigitalOutput(0)
            # serial_number = self.parameters['digital_output']['lights']['serial_number']
            # relay_controller = self.relay_controllers.getBySerialNumber(serial_number)
            # relay_controller.toggleDigitalOutput(self.parameters['digital_output']['lights']['digital_output'])

    def _setLightsOn(self):
        rospy.loginfo('relay_controller.setDigitalOutputHigh({0})'.format(self.parameters['digital_output']['lights']['digital_output']))
        if self.hardware:
            # self.relay_controller.setDigitalOutputHigh(self.parameters['digital_output']['lights'])
            serial_number = self.parameters['digital_output']['lights']['serial_number']
            relay_controller = self.relay_controllers.getBySerialNumber(serial_number)
            relay_controller.setDigitalOutputHigh(self.parameters['digital_output']['lights']['digital_output'])

    def _setLightsOff(self):
        rospy.loginfo('relay_controller.setDigitalOutputLow({0})'.format(self.parameters['digital_output']['lights']['digital_output']))
        if self.hardware:
            # self.relay_controller.setDigitalOutputLow(self.parameters['digital_output']['lights'])
            serial_number = self.parameters['digital_output']['lights']['serial_number']
            relay_controller = self.relay_controllers.getBySerialNumber(serial_number)
            relay_controller.setDigitalOutputLow(self.parameters['digital_output']['lights']['digital_output'])

    def _cap(self):
        rospy.loginfo('relay_controller.pulseDigitalOutput({0},{1})'.format(self.parameters['digital_output']['cap']['digital_output'],
                                                                            self.parameters['cap']['duration']))
        if self.hardware:
            # self.relay_controller.pulseDigitalOutput(self.parameters['digital_output']['cap'],self.parameters['cap']['duration'])
            serial_number = self.parameters['digital_output']['lights']['serial_number']
            relay_controller = self.relay_controllers.getBySerialNumber(serial_number)
            relay_controller.pulseDigitalOutput(self.parameters['digital_output']['cap']['digital_output'],self.parameters['cap']['duration'])

    def handleCmd(self,cmd,args):
        rsp_dict = {}
        if (cmd is None) and (args is None):
            rsp_dict['cmds'] = self.parameters['cmd']
        elif (cmd is not None):
            cmd = str(cmd)
            rsp_dict['cmd'] = cmd
            if cmd in self.parameters['cmd']:
                if cmd == 'sendData':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('ndetections' in args_object) and \
                        ('detections' in args_object) and \
                        ('time_acquired' in args_object) and \
                        ('time_sent' in args_object):
                            ndetections = args_object['ndetections']
                            detections = args_object['detections']
                            time_acquired = args_object['time_acquired']
                            time_sent = args_object['time_sent']

                            if not (ndetections == len(detections)):
                                rsp_dict['status'] = 'error'
                                rsp_dict['err_msg'] = 'length of detections does not match ndetections'
                            elif time_sent < time_acquired:
                                rsp_dict['status'] = 'error'
                                rsp_dict['err_msg'] = 'time_acquired later than time_sent'
                            elif 1 <= ndetections:
                                time_now = time.time()
                                if self.min_time_between_puffs <= (time_now - self.time_puff_last):
                                    self.time_puff_last = time_now
                                    # fly_data = args_object['detections'][0]
                                    for fly_data in args_object['detections']:
                                        fly_type = str(fly_data['fly_type']).lower()
                                        fly_id = int(fly_data['fly_id'])
                                        fly_x = float(fly_data['x'])
                                        fly_y = float(fly_data['y'])

                                        y_offset_pixels = self.parameters['camera']['y_pixels']/2.0 - fly_y
                                        y_offset_deg = y_offset_pixels/self.parameters['camera']['y_pixels_per_degree']
                                        y_offset_in = y_offset_deg/self.deg_per_in

                                        # if self.mode == 'all_male':
                                        #     fly_type = 'male'
                                        # elif self.mode == 'male_female':
                                        #     coin_flip_heads = random.random() < 0.5
                                        #     if coin_flip_heads:
                                        #         fly_type = 'male'
                                        #     else:
                                        #         fly_type = 'female'

                                        if fly_type == 'male':
                                            travel = self.travel_camera_to_puffer_male
                                        elif fly_type == 'female':
                                            travel = self.travel_camera_to_puffer_female
                                        else:
                                            fly_type = 'unknown'
                                            travel = self.travel_camera_to_puffer_unknown
                                        time_travel = (travel + y_offset_in)/self.turntable_vel
                                        time_spent = time_sent - time_acquired
                                        time_diff = time_travel - time_spent
                                        time_adjusted = int(time_diff*1000 + self.puff_offset)
                                        rsp_dict['fly_type'] = fly_type
                                        rsp_dict['puff_delay'] = time_adjusted
                                        rsp_dict['puff_duration'] = self.puff_duration
                                        rsp_dict['puff_offset'] = self.puff_offset
                                        rsp_dict['time_acquired'] = time_acquired
                                        if self.mode == 'normal' and (fly_type != 'unknown'):
                                            self._addPuff(fly_type,time_adjusted,self.puff_duration)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args need ndetections, detections, time_acquired, and time_sent'

                elif cmd == 'startPuffRepeated':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('relay' in args_object) and \
                        ('time_on' in args_object) and \
                        ('time_off' in args_object) and \
                        ('count' in args_object):
                            relay = args_object['relay']
                            time_on = args_object['time_on']
                            time_off = args_object['time_off']
                            count = args_object['count']
                            period = time_on + time_off
                            try:
                                duty_cycle = int(100*(time_on/period))
                            except ZeroDivisionError:
                                duty_cycle = 0
                            self._startPuffRepeated(relay,period,duty_cycle,count)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args need relay, time_on, time_off, and count'

                elif cmd == 'stopPuffRepeated':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('relay' in args_object):
                            relay = args_object['relay']
                            self._stopPuffRepeated(relay)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args needs relay'

                elif cmd == 'stopAllPuffs':
                    self._stopAllPuffs()

                elif cmd == 'addPuff':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('relay' in args_object) and \
                        ('delay' in args_object) and \
                        ('duration' in args_object):
                            relay = args_object['relay']
                            delay = args_object['delay']
                            duration = args_object['duration']
                            self._addPuff(relay,delay,duration)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args need relay, delay, and duration'

                elif cmd == 'toggleLights':
                    self._toggleLights()

                elif cmd == 'setLightsOn':
                    self._setLightsOn()

                elif cmd == 'setLightsOff':
                    self._setLightsOff()

                elif cmd == 'cap':
                    self._cap()

                elif cmd == 'setRelayOn':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('relay' in args_object):
                            relay = args_object['relay']
                            self._setRelayOn(relay)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args needs relay'

                elif cmd == 'setRelayOff':
                    if args is None:
                        rsp_dict['status'] = 'error'
                        rsp_dict['err_msg'] = 'no args'
                    else:
                        args_object = yaml.load(args)
                        if ('relay' in args_object):
                            relay = args_object['relay']
                            self._setRelayOff(relay)
                        else:
                            rsp_dict['status'] = 'error'
                            rsp_dict['err_msg'] = 'args needs relay'

            else:
                rsp_dict['status'] = 'error'
                rsp_dict['err_msg'] = 'unknown cmd: {0}'.format(cmd)
        else:
            rsp_dict['status'] = 'error'
        if 'status' not in rsp_dict:
            rsp_dict['status'] = 'success'
        rsp = json.dumps(rsp_dict,sort_keys=True,separators=(',', ':'))
        return rsp

