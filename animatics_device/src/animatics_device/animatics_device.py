from __future__ import print_function, division
import serial
import time
import atexit
import json
import operator
from functools import partial
import threading

from serial_device import SerialDevice, findSerialDevicePorts, WriteFrequencyError


DEBUG = True
MOTOR_ADDRESS_OFFSET = 128
MOTOR_ADDRESS_MAX = 120
MOTOR_COUNT_LIMIT = 16

lock = threading.Lock()

class AnimaticsMotor(object):

    """sendCmd and sendCmdGetRsp methods must be attached to
    AnimaticsMotor instance before initialize method is called."""

    def __init__(self,*args,**kwargs):
        try:
            self.motor_address = kwargs.pop('motor_address')
        except KeyError:
            self.motor_address = None
        try:
            self.motor_axis = kwargs.pop('motor_axis')
        except KeyError:
            self.motor_axis = None
        try:
            self.motor_name = kwargs.pop('motor_name')
        except KeyError:
            self.motor_name = ''
        try:
            self.scale_factor = kwargs.pop('scale_factor')
        except KeyError:
            self.scale_factor = 1
        try:
            self.set_default_mode = kwargs.pop('set_default_mode')
        except KeyError:
            self.set_default_mode = True

        self.encoder_resolution = None
        self.sample_rate = None
        self.acc_default = 1
        self.vel_default = 1
        self.vel_max = 2
        self.current_limit_default = 100
        self.homing = False
        self.homed = False
        self.vel_default_home = -0.1
        self.current_limit_default_home = 40
        self.origin_shift = 0
        self.pos_list = []
        self.pos_dict = {}
        self.mode = None

    def initialize(self,*args,**kwargs):
        self.encoder_resolution = self.getEncoderResolution()
        self.sample_rate = self.getSampleRate()
        if self.set_default_mode:
            self.setVelModeWithDefaults()

    def getAnimaticsMotorInfo(self):
        animatics_motor_info = {'motor_address': self.motor_address,
                                'motor_axis': self.motor_axis,
                                'motor_name': self.motor_name,
                                'scale_factor': self.scale_factor,
                                'acc_default': self.acc_default,
                                'vel_default': self.vel_default,
                                'vel_max': self.vel_max,
                                'current_limit_default': self.current_limit_default,
                                'vel_default_home': self.current_limit_default,
                                'current_limit_default_home': self.current_limit_default,
                                'homing': self.homing,
                                'homed': self.homed,
                                'mode': self.mode,
                                'origin_shift': self.origin_shift,
                                'pos_list': self.pos_list,
                                'pos_dict': self.pos_dict,
                                'encoder_resolution': self.encoder_resolution,
                                'sample_rate': self.sample_rate,
                                'acc_target': self.getAccTarget(),
                                'vel_target': self.getVelTarget(),
                                'vel': self.getVel(),
                                'pos_target': self.getPosTarget(),
                                'pos': self.getPos(),
                                'pos_error': self.getPosError(),
                                'temperature': self.getTemperature(),
                                'current': self.getCurrent(),
                                'current_limit': self.getCurrentLimit(),
                                'pid_values': self.getPidValues(),
                                'status': self.getStatusTrue(),
                                }
        return animatics_motor_info

    def setParameters(self,parameters):
        if 'motor_axis' in parameters:
            self.setMotorAxis(parameters['motor_axis'])
        if 'motor_name' in parameters:
            self.setMotorName(parameters['motor_name'])
        if 'scale_factor' in parameters:
            self.setScaleFactor(parameters['scale_factor'])
        if 'acc_default' in parameters:
            self.setAccDefault(parameters['acc_default'])
        if 'vel_max' in parameters:
            self.vel_max = parameters['vel_max']
        if 'vel_default' in parameters:
            self.setVelDefault(parameters['vel_default'])
        if 'current_limit_default' in parameters:
            self.setCurrentLimitDefault(parameters['current_limit_default'])
        if 'vel_default_home' in parameters:
            self.vel_default_home = parameters['vel_default_home']
        if 'current_limit_default_home' in parameters:
            self.current_limit_default_home = parameters['current_limit_default_home']
        if 'mode' in parameters:
            mode = parameters['mode']
            try:
                if mode[0].lower() == 'p':
                    self.setPosMode()
                elif mode[0].lower() == 'v':
                    self.setVelMode()
            except IndexError:
                pass
        if 'hardware_limits' in parameters:
            hardware_limits = parameters['hardware_limits']
            if not hardware_limits:
                self.disableHardwareLimits()
        if 'origin_shift' in parameters:
            self.origin_shift = parameters['origin_shift']
        if 'pos_list' in parameters:
            self.pos_list = parameters['pos_list']
        if 'pos_dict' in parameters:
            self.pos_dict = parameters['pos_dict']
        if 'pid_values' in parameters:
            self.setPidValues(parameters['pid_values'])

    def setVelModeWithDefaults(self):

        '''Set acceleration and velocity defaults, set to velocity
        mode, and disable hardware limits.'''

        self.setAcc(self.acc_default)
        self.setVel(self.vel_default)
        self.setVelMode()
        self.disableHardwareLimits()

    def setPosModeWithDefaults(self):

        '''Set acceleration and velocity defaults, set to position
        mode, and disable hardware limits.'''

        self.setAcc(self.acc_default)
        self.setVel(self.vel_default)
        self.setPosMode()
        self.disableHardwareLimits()

    def getMotorAddress(self):
        return self.motor_address

    def setMotorAddress(self,motor_address):
        self.motor_address = int(motor_address)

    def getMotorAxis(self):
        return self.motor_axis

    def setMotorAxis(self,motor_axis):
        self.motor_axis = int(motor_axis)

    def getMotorName(self):
        return self.motor_name

    def setMotorName(self,motor_name):
        self.motor_name = str(motor_name)

    def setAcc(self,acc):

        '''Set acceleration and deceleration target
        (output_unit/s/s)'''

        acc = self._convertAccToMotorUnits(acc)
        bytes_written = self.sendCmd('ADT={acc}'.format(acc=acc))
        return bytes_written

    def setAccDefault(self,acc):

        '''Set acceleration and deceleration target and default
        (output_unit/s/s)'''

        self.acc_default = acc
        bytes_written = self.setAcc(acc)
        return bytes_written

    def setVel(self,vel,suggest=False):

        '''Set velocity target (output_unit/s)'''

        if self.vel_max < vel:
            vel = self.vel_max
        vel = self._convertVelToMotorUnits(vel)
        bytes_written = self.sendCmd('VT={vel}'.format(vel=vel),suggest=suggest)
        return bytes_written

    def setVelDefault(self,vel):

        '''Set velocity target and default (output_unit/s)'''

        if self.vel_max < vel:
            vel = self.vel_max
        self.vel_default = vel
        bytes_written = self.setVel(vel)
        return bytes_written

    def setPos(self,pos,suggest=False):

        '''Set absolute position target (output_unit)'''

        pos = self._convertPosToMotorUnits(pos)
        bytes_written = self.sendCmd('PT={pos}'.format(pos=pos),suggest=suggest)
        return bytes_written

    def setPosRel(self,pos,suggest=False):

        '''Set relative position target (output_unit)'''

        pos = self._convertPosToMotorUnits(pos)
        bytes_written = self.sendCmd('PRT={pos}'.format(pos=pos),suggest=suggest)
        return bytes_written

    def setOrigin(self,pos=0):

        '''Set origin (output_unit) default = 0'''

        pos = self._convertPosToMotorUnits(pos)
        bytes_written = self.sendCmd('O={pos}'.format(pos=pos))
        return bytes_written

    def shiftOrigin(self,pos=0):

        '''Shift the origin by a relative amount (output_unit) default
        = 0 The origin can be shifted during motion without losing
        position counts.'''

        pos = self._convertPosToMotorUnits(pos)
        bytes_written = self.sendCmd('OSH={pos}'.format(pos=pos))
        return bytes_written

    def setVelMode(self):

        '''Set velocity mode. Enables continuous rotation of the motor
        shaft. The go command will cause the shaft to accelerate to
        the velocity target and continue spinning at that
        rate. Positive velocity targets will cause the shaft to rotate
        in one direction, negative in the other.'''

        self.mode = 'vel'
        bytes_written = self.sendCmd('MV')
        return bytes_written

    def setPosMode(self):

        '''Set position mode. The go command will cause the shaft to
        acclerate to the velocity target and spin until decelerating
        and stopping on the position target. Only the absolute value
        of the velocity target is considered, the shaft spins in the
        proper direction to reach the velocity target from the
        starting position.'''

        self.mode = 'pos'
        bytes_written = self.sendCmd('MP')
        return bytes_written

    def setScaleFactor(self,scale_factor):

        '''scale_factor = motor shaft revolutions per 1 output unit
        scale_factor cannot equal 0
        e.g. radian output: scale_factor = 1/(2*math.pi)
        e.g. 100:1 gearhead ratio: scale_factor = 100
        e.g. 100:1 gearhead ratio with reversed direction: scale_factor = -100'''

        if scale_factor == 0:
            raise ValueError('scale_factor cannot be 0!')
        else:
            self.scale_factor = scale_factor

    def getScaleFactor(self):

        '''scale_factor = motor shaft revolutions per 1 output unit
        scale_factor cannot equal 0
        e.g. radian output: scale_factor = 1/(2*math.pi)
        e.g. 100:1 gearhead ratio: scale_factor = 100
        e.g. 100:1 gearhead ratio with reversed direction: scale_factor = -100'''

        return self.scale_factor

    def reset(self):

        '''Total Animatics device reset.'''

        bytes_written = self.sendCmd('Z')
        return bytes_written

    def go(self):

        '''Start motion.'''

        bytes_written = self.sendCmd('G')
        return bytes_written

    def stop(self):

        '''Decelerate the motor to stop.'''

        bytes_written = self.sendCmd('X')
        return bytes_written

    def stopAbrupt(self):

        '''Stop motor abruptly.'''

        bytes_written = self.sendCmd('S')
        return bytes_written

    def goToVel(self,vel=None,suggest=False):

        '''Set velocity mode if necessary and go to directly to
        velocity. Do not pass Go, do not collect $200. Set suggest =
        True if it is OK to ignore this command if sent too quickly
        after previous command. suggest = False (default) causes
        program to wait and block until command is able to send.'''

        if vel is None:
            vel = self.vel_default
        bytes_written = self.setVel(vel,suggest=suggest)
        if 0 < bytes_written:
            if self.mode != 'vel':
                self.setVelMode()
            self.go()
        return bytes_written

    def goToPos(self,pos=0,vel=None,suggest=False):

        '''Set position mode if necessary and go directly to position
        in absolute coordinates. Do not pass Go, do not collect
        $200. Set suggest = True if it is OK to ignore this command if
        sent too quickly after previous command. suggest = False
        (default) causes program to wait and block until command is
        able to send.'''

        if vel is None:
            vel = self.vel_default
        bytes_written = self.setPos(pos,suggest=suggest)
        if 0 < bytes_written:
            if self.mode != 'pos':
                self.setPosMode()
            self.setVel(vel,suggest=False)
            self.go()
        return bytes_written

    def goToPosRel(self,pos=0,vel=None,suggest=False):

        '''Set position mode if necessary and go directly to position
        in relative coordinates. Do not pass Go, do not collect
        $200. Set suggest = True if it is OK to ignore this command if
        sent too quickly after previous command. suggest = False
        (default) causes program to wait and block until command is
        able to send.'''

        if vel is None:
            vel = self.vel_default
        bytes_written = self.setPosRel(pos,suggest=suggest)
        if 0 < bytes_written:
            if self.mode != 'pos':
                self.setPosMode()
            self.setVel(vel,suggest=False)
            self.go()
        return bytes_written

    def goToPosList(self,index,vel=None):

        '''Go to the position stored in instance variable pos_list at
        the given index and vel.'''

        index = int(index)
        pos = self.pos_list[index]
        self.goToPos(pos=pos,vel=vel)

    def goToPosDict(self,key,vel=None):

        '''Go to the position stored in instance variable pos_dict at
        the given key and vel.'''

        key = str(key)
        pos = self.pos_dict[key]
        self.goToPos(pos=pos,vel=vel)

    def atPosDict(self,key):

        '''Check to see if position matches position stored in
        instance variable pos_dict at the given key.'''

        if self.getStatusTrajectoryInProgress():
            return False
        key = str(key)
        pos_target = self.pos_dict[key]
        pos = self.getPos()
        pos_error = abs(pos - pos_target)
        max_pos_error = 0.01
        return pos_error < max_pos_error

    def off(self):

        '''Stop servoing the motor. This will not cause the motor to
        spin freely, use method "free" for that.'''

        bytes_written = self.sendCmd('OFF')
        return bytes_written


    def homeStart(self,vel=None,current_limit=None):
        if vel is None:
            vel = self.vel_default_home
        if current_limit is None:
            current_limit = self.current_limit_default_home
        self.stop()
        self.setCurrentLimit(current_limit)
        self.resetErrors()
        self.goToVel(vel)
        self.homing = True

    def homeInProgress(self):
        return self.homing and not self.homeCompleted()

    def homeCompleted(self):
        return self.homing and \
               not self.getStatusTrajectoryInProgress() and \
               (self.getStatusOverCurrent() or self.getStatusPosError())

    def setHome(self):

        '''Set origin, shift origin by origin_shift value, reset
        errors, and set current limit back to default.'''

        self.homing = False
        self.homed = True
        self.setOrigin()
        self.shiftOrigin(self.origin_shift)
        self.resetErrors()
        self.setCurrentLimit(self.current_limit_default)

    def isHomed(self):
        return self.homed

    def brakeRelease(self):

        '''Release the power safe brakes.'''

        bytes_written = self.sendCmd('BRKRLS')
        return bytes_written

    def brakeEngage(self):

        '''Engage the power safe brakes.'''

        bytes_written = self.sendCmd('BRKENG')
        return bytes_written

    def free(self):

        '''Causes the motor to spin freely.'''

        self.off()
        self.brakeRelease()
        bytes_written = self.resetErrors()
        return bytes_written

    def disablePosHardwareLimit(self):
        if 0 < self.scale_factor:
            bytes_written = self.sendCmd('EIGN(2)')
        else:
            bytes_written = self.sendCmd('EIGN(3)')
        return bytes_written

    def disableNegHardwareLimit(self):
        if 0 < self.scale_factor:
            bytes_written = self.sendCmd('EIGN(3)')
        else:
            bytes_written = self.sendCmd('EIGN(2)')
        return bytes_written

    def resetErrors(self):

        '''Reset system latches to power-up state.'''

        bytes_written = self.sendCmd('ZS')
        return bytes_written

    def disableHardwareLimits(self):
        self.disablePosHardwareLimit()
        self.disableNegHardwareLimit()
        bytes_written = self.resetErrors()
        return bytes_written

    def getEncoderResolution(self):
        return self.sendCmdGetRsp('RRES')

    def getSampleRate(self):
        return self.sendCmdGetRsp('RSAMP')

    def getAccTarget(self):
        acc = self.sendCmdGetRsp('RAT')
        acc = self._convertAccFromMotorUnits(acc)
        return acc

    def getVelTarget(self):
        vel = self.sendCmdGetRsp('RVT')
        vel = self._convertVelFromMotorUnits(vel)
        return vel

    def getVel(self):
        vel = self.sendCmdGetRsp('RVA')
        vel = self._convertVelFromMotorUnits(vel)
        return vel

    def getPosTarget(self):
        pos = self.sendCmdGetRsp('RPT')
        pos = self._convertPosFromMotorUnits(pos)
        return pos

    def getPos(self):
        pos = self.sendCmdGetRsp('RPA')
        pos = self._convertPosFromMotorUnits(pos)
        return pos

    def getPosModulo(self):
        pos = self.sendCmdGetRsp('RPMA')
        pos = self._convertPosFromMotorUnits(pos)
        return pos

    def getPosError(self):
        pos = self.sendCmdGetRsp('REA')
        pos = self._convertPosFromMotorUnits(pos)
        return pos

    def getTemperature(self):
        temp = self.sendCmdGetRsp('RTEMP')
        return temp

    def getCurrent(self):
        current = self.sendCmdGetRsp('RUIA')
        return current

    def setCurrentLimit(self,limit):

        """Set current limit as a percentage of the maximum value. 0 <=
        limit <= 100"""

        amps = (limit/100)*1023
        bytes_written = self.sendCmd('AMPS={amps}'.format(amps=amps))
        return bytes_written

    def setCurrentLimitDefault(self,limit):

        """Set current limit as a percentage of the maximum value. 0 <=
        limit <= 100"""

        self.current_limit_default = limit
        bytes_written = self.setCurrentLimit(limit)
        return bytes_written

    def getCurrentLimit(self):

        """Get current limit as a percentage of the maximum value. 0 <=
        limit <= 100"""

        amps = self.sendCmdGetRsp('RAMPS')
        limit = (amps/1023)*100
        return limit

    def getVoltage(self):
        voltage = self.sendCmdGetRsp('RUJA')
        return voltage

    def getPi(self):

        """Gets the value of PI from the motor. Can be used to check
        if the motor is responding properly."""

        pi = self.sendCmdGetRsp('RPI')
        return pi

    def getSerialAddress(self):
        addr = self.sendCmdGetRsp('RADDR')
        return addr

    def getCanAddress(self):
        addr = self.sendCmdGetRsp('RCADDR')
        return addr

    def getClock(self):
        clock = self.sendCmdGetRsp('RCLK')
        return clock

    def getModeNumber(self):
        mode = self.sendCmdGetRsp('RMODE')
        return mode

    def getMode(self):
        return self.mode

    def getSoftLimitMode(self):
        mode = self.sendCmdGetRsp('RSLM')
        return mode

    def getSoftLimitNeg(self):
        if 0 < self.scale_factor:
            limit = self.sendCmdGetRsp('RSLN')
        else:
            limit = self.sendCmdGetRsp('RSLP')
        return limit

    def getSoftLimitPos(self):
        if 0 < self.scale_factor:
            limit = self.sendCmdGetRsp('RSLP')
        else:
            limit = self.sendCmdGetRsp('RSLN')
        return limit

    def getInputs(self):
        inputs = []
        for input in range(8):
            inputs.append(self.sendCmdGetRsp('RIN({0})'.format(input)))
        return inputs

    def getStatus(self):
        sw0 = self.sendCmdGetRsp('RW(0)')
        sw1 = self.sendCmdGetRsp('RW(1)')
        sw2 = self.sendCmdGetRsp('RW(2)')
        sw3 = self.sendCmdGetRsp('RW(3)')
        if 0 < self.scale_factor:
            p = 'pos'
            n = 'neg'
        else:
            p = 'neg'
            n = 'pos'
        status = {
            'drive_ready': bool(sw0 & 1<<0),
            'motor_is_off': bool(sw0 & 1<<1),
            'trajectory_in_progress': bool(sw0 & 1<<2),
            'bus_voltage_fault': bool(sw0 & 1<<3),
            'over_current_occurred': bool(sw0 & 1<<4),
            'excessive_temperature_fault': bool(sw0 & 1<<5),
            'excessive_position_error': bool(sw0 & 1<<6),
            'velocity_limit': bool(sw0 & 1<<7),
            'real_time_temperature_limit': bool(sw0 & 1<<8),
            'de_dt_error_limit': bool(sw0 & 1<<9),
            p+'_hardware_limit_enabled': bool(sw0 & 1<<10),
            n+'_hardware_limit_enabled': bool(sw0 & 1<<11),
            'historical_'+p+'_hardware_limit': bool(sw0 & 1<<12),
            'historical_'+n+'_hardware_limit': bool(sw0 & 1<<13),
            p+'_hardware_limit_asserted': bool(sw0 & 1<<14),
            n+'_hardware_limit_asserted': bool(sw0 & 1<<15),

            'rise_capture_encoder_0_armed': bool(sw1 & 1<<0),
            'fall_capture_encoder_0_armed': bool(sw1 & 1<<1),
            'rising_edge_captured_encoder_0': bool(sw1 & 1<<2),
            'falling_edge_captured_encoder_0': bool(sw1 & 1<<3),
            'rise_capture_encoder_1_armed': bool(sw1 & 1<<4),
            'fall_capture_encoder_1_armed': bool(sw1 & 1<<5),
            'rising_edge_captured_encoder_1': bool(sw1 & 1<<6),
            'falling_edge_captured_encoder_1': bool(sw1 & 1<<7),
            'capture_0_input_state': bool(sw1 & 1<<8),
            'capture_1_input_state': bool(sw1 & 1<<9),
            'software_limits_enabled': bool(sw1 & 1<<10),
            'software_limit_mode': bool(sw1 & 1<<11),
            'historical_'+p+'_software_limit': bool(sw1 & 1<<12),
            'historical_'+n+'_software_limit': bool(sw1 & 1<<13),
            p+'_software_limit_asserted': bool(sw1 & 1<<14),
            n+'_software_limit_asserted': bool(sw1 & 1<<15),

            'error_channel_0': bool(sw2 & 1<<0),
            'error_channel_1': bool(sw2 & 1<<1),
            'usb_error': bool(sw2 & 1<<2),
            'can_error': bool(sw2 & 1<<4),
            'ethernet_error': bool(sw2 & 1<<6),
            'i2c_running': bool(sw2 & 1<<7),
            'watchdog_event': bool(sw2 & 1<<8),
            'adb_bad_checksum': bool(sw2 & 1<<9),
            'program_running': bool(sw2 & 1<<10),
            'trace_in_progress': bool(sw2 & 1<<11),
            'ee_write_buffer_overflow': bool(sw2 & 1<<12),
            'ee_busy': bool(sw2 & 1<<13),
            'command_error': bool(sw2 & 1<<14),
            'checksum_error': bool(sw2 & 1<<15),

            'position_software_error_limit': bool(sw3 & 1<<0),
            'torque_saturation': bool(sw3 & 1<<1),
            'voltage_saturation': bool(sw3 & 1<<2),
            'wraparound_occurred': bool(sw3 & 1<<3),
            'kg_enabled': bool(sw3 & 1<<4),
            'velocity_direction': bool(sw3 & 1<<5),
            'torque_direction': bool(sw3 & 1<<6),
            'io_fault_latch': bool(sw3 & 1<<7),
            'relative_position_mode': bool(sw3 & 1<<8),
            'x_stop_in_progress': bool(sw3 & 1<<9),
            'peak_current_saturation': bool(sw3 & 1<<10),
            'modulo_rollover': bool(sw3 & 1<<11),
            'brake_asserted': bool(sw3 & 1<<12),
            'brake_ok': bool(sw3 & 1<<13),
            'external_input_go_enabled': bool(sw3 & 1<<14),
            'velocity_target_reached': bool(sw3 & 1<<15),
            }
        return status

    def getStatusTrue(self):
        status = self.getStatus()
        status_true = [key for key in status.keys() if status[key]]
        status_true.sort()
        return status_true

    def getStatusTrueFormatted(self):
        status = self.getStatusTrue()
        # status_formatted = ''
        # for line in status:
        #     status_formatted += line + '\n'
        status_formatted = json.dumps(status,sort_keys=True,separators=(',', ':'))

        return status_formatted

    def getStatusOverCurrent(self):
        response = self.sendCmdGetRsp('RBa')
        return bool(response)

    def getStatusPosError(self):
        response = self.sendCmdGetRsp('RBe')
        return bool(response)

    def getStatusVelError(self):
        response = self.sendCmdGetRsp('RBv')
        return bool(response)

    def getStatusTemperatureError(self):
        response = self.sendCmdGetRsp('RBh')
        return bool(response)

    def getStatusTrajectoryInProgress(self):
        response = self.sendCmdGetRsp('RBt')
        return bool(response)

    def getPidValues(self):
        pid_values = {}
        pid_values['kp'] = self.sendCmdGetRsp('RKP')
        pid_values['ki'] = self.sendCmdGetRsp('RKI')
        pid_values['kd'] = self.sendCmdGetRsp('RKD')
        pid_values['kl'] = self.sendCmdGetRsp('RKL')
        pid_values['ks'] = self.sendCmdGetRsp('RKS')
        pid_values['kv'] = self.sendCmdGetRsp('RKV')
        pid_values['ka'] = self.sendCmdGetRsp('RKA')
        pid_values['kg'] = self.sendCmdGetRsp('RKG')
        return pid_values

    def setPidValues(self,pid_values):
        if 'kp' in pid_values:
            bytes_written = self.sendCmd('KP={0}'.format(pid_values['kp']))
        if 'ki' in pid_values:
            bytes_written = self.sendCmd('KI={0}'.format(pid_values['ki']))
        if 'kd' in pid_values:
            bytes_written = self.sendCmd('KD={0}'.format(pid_values['kd']))
        if 'kl' in pid_values:
            bytes_written = self.sendCmd('KL={0}'.format(pid_values['kl']))
        if 'ks' in pid_values:
            bytes_written = self.sendCmd('KS={0}'.format(pid_values['ks']))
        if 'kv' in pid_values:
            bytes_written = self.sendCmd('KV={0}'.format(pid_values['kv']))
        if 'ka' in pid_values:
            bytes_written = self.sendCmd('KA={0}'.format(pid_values['ka']))
        if 'kg' in pid_values:
            bytes_written = self.sendCmd('KG={0}'.format(pid_values['kg']))
        bytes_written = self.sendCmd('F')
        return bytes_written

    def getModelNumber(self):
        model_number = ''
        model_number += self._getEepromValue(32474,3)
        model_number += self._getEepromValue(32477,1)
        model_number += self._getEepromValue(32481,2)
        return model_number

    def getSerialNumber(self):
        serial_number = ''
        serial_number += self._getEepromValue(32512,1)
        return serial_number


    def _getEepromValue(self,address,length):
        self.sendCmd('xxx=' + str(address))
        self.sendCmd('EPTR=xxx')
        if length == 1:
            self.sendCmd('VLD(dd,{length})'.format(length=length))
            response = self.sendCmdGetRsp('Rdd')
            return str(response)
        elif 1 < length:
            self.sendCmd('VLD(ab[0],{length})'.format(length=length))
            rtn_str = ''
            for index in range(length):
                response = self.sendCmdGetRsp('Rab[{index}]'.format(index=index))
                if response != 0:
                    try:
                        rtn_str += chr(response)
                    except (TypeError,ValueError):
                        rtn_str += str(response)
            return rtn_str
        else:
            return ''

    # From output_unit/s/s
    def _convertAccToMotorUnits(self,acc):
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        print("type(er) = {0}".format(type(er)))
        print("er = {0}".format(er))
        print("type(sr) = {0}".format(type(sr)))
        print("type(sf) = {0}".format(type(sf)))
        acc = acc*(65536*(er/(sr**2))*sf)
        return abs(int(acc))

    # To output_unit/s/s
    def _convertAccFromMotorUnits(self,acc):
        if type(acc) != int:
            raise RuntimeError("acc must be type int, instead {0} is type {1}".format(acc,type(acc)))
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        acc = acc/(65536*(er/(sr**2))*sf)
        return abs(acc)

    # From output_unit/s
    def _convertVelToMotorUnits(self,vel):
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        vel = vel*(65536*(er/sr)*sf)
        return int(vel)

    # To output_unit/s
    def _convertVelFromMotorUnits(self,vel):
        if type(vel) != int:
            raise RuntimeError("vel must be type int, instead {0} is type {1}".format(vel,type(vel)))
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        vel = vel/(65536*(er/sr)*sf)
        return vel

    # From output_unit
    def _convertPosToMotorUnits(self,pos):
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        pos = pos*(er*sf)
        return int(pos)

    # To output_unit
    def _convertPosFromMotorUnits(self,pos):
        if type(pos) != int:
            raise RuntimeError("pos must be type int, instead {0} is type {1}".format(pos,type(pos)))
        er = self.encoder_resolution
        sr = self.sample_rate
        sf = self.scale_factor
        pos = pos/(er*sf)
        return pos


class AnimaticsMotors(list):

    def __init__(self,*args,**kwargs):
        if 'motor_names' in kwargs:
            self.motor_names = kwargs.pop('motor_names')
        else:
            self.motor_names = []
        if 'set_default_mode' in kwargs:
            self.set_default_mode = kwargs.pop('set_default_mode')
        else:
            self.set_default_mode = True

    def getAnimaticsMotorsInfo(self):
        animatics_motors_info = []
        for motor in self:
            if motor.motor_address is not 0:
                animatics_motors_info.append(motor.getAnimaticsMotorInfo())
        return animatics_motors_info

    def areAllHomed(self):
        homed = True
        motor_count = len(self) - 1
        for motor_index in range(1,(motor_count+1)):
            homed = homed and self[motor_index].isHomed()
        return homed

    def address(self,motor_count_limit=MOTOR_COUNT_LIMIT):
        self.sendCmd("WAKE",0)
        time.sleep(0.1)
        self.sendCmd("ECHO",0)
        self.sendCmd("END",0)
        self.sendCmd("SADDR0",0)
        self.sendCmd("ECHO_OFF",0)
        for motor_address in range(motor_count_limit):
            motor_address += 1
            self.sendCmd("SADDR" + str(motor_address),0)
            self.sendCmd("ECHO",motor_address)
            self.sendCmd("SLEEP",motor_address)
        self.sendCmd("WAKE",0)
        time.sleep(0.1)

    def count(self,motor_count_limit=MOTOR_COUNT_LIMIT):
        motor_count = 0
        for motor_address in range(motor_count_limit):
            motor_address += 1
            response = self.sendCmdGetRsp("RPI",motor_address)
            if (0 < len(response)) and (response[0] == '3'):
                print('Found motor {0}!'.format(motor_address))
                motor_count = motor_address
            else:
                break
        if motor_count == 0:
            self.debugPrint("No motors counted!")
        return motor_count

    def initialize(self,motor_count=None):
        if motor_count is None:
            motor_count = self.count()
        for motor_address in range(motor_count+1):
            self.debugPrint('Initializing motor {0}'.format(motor_address))
            motor = AnimaticsMotor(motor_address=motor_address,set_default_mode=self.set_default_mode)
            motor.sendCmd = partial(self.sendCmd,motor_address=motor_address)
            motor.sendCmdGetRsp = partial(self.sendCmdGetRsp,motor_address=motor_address)
            if motor_address is not 0:
                motor.initialize()
            self.append(motor)

    def sortByMotorAddress(self,*args,**kwargs):
        kwargs['key'] = operator.attrgetter('motor_address')
        self.sort(**kwargs)

    def getByMotorAddress(self,motor_address):
        motor_list = []
        for motor_index in range(len(self)):
            motor = self[motor_index]
            if motor.motor_address == motor_address:
                motor_list.append(motor)
        if len(motor_list) == 1:
            return motor_list[0]
        elif 1 < len(motor_list):
            return motor_list

    def sortByMotorAxis(self,*args,**kwargs):
        kwargs['key'] = operator.attrgetter('motor_axis','motor_address')
        self.sort(**kwargs)

    def getByMotorAxis(self,motor_axis):
        motor_list = []
        for motor_index in range(len(self)):
            motor = self[motor_index]
            if motor.motor_axis == motor_axis:
                motor_list.append(motor)
        if len(motor_list) == 1:
            return motor_list[0]
        elif 1 < len(motor_list):
            return motor_list

    def sortByMotorName(self,*args,**kwargs):
        kwargs['key'] = operator.attrgetter('motor_name','motor_address')
        self.sort(**kwargs)

    def getByMotorName(self,motor_name):
        motor_name = str(motor_name)
        motor_list = []
        for motor_index in range(len(self)):
            motor = self[motor_index]
            if motor.motor_name == motor_name:
                motor_list.append(motor)
        if len(motor_list) == 1:
            return motor_list[0]
        elif 1 < len(motor_list):
            return motor_list

    def setParameters(self,parameters_list):
        if len(parameters_list) <= len(self):
            for index in range(len(parameters_list)):
                parameters = parameters_list[index]
                self[index].setParameters(parameters)

    def stop(self):
        self[0].stop()

    def resetErrors(self):
        self[0].resetErrors()


class AnimaticsDevice(SerialDevice):

    BAUDRATE = 9600
    TIMEOUT = 0.05
    WRITE_READ_DELAY = 0.05
    WRITE_WRITE_DELAY = 0.05
    RESET_SLEEP = 1.0

    def __init__(self,*args,**kwargs):
        if 'baudrate' not in kwargs:
            kwargs.update({'baudrate': self.BAUDRATE})
        if 'timeout' not in kwargs:
            kwargs.update({'timeout': self.TIMEOUT})
        if 'write_read_delay' not in kwargs:
            kwargs.update({'write_read_delay': self.WRITE_READ_DELAY})
        if 'write_write_delay' not in kwargs:
            kwargs.update({'write_write_delay': self.WRITE_WRITE_DELAY})
        if 'port' not in kwargs:
            port =  findAnimaticsDevicePort(self.BAUDRATE)
            kwargs.update({'port': port})
        if 'debug' not in kwargs:
            kwargs.update({'debug': DEBUG})
            self.debug = DEBUG
        else:
            self.debug = kwargs['debug']
        if 'set_default_mode' in kwargs:
            self.set_default_mode = kwargs.pop('set_default_mode')
        else:
            self.set_default_mode = True

        t_start = time.time()
        super(AnimaticsDevice,self).__init__(*args,**kwargs)
        atexit.register(self._exitAnimaticsDevice)
        self.motors = AnimaticsMotors(set_default_mode=self.set_default_mode)
        self.motors.sendCmd = self.sendCmd
        self.motors.sendCmdGetRsp = self.sendCmdGetRsp
        self.motors.debugPrint = self.debugPrint
        self.motors.address()
        self.motors.initialize()
        t_end = time.time()
        self.debugPrint('Initialization time =', (t_end - t_start))

    def _exitAnimaticsDevice(self):
        self.stop()

    def getAnimaticsDeviceInfo(self):
        animatics_device_info = self.getSerialDeviceInfo()
        animatics_device_info['motors_info'] = self.motors.getAnimaticsMotorsInfo()
        return animatics_device_info

    def _conditionCmdStr(self,cmd_str,motor_address):
        if motor_address is not None:
            if (0 == len(cmd_str)) or not (MOTOR_ADDRESS_OFFSET <= ord(cmd_str[0]) <= (MOTOR_ADDRESS_OFFSET + MOTOR_ADDRESS_MAX)):
                cmd_str = chr(motor_address + MOTOR_ADDRESS_OFFSET) + cmd_str
        if not (cmd_str.endswith('\r') or cmd_str.endswith(' ')):
            cmd_str += ' '
        return cmd_str

    def sendCmd(self,cmd_str,motor_address=None,suggest=False):

        '''Sends Animatics command to device over serial port and
        returns number of bytes written. Setting suggest=True discards
        the command if it is sent too soon after the previously sent
        command, determined by write_write_delay value.'''

        lock.acquire()
        cmd_str = self._conditionCmdStr(cmd_str,motor_address)
        if (0 < len(cmd_str)) and (MOTOR_ADDRESS_OFFSET <= ord(cmd_str[0]) <= (MOTOR_ADDRESS_OFFSET + MOTOR_ADDRESS_MAX)):
            self.debugPrint(str(ord(cmd_str[0])-MOTOR_ADDRESS_OFFSET) + cmd_str[1:])
        if not suggest:
            bytes_written = self.writeCheckFreq(cmd_str,delay_write=True)
        else:
            try:
                bytes_written = self.writeCheckFreq(cmd_str,delay_write=False)
            except WriteFrequencyError:
                bytes_written = 0
        lock.release()
        return bytes_written

    def sendCmdGetRsp(self,cmd_str,motor_address=None):

        '''Sends Animatics command to device over serial port and
        returns response'''

        lock.acquire()
        cmd_str = self._conditionCmdStr(cmd_str,motor_address)
        if (0 < len(cmd_str)) and (MOTOR_ADDRESS_OFFSET <= ord(cmd_str[0]) <= (MOTOR_ADDRESS_OFFSET + MOTOR_ADDRESS_MAX)):
            self.debugPrint(str(ord(cmd_str[0])-MOTOR_ADDRESS_OFFSET) + cmd_str[1:])
        response = self.writeRead(cmd_str,use_readline=False,check_write_freq=True)
        if (0 < len(response)) and (MOTOR_ADDRESS_OFFSET <= ord(response[0]) <= (MOTOR_ADDRESS_OFFSET + MOTOR_ADDRESS_MAX)):
            self.debugPrint(str(ord(response[0])-MOTOR_ADDRESS_OFFSET) + response[1:])

        # Remove ECHO from response if necessary
        response = response.replace(cmd_str,'')
        try:
            response = int(response)
        except (ValueError,TypeError):
            pass
        lock.release()
        return response

    def stop(self):
        self.motors.stop()

    def resetErrors(self):
        self.motors.resetErrors()


def findAnimaticsDevicePort(baudrate):
    serialPortList = findSerialDevicePorts()
    matchingList = []
    for port in serialPortList:
        try:
            dev = SerialDevice(port=port,
                               baudrate=baudrate,
                               timeout=0.1,
                               write_write_delay=0.1,
                               write_read_delay=0.1,
                               debug=DEBUG)
            dev.flushInput()
            dev.flushOutput()
            dev.writeCheckFreq(chr(MOTOR_ADDRESS_OFFSET)+'WAKE ',delay_write=True)
            time.sleep(0.1)
            dev.writeCheckFreq(chr(MOTOR_ADDRESS_OFFSET)+'WAKE ',delay_write=True)
            time.sleep(0.1)
            dev.flushInput()
            dev.writeCheckFreq(chr(MOTOR_ADDRESS_OFFSET)+'ECHO ',delay_write=True)
            dev.readlines()
            response = dev.writeRead(chr(MOTOR_ADDRESS_OFFSET)+'RPI ',use_readline=False,check_write_freq=True)
            dev.close()
            # Animatics response will either be 'RPI 3.141592741 '
            # or '3.141592741 ' depending on ECHO setting
            # Must enable ECHO so it will work when multiple motors are chained
            response = response.replace(chr(MOTOR_ADDRESS_OFFSET)+'RPI ','')
            if (0 < len(response)) and (response[0] == '3'):
                print("Found an Animatics device!")
                return port
        except serial.SerialException:
            pass

    if len(matchingList) == 0:
        err_str = 'Could not find Animatics device. Check connections and power or specify port.\n'
        err_str += 'Tried ports: ' + str(serialPortList)
        raise RuntimeError(err_str)
    elif 1 < len(matchingList):
        err_str = 'Found more than one Animatics device. Specify port.'
        err_str += 'Matching ports: ' + str(matchingList)
        raise RuntimeError(err_str)
    else:
        return matchingList[0]

# -----------------------------------------------------------------------------------------
if __name__ == '__main__':

    debug = False
    dev = AnimaticsDevice(debug=debug)
