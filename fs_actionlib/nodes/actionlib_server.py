#! /usr/bin/env python
import roslib; roslib.load_manifest('fs_actionlib')
import rospy
import time
import os

from animatics_device import AnimaticsDevice

import actionlib

import fs_actionlib.msg


debug_sleep_base = 1

class FlySorterAnimaticsDevice(AnimaticsDevice):
    def __init__(self,*args,**kwargs):
        serial_port = rospy.get_param('/fs_parameters/serial_port/animatics')
        self.parameters = rospy.get_param('/fs_parameters/animatics')
        if HARDWARE:
            super(FlySorterAnimaticsDevice,self).__init__(port=serial_port,set_default_mode=False)
            motors_found_count = len(self.motors) - 1
            fly_sorter_motors_count = len(self.parameters) - 1
            if motors_found_count is not fly_sorter_motors_count:
                raise RuntimeError("{0} out of {1} Animatics motors were found, check connections and power.".format(motors_found_count,fly_sorter_motors_count))
            self.motors.setParameters(self.parameters[:(motors_found_count + 1)])
            self.motors.sortByMotorAxis()


class HomeInputAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.HomeFeedback()
    _result   = fs_actionlib.msg.HomeResult()

    def __init__(self):
        self._action_name = 'home_input'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.HomeAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running HomeInputAction on motor {0}'.format(msg.motor_name))

        if HARDWARE:
            self._feedback.count = 0
            motor = fly_sorter_animatics_device.motors.getByMotorName(msg.motor_name)
            motor.homeStart()
            while not motor.homeCompleted():
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.count += 1
                self._as.publish_feedback(self._feedback)
            motor.setHome()
        else:
            rospy.sleep(debug_sleep_base)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

class HomeOutputAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.HomeFeedback()
    _result   = fs_actionlib.msg.HomeResult()

    def __init__(self):
        self._action_name = 'home_output'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.HomeAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running HomeOutputAction on motor {0}'.format(msg.motor_name))

        if HARDWARE:
            self._feedback.count = 0
            motor = fly_sorter_animatics_device.motors.getByMotorName(msg.motor_name)
            motor.homeStart()
            while not motor.homeCompleted():
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.count += 1
                self._as.publish_feedback(self._feedback)
            motor.setHome()
        else:
            rospy.sleep(debug_sleep_base*2)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

class GoToInputPosAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.GoToPosFeedback()
    _result   = fs_actionlib.msg.GoToPosResult()

    def __init__(self):
        self._action_name = 'go_to_input_pos'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.GoToPosAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running GoToInputPosAction on motor {0}, position {1}'.format(msg.motor_name,msg.position))

        if HARDWARE:
            self._feedback.count = 0
            motor = fly_sorter_animatics_device.motors.getByMotorName(msg.motor_name)
            motor.goToPosDict(msg.position)
            while not motor.atPosDict(msg.position):
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.count += 1
                self._as.publish_feedback(self._feedback)
        else:
            rospy.sleep(debug_sleep_base)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class OscillateInputPosAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.GoToPosFeedback()
    _result   = fs_actionlib.msg.GoToPosResult()

    def __init__(self):
        self._action_name = 'oscillate_input_pos'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self.parameters = rospy.get_param('/fs_parameters/oscillate_input')
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running OscillateInputPosAction')

        if HARDWARE:
            self._feedback.count = 0
            motor_y = fly_sorter_animatics_device.motors.getByMotorName('input_y')
            positions_y = ['empty','dispense']
            motor_x = fly_sorter_animatics_device.motors.getByMotorName('input_x')
            positions_x = ['dispense-','dispense+','dispense']
            for cycle in range(self.parameters['cycle_count']*2):
                position_y = positions_y[cycle%len(positions_y)]
                motor_y.goToPosDict(position_y)
                while not motor_y.atPosDict(position_y):
                    r.sleep()
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        success = False
                        break
                    self._feedback.count += 1
                    self._as.publish_feedback(self._feedback)
                if position_y == 'empty':
                    for position_x in positions_x:
                        motor_x.goToPosDict(position_x)
                        while not motor_x.atPosDict(position_x):
                            r.sleep()
                            if self._as.is_preempt_requested():
                                rospy.loginfo('%s: Preempted' % self._action_name)
                                self._as.set_preempted()
                                success = False
                                break
                            self._feedback.count += 1
                            self._as.publish_feedback(self._feedback)
        else:
            rospy.sleep(debug_sleep_base)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class GoToOutputPosAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.GoToPosFeedback()
    _result   = fs_actionlib.msg.GoToPosResult()

    def __init__(self):
        self._action_name = 'go_to_output_pos'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.GoToPosAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running GoToOutputPosAction on motor {0}, position {1}'.format(msg.motor_name,msg.position))

        if HARDWARE:
            self._feedback.count = 0
            motor = fly_sorter_animatics_device.motors.getByMotorName(msg.motor_name)
            motor.goToPosDict(msg.position)
            while not motor.atPosDict(msg.position):
                r.sleep()
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._as.set_preempted()
                    success = False
                    break
                self._feedback.count += 1
                self._as.publish_feedback(self._feedback)
        else:
            rospy.sleep(debug_sleep_base*2)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class TurnOnTurntableAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.EmptyFeedback()
    _result   = fs_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'turn_on_turntable'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running TurnOnTurntableAction')

        if HARDWARE:
            motor = fly_sorter_animatics_device.motors.getByMotorName('turntable')
            motor.goToVel()
        else:
            rospy.sleep(debug_sleep_base)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class TurnOnCamAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.EmptyFeedback()
    _result   = fs_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'turn_on_cam'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running TurnOnCamAction')

        if HARDWARE:
            motor = fly_sorter_animatics_device.motors.getByMotorName('cam')
            motor.goToVel()
        else:
            rospy.sleep(debug_sleep_base)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


class StopAllMotorsAction(object):
    # create messages that are used to publish feedback/result
    _feedback = fs_actionlib.msg.EmptyFeedback()
    _result   = fs_actionlib.msg.EmptyResult()

    def __init__(self):
        self._action_name = 'stop_all_motors'
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                fs_actionlib.msg.EmptyAction,
                                                self.execute_cb,
                                                False)
        self._as.start()

    def execute_cb(self, msg):
        # helper variables
        success = True

        # publish info to the console for the user
        rospy.loginfo('Running StopAllMotorsAction')

        if HARDWARE:
            fly_sorter_animatics_device.stop()
        else:
            rospy.sleep(1)

        if success:
            self._result.status = 'succeeded'
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('fs_actionlib_server',log_level=rospy.INFO)
    HARDWARE = rospy.get_param('/fs_actionlib/fs_actionlib_server/hardware')
    fly_sorter_animatics_device = FlySorterAnimaticsDevice()
    HomeInputAction()
    HomeOutputAction()
    GoToInputPosAction()
    OscillateInputPosAction()
    GoToOutputPosAction()
    TurnOnTurntableAction()
    TurnOnCamAction()
    StopAllMotorsAction()
    rospy.spin()
