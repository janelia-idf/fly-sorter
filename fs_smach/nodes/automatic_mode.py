#!/usr/bin/env python
from __future__ import division
import roslib; roslib.load_manifest('fs_smach')
import rospy
import smach
import smach_ros
import actionlib

import time
import os
import yaml
import urllib2

from smach_ros import SimpleActionState

from std_msgs.msg import Empty

from fs_smach.msg import RunParameters
from fs_smach.msg import Vial
from fs_smach.msg import RunSummary

from fs_utilities import FileTools

from fs_actionlib.msg import EmptyAction, EmptyGoal
from fs_actionlib.msg import HomeAction, HomeGoal
from fs_actionlib.msg import GoToPosAction, GoToPosGoal

from fs_actuation.srv import AddPuff
from fs_actuation.srv import Cap
from fs_actuation.srv import SetLightsOff
from fs_actuation.srv import SetLightsOn
from fs_actuation.srv import SetRelayOff
from fs_actuation.srv import SetRelayOn
from fs_actuation.srv import StartPuffRepeated
from fs_actuation.srv import StopPuffRepeated

from fs_bias_interface.srv import StartCamera
from fs_bias_interface.srv import StopCamera


file_tools = FileTools()

HARDWARE = rospy.get_param('/fs_smach/hardware')
FLY_SORTER_PARAMETERS = rospy.get_param('/fs_parameters')
ACTIONSTATE_WAIT_TIMEOUT = 200.0

class SetupRunDataPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SETUP_RUN_DATA_PATH')
        run_mode = rospy.get_param('/fs_smach/run_mode')
        run_mode = run_mode.lower()
        file_tools.create_run_data_path(run_mode)

        return 'succeeded'


class SetupVialDataPath(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'],
                             input_keys=['vial_name'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SETUP_VIAL_DATA_PATH')
        file_tools.create_vial_data_path(userdata.vial_name)
        return 'succeeded'


class SaveRunData(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        run_data = {}
        run_mode = rospy.get_param('/fs_smach/run_mode')
        run_mode = run_mode.lower()
        run_data['run_mode'] = run_mode
        file_tools.save_run_data(run_data)

        return 'succeeded'


class SaveVialData(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'],
                             input_keys=['training_gender'])
        actuation_webserver_port = rospy.get_param('/fs_parameters/webserver_port/actuation')
        self.actuation_webserver_url = 'http://localhost:{0}'.format(actuation_webserver_port)

    def execute(self, userdata):
        rospy.loginfo('Executing state SAVE_VIAL_DATA')
        vial_data = {}
        run_mode = rospy.get_param('/fs_smach/run_mode')
        run_mode = run_mode.lower()
        vial_data['run_mode'] = run_mode
        if run_mode == 'sorting':
            url_str = self.actuation_webserver_url + '/getSavedData'
            rsp = urllib2.urlopen(url_str)
            fly_data_json = rsp.read()
            fly_data = yaml.safe_load(fly_data_json)
            vial_data['fly_data'] = fly_data
        elif run_mode == 'training':
            video_path = rospy.get_param('/fs_data/video_path')
            vial_data['video_path'] = video_path
            vial_data['training_gender'] = userdata.training_gender
        file_tools.save_vial_data(vial_data)

        return 'succeeded'


class PublishInitializing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.pub = rospy.Publisher('/fs_status/initializing', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_INITIALIZING')
        self.pub.publish(Empty())
        return 'succeeded'


class PublishInitialized(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.pub = rospy.Publisher('/fs_status/initialized', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_INITIALIZED')
        self.pub.publish(Empty())
        return 'succeeded'


class PublishHibernated(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])
        self.pub = rospy.Publisher('/fs_status/hibernated', Empty)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_HIBERNATED')
        self.pub.publish(Empty())
        return 'succeeded'


class PublishVialRunning(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'],
                             input_keys=['vial'])
        self.pub = rospy.Publisher('/fs_status/vial/running', Vial)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_VIAL_RUNNING')
        self.pub.publish(Vial(vial=userdata.vial))
        return 'succeeded'


class PublishVialFinished(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'],
                             input_keys=['vial'])
        self.pub = rospy.Publisher('/fs_status/vial/finished', Vial)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_VIAL_FINISHED')
        rospy.wait_for_service('/fs_actuation/set_relay_off')
        rospy.wait_for_service('/fs_actuation/stop_puff_repeated')
        set_relay_off = rospy.ServiceProxy('/fs_actuation/set_relay_off',SetRelayOff)
        stop_puff_repeated = rospy.ServiceProxy('/fs_actuation/stop_puff_repeated',
                                                StopPuffRepeated)
        try:
            set_relay_off(relay='unknown')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        relays = ['male_funnel', 'female_funnel', 'unknown_funnel']
        for relay in relays:
            try:
                resp = stop_puff_repeated(relay=relay)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        self.pub.publish(Vial(vial=userdata.vial))
        return 'succeeded'


class TurnOnLights(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TURN_ON_LIGHTS')
        rospy.wait_for_service('/fs_actuation/set_lights_on')
        try:
            set_lights_on = rospy.ServiceProxy('/fs_actuation/set_lights_on', SetLightsOn)
            resp = set_lights_on()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'succeeded'


class TurnOffLights(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state TURN_OFF_LIGHTS')
        rospy.wait_for_service('/fs_actuation/set_lights_off')
        try:
            set_lights_off = rospy.ServiceProxy('/fs_actuation/set_lights_off', SetLightsOff)
            resp = set_lights_off()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'succeeded'


class CheckRunParameters(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['vials_finished','run_vial','aborted','preempted'],
                             input_keys=['run_parameters'],
                             output_keys=['vial_to_run','vial_training_gender'])
        self.vials_to_run = None

    def execute(self, userdata):
        rospy.loginfo('Executing state CHECK_RUN_PARAMETERS')
        if self.vials_to_run is None:
            self.vials_to_run = userdata.run_parameters['vials_to_run']
            self.vial_training_genders = userdata.run_parameters['vial_training_genders']
            self.run_mode = rospy.get_param('/fs_smach/run_mode')
            # rospy.logwarn(self.vials_to_run)
        if len(self.vials_to_run) == 0:
            self.vials_to_run = None
            self.vial_training_genders = None
            return 'vials_finished'
        else:
            vial_number = self.vials_to_run.pop(0)
            userdata.vial_to_run = "vial_" + str(vial_number)
            userdata.vial_training_gender = None
            if self.run_mode == 'training':
                try:
                    userdata.vial_training_gender = self.vial_training_genders.pop("vial_" + str(vial_number))
                except KeyError:
                    pass
            return 'run_vial'


class CoolFlies(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state COOL_FLIES')
        cool_duration = FLY_SORTER_PARAMETERS['cool']['duration_seconds']
        rospy.loginfo('cooling flies for {0} seconds'.format(cool_duration))
        if not HARDWARE:
            cool_duration = 4
        rospy.sleep(cool_duration)
        return 'succeeded'


class EjectPuff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EJECT_PUFF')
        rospy.wait_for_service('/fs_actuation/add_puff')
        relay = 'eject'
        delay = 0
        duration = FLY_SORTER_PARAMETERS['eject']['duration']
        try:
            add_puff = rospy.ServiceProxy('/fs_actuation/add_puff', AddPuff)
            resp = add_puff(relay=relay,
                            delay=delay,
                            duration=duration)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'succeeded'


class CleanTurntable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CLEAN_TURNTABLE')
        rospy.wait_for_service('/fs_actuation/add_puff')
        relays = ['unknown','disk']
        delay = 0
        duration = FLY_SORTER_PARAMETERS['clean']['duration_seconds']
        if not HARDWARE:
            duration = 4
        duration_ms = duration*1000
        try:
            add_puff = rospy.ServiceProxy('/fs_actuation/add_puff', AddPuff)
            for relay in relays:
                resp = add_puff(relay=relay,
                                delay=delay,
                                duration=duration_ms)
            rospy.sleep(duration)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'succeeded'


class RunDispenseSeq(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'],
                             input_keys=['training_gender'])

    def execute(self, userdata):
        rospy.loginfo('Executing state RUN_DISPENSE_SEQ')
        run_mode = rospy.get_param('/fs_smach/run_mode')
        if run_mode.lower() == 'training':
            rospy.wait_for_service('/fs_bias_interface/start_camera')
            try:
                start_camera = rospy.ServiceProxy('/fs_bias_interface/start_camera',StartCamera)
                start_camera(userdata.training_gender)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        if HARDWARE:
            rospy.wait_for_service('/fs_actuation/start_puff_repeated')
            start_puff_repeated = rospy.ServiceProxy('/fs_actuation/start_puff_repeated',
                                                     StartPuffRepeated)
            rospy.wait_for_service('/fs_actuation/set_relay_on')
            set_relay_on = rospy.ServiceProxy('/fs_actuation/set_relay_on',SetRelayOn)
            try:
                set_relay_on(relay='unknown')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            relays = ['male_funnel', 'female_funnel', 'unknown_funnel']
            seq = FLY_SORTER_PARAMETERS['funnel_puff_seq']
            for relay in relays:
                time_on = seq['on']
                time_off = seq['off']
                count = -1
                try:
                    resp = start_puff_repeated(relay=relay,
                                               time_on=time_on,
                                               time_off=time_off,
                                               count=count)
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e
            relays = ['dispense_low', 'dispense_med', 'dispense_high']
            for relay in relays:
                seq = FLY_SORTER_PARAMETERS['dispense_seq'][relay]
                for stage in seq:
                    time_on = stage['on']
                    time_off = stage['off']
                    count = stage['count']
                    try:
                        resp = start_puff_repeated(relay=relay,
                                                   time_on=time_on,
                                                   time_off=time_off,
                                                   count=count)
                        duration = count*(time_on + time_off)/1000.0
                        rospy.sleep(duration)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
        else:
            rospy.sleep(2)
        return 'succeeded'


class WaitAfterDispense(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_AFTER_DISPENSE')
        wait_duration = FLY_SORTER_PARAMETERS['wait_after_dispense']['duration_seconds']
        if not HARDWARE:
            wait_duration = 1
        rospy.sleep(wait_duration)
        return 'succeeded'


class CapVials(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CAP_VIALS')
        rospy.wait_for_service('/fs_actuation/cap')
        try:
            cap = rospy.ServiceProxy('/fs_actuation/cap', Cap)
            resp = cap()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        run_mode = rospy.get_param('/fs_smach/run_mode')
        if run_mode.lower() == 'training':
            rospy.wait_for_service('/fs_bias_interface/stop_camera')
            try:
                stop_camera = rospy.ServiceProxy('/fs_bias_interface/stop_camera',StopCamera)
                stop_camera()
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        return 'succeeded'


class WaitToGetRunParameters(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','hibernate','aborted','preempted'],
                             output_keys=['run_parameters'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_TO_GET_RUN_PARAMETERS')
        rospy.Subscriber("/fs_controls/set_run_parameters", RunParameters, self.run_callback)
        rospy.Subscriber("/fs_controls/hibernate", Empty, self.hibernate_callback)
        self.return_outcome = None
        while self.return_outcome is None:
            rospy.sleep(0.1)
        if self.return_outcome is 'succeeded':
            userdata.run_parameters = self.run_parameters
            rospy.loginfo("run_parameters = " + str(self.run_parameters))
        return self.return_outcome

    def run_callback(self,data):
        vials_to_run_dict = yaml.safe_load(data.vials_to_run)
        vial_training_genders = yaml.safe_load(data.vial_training_genders)
        vials_to_run_list = []
        for vial in vials_to_run_dict:
            if vials_to_run_dict[vial]:
                vials_to_run_list.append(int(vial.replace('vial_','')))
            else:
                try:
                    vial_training_genders.pop(vial)
                except KeyError:
                    pass
        vials_to_run_list.sort()
        self.run_parameters = {}
        self.run_parameters['vials_to_run'] = vials_to_run_list
        self.run_parameters['vial_training_genders'] = vial_training_genders
        self.return_outcome = 'succeeded'

    def hibernate_callback(self,data):
        self.return_outcome = 'hibernate'


def empty_monitor_cb(ud, msg):
    return False


class PublishRunFinished(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded','aborted','preempted'])
        self.pub = rospy.Publisher('/fs_status/run_finished', RunSummary)

    def execute(self, userdata):
        rospy.loginfo('Executing state PUBLISH_RUN_FINISHED')
        self.pub.publish(RunSummary(run_data=file_tools.run_data,
                                    set_data=file_tools.set_data,
                                    fly_sorter_data=file_tools.fly_sorter_data))
        return 'succeeded'


class WaitToRunAgainOrHibernate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['run_again','hibernate','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_TO_RUN_AGAIN_OR_HIBERNATE')
        rospy.Subscriber("/fs_controls/run_again", Empty, self.run_again_callback)
        rospy.Subscriber("/fs_controls/hibernate", Empty, self.hibernate_callback)
        self.return_outcome = None
        while self.return_outcome is None:
            rospy.sleep(0.1)
        return self.return_outcome

    def run_again_callback(self,data):
        self.return_outcome = 'run_again'

    def hibernate_callback(self,data):
        self.return_outcome = 'hibernate'


class EmptyVial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state EMPTY_VIAL')
        client = actionlib.SimpleActionClient('/fs_actionlib/oscillate_input_pos',EmptyAction)
        rospy.wait_for_service('/fs_actuation/set_relay_on')
        set_relay_on = rospy.ServiceProxy('/fs_actuation/set_relay_on',SetRelayOn)
        rospy.wait_for_service('/fs_actuation/set_relay_off')
        set_relay_off = rospy.ServiceProxy('/fs_actuation/set_relay_off',SetRelayOff)
        try:
            set_relay_on(relay='dispense_high')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        client.wait_for_server()
        goal = EmptyGoal()
        client.send_goal(goal)
        client.wait_for_result()
        client.get_result()
        try:
            set_relay_off(relay='dispense_high')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        duration_before = FLY_SORTER_PARAMETERS['ramp']['duration_before_seconds']
        rospy.sleep(duration_before)
        duration_on = FLY_SORTER_PARAMETERS['ramp']['duration_on_seconds']
        try:
            set_relay_on(relay='ramp')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(duration_on)
        try:
            set_relay_off(relay='ramp')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return 'succeeded'


class AutomaticModeSmach(object):
    def __init__(self):
        rospy.init_node('fs_smach_automatic_mode',log_level=rospy.INFO)

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('WAIT_TO_INITIALIZE',
                                   smach_ros.MonitorState("/fs_controls/initialize",
                                                          Empty,
                                                          empty_monitor_cb),
                                   transitions={'invalid':'PUBLISH_INITIALIZING',
                                                'valid':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('PUBLISH_INITIALIZING', PublishInitializing(),
                                   transitions={'succeeded':'INITIALIZE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            self.sm_initialize_output = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_initialize_output:
                smach.StateMachine.add('HOME_OUTPUT',
                                       SimpleActionState('/fs_actionlib/home_output',
                                                         HomeAction,
                                                         goal=HomeGoal(motor_name="output"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'GO_TO_POS_START_OUTPUT',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('GO_TO_POS_START_OUTPUT',
                                       SimpleActionState('/fs_actionlib/go_to_output_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="output",
                                                                          position="origin"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_initialize_input = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_initialize_input:
                smach.StateMachine.add('HOME_INPUT_Y',
                                       SimpleActionState('/fs_actionlib/home_input',
                                                         HomeAction,
                                                         goal=HomeGoal(motor_name="input_y"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'GO_TO_POS_START_INPUT_Y',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('GO_TO_POS_START_INPUT_Y',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_y",
                                                                          position="ready"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'HOME_INPUT_X',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('HOME_INPUT_X',
                                       SimpleActionState('/fs_actionlib/home_input',
                                                         HomeAction,
                                                         goal=HomeGoal(motor_name="input_x"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'GO_TO_POS_START_INPUT_X',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('GO_TO_POS_START_INPUT_X',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_x",
                                                                          position="vial_10"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_initialize_other = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_initialize_other:
                smach.StateMachine.add('TURN_ON_LIGHTS', TurnOnLights(),
                                       transitions={'succeeded':'TURN_ON_TURNTABLE',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('TURN_ON_TURNTABLE',
                                       SimpleActionState('/fs_actionlib/turn_on_turntable',
                                                         EmptyAction),
                                       transitions={'succeeded':'TURN_ON_CAM',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('TURN_ON_CAM',
                                       SimpleActionState('/fs_actionlib/turn_on_cam',
                                                         EmptyAction),
                                       transitions={'succeeded':'CLEAN_TURNTABLE_SETUP',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('CLEAN_TURNTABLE_SETUP', CleanTurntable(),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_initialize = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                   default_outcome='succeeded',
                                                   outcome_map={'succeeded':
                                                                {'INITIALIZE_OUTPUT':'succeeded',
                                                                 'INITIALIZE_INPUT':'succeeded',
                                                                 'INITIALIZE_OTHER':'succeeded',
                                                                 }})
            with self.sm_initialize:
                smach.Concurrence.add('INITIALIZE_OUTPUT',self.sm_initialize_output)
                smach.Concurrence.add('INITIALIZE_INPUT',self.sm_initialize_input)
                smach.Concurrence.add('INITIALIZE_OTHER',self.sm_initialize_other)

            smach.StateMachine.add('INITIALIZE', self.sm_initialize,
                                   transitions={'succeeded':'PUBLISH_INITIALIZED',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('PUBLISH_INITIALIZED', PublishInitialized(),
                                   transitions={'succeeded':'WAIT_TO_GET_RUN_PARAMETERS',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('WAIT_TO_GET_RUN_PARAMETERS', WaitToGetRunParameters(),
                                   transitions={'succeeded':'RUN',
                                                'hibernate':'HIBERNATE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'run_parameters':'run_parameters'})

            self.sm_run = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                             input_keys=['run_parameters'])
            with self.sm_run:
                smach.StateMachine.add('SETUP_RUN_DATA_PATH', SetupRunDataPath(),
                                       transitions={'succeeded':'CHECK_RUN_PARAMETERS',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('CHECK_RUN_PARAMETERS', CheckRunParameters(),
                                       transitions={'vials_finished':'succeeded',
                                                    'run_vial':'RUN_VIAL',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'},
                                       remapping={'run_parameters':'run_parameters',
                                                  'vial_to_run':'run_vial_data',
                                                  'vial_training_gender':'vial_training_gender'})

                self.sm_run_vial = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                                      input_keys=['run_vial_input','vial_training_gender'])
                with self.sm_run_vial:
                    def go_to_output_pos_goal_cb(userdata, goal):
                        go_to_pos_goal = GoToPosGoal()
                        go_to_pos_goal.motor_name = 'output'
                        output_pos = userdata.vial_to_run.replace("vial","output")
                        go_to_pos_goal.position = output_pos
                        return go_to_pos_goal
                    def go_to_input_pos_goal_cb(userdata, goal):
                        go_to_pos_goal = GoToPosGoal()
                        go_to_pos_goal.motor_name = 'input_x'
                        go_to_pos_goal.position = userdata.vial_to_run
                        return go_to_pos_goal
                    smach.StateMachine.add('PUBLISH_VIAL_RUNNING', PublishVialRunning(),
                                           transitions={'succeeded':'SETUP_VIAL_DATA_PATH',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'vial':'run_vial_input'})
                    smach.StateMachine.add('SETUP_VIAL_DATA_PATH', SetupVialDataPath(),
                                           transitions={'succeeded':'SETUP_INPUT_AND_OUTPUT',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'vial_name':'run_vial_input'})
                    self.sm_setup_input = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                                             input_keys=['vial_to_run'])
                    with self.sm_setup_input:
                        smach.StateMachine.add('INITIALIZE_INPUT_Y',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal=GoToPosGoal(motor_name="input_y",
                                                                                  position="ready"),
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                               transitions={'succeeded':'GO_TO_INPUT_VIAL_POSITION',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})
                        smach.StateMachine.add('GO_TO_INPUT_VIAL_POSITION',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal_cb=go_to_input_pos_goal_cb,
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT),
                                                                 input_keys=['vial_to_run']),
                                               transitions={'succeeded':'succeeded',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'},
                                               remapping={'vial_to_run':'vial_to_run'})
                    self.sm_setup_output = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                                              input_keys=['vial_to_run'])
                    with self.sm_setup_output:
                        smach.StateMachine.add('GO_TO_OUTPUT_VIAL_POSITION',
                                               SimpleActionState('/fs_actionlib/go_to_output_pos',
                                                                 GoToPosAction,
                                                                 goal_cb=go_to_output_pos_goal_cb,
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT),
                                                                 input_keys=['vial_to_run']),
                                               transitions={'succeeded':'succeeded',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'},
                                               remapping={'vial_to_run':'vial_to_run'})
                    self.sm_setup_input_and_output = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                                       default_outcome='succeeded',
                                                                       input_keys=['vial_to_run'],
                                                                       outcome_map={'succeeded':
                                                                                    {'SETUP_INPUT':'succeeded',
                                                                                     'SETUP_OUTPUT':'succeeded',
                                                                                     }})
                    with self.sm_setup_input_and_output:
                        smach.Concurrence.add('SETUP_INPUT',self.sm_setup_input)
                        smach.Concurrence.add('SETUP_OUTPUT',self.sm_setup_output)
                    smach.StateMachine.add('SETUP_INPUT_AND_OUTPUT', self.sm_setup_input_and_output,
                                           transitions={'succeeded':'INSERT_VIAL',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'vial_to_run':'run_vial_input'})
                    smach.StateMachine.add('INSERT_VIAL',
                                           SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                             GoToPosAction,
                                                             goal=GoToPosGoal(motor_name="input_y",
                                                                              position="vial_inserted"),
                                                             server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                           transitions={'succeeded':'COOL_FLIES',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})
                    smach.StateMachine.add('COOL_FLIES', CoolFlies(),
                                           transitions={'succeeded':'REMOVE_VIAL',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})
                    smach.StateMachine.add('REMOVE_VIAL',
                                           SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                             GoToPosAction,
                                                             goal=GoToPosGoal(motor_name="input_y",
                                                                              position="ready"),
                                           server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                           transitions={'succeeded':'GO_TO_DISPENSE',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})
                    smach.StateMachine.add('GO_TO_DISPENSE',
                                           SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                             GoToPosAction,
                                                             goal=GoToPosGoal(motor_name="input_x",
                                                                              position="dispense"),
                                                             server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                           transitions={'succeeded':'PLACE_VIAL_IN_DISPENSE',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})
                    smach.StateMachine.add('PLACE_VIAL_IN_DISPENSE',
                                           SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                             GoToPosAction,
                                                             goal=GoToPosGoal(motor_name="input_y",
                                                                              position="dispense"),
                                                             server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                           transitions={'succeeded':'RUN_DISPENSE_SEQ',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})
                    smach.StateMachine.add('RUN_DISPENSE_SEQ', RunDispenseSeq(),
                                           transitions={'succeeded':'EMPTY_VIAL',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'training_gender':'vial_training_gender'})
                    smach.StateMachine.add('EMPTY_VIAL', EmptyVial(),
                                           transitions={'succeeded':'WAIT_CAP_RETURN_VIAL',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'})

                    self.sm_wait_and_cap = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
                    with self.sm_wait_and_cap:
                        smach.StateMachine.add('WAIT_AFTER_DISPENSE', WaitAfterDispense(),
                                               transitions={'succeeded':'CAP_VIALS',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})
                        smach.StateMachine.add('CAP_VIALS', CapVials(),
                                               transitions={'succeeded':'succeeded',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})

                    self.sm_return_vial = smach.StateMachine(outcomes=['succeeded','aborted','preempted'],
                                                             input_keys=['run_vial_input'])
                    with self.sm_return_vial:
                        smach.StateMachine.add('REMOVE_VIAL_FROM_DISPENSE',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal=GoToPosGoal(motor_name="input_y",
                                                                                  position="ready"),
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                               transitions={'succeeded':'RETURN_TO_VIAL_POSITION',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})
                        smach.StateMachine.add('RETURN_TO_VIAL_POSITION',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal_cb=go_to_input_pos_goal_cb,
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT),
                                                                 input_keys=['vial_to_run']),
                                               transitions={'succeeded':'REPLACE_VIAL',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'},
                                               remapping={'vial_to_run':'run_vial_input'})
                        smach.StateMachine.add('REPLACE_VIAL',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal=GoToPosGoal(motor_name="input_y",
                                                                                  position="vial_inserted"),
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                               transitions={'succeeded':'EJECT_PUFF',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})
                        smach.StateMachine.add('EJECT_PUFF', EjectPuff(),
                                               transitions={'succeeded':'LEAVE_VIAL',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})
                        smach.StateMachine.add('LEAVE_VIAL',
                                               SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                                 GoToPosAction,
                                                                 goal=GoToPosGoal(motor_name="input_y",
                                                                                  position="ready"),
                                                                 server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                               transitions={'succeeded':'succeeded',
                                                            'aborted':'aborted',
                                                            'preempted':'preempted'})

                    self.sm_wait_cap_return_vial = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                                     default_outcome='succeeded',
                                                                     input_keys=['run_vial_input'],
                                                                     outcome_map={'succeeded':
                                                                                  {'WAIT_AND_CAP':'succeeded',
                                                                                   'RETURN_VIAL':'succeeded',
                                                                                   }})
                    with self.sm_wait_cap_return_vial:
                        smach.Concurrence.add('WAIT_AND_CAP',self.sm_wait_and_cap)
                        smach.Concurrence.add('RETURN_VIAL',self.sm_return_vial,
                                              remapping={'run_vial_input':'run_vial_input'})

                    smach.StateMachine.add('WAIT_CAP_RETURN_VIAL', self.sm_wait_cap_return_vial,
                                           transitions={'succeeded':'SAVE_VIAL_DATA',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'run_vial_input':'run_vial_input'})

                    smach.StateMachine.add('SAVE_VIAL_DATA', SaveVialData(),
                                           transitions={'succeeded':'PUBLISH_VIAL_FINISHED',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'training_gender':'vial_training_gender'})

                    smach.StateMachine.add('PUBLISH_VIAL_FINISHED', PublishVialFinished(),
                                           transitions={'succeeded':'succeeded',
                                                        'aborted':'aborted',
                                                        'preempted':'preempted'},
                                           remapping={'vial':'run_vial_input'})

                smach.StateMachine.add('RUN_VIAL', self.sm_run_vial,
                                       transitions={'succeeded':'CHECK_RUN_PARAMETERS',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'},
                                       remapping={'run_vial_input':'run_vial_data',
                                                  'vial_training_gender':'vial_training_gender'})


            smach.StateMachine.add('RUN', self.sm_run,
                                   transitions={'succeeded':'SAVE_RUN_DATA',
                                                'aborted':'aborted',
                                                'preempted':'preempted'},
                                   remapping={'run_parameters':'run_parameters'})

            smach.StateMachine.add('SAVE_RUN_DATA', SaveRunData(),
                                   transitions={'succeeded':'FINISH_RUN',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            self.sm_finish_run_output = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_finish_run_output:
                finish_run_output_position = 'origin'
                smach.StateMachine.add('GO_TO_POS_FINISH_RUN_OUTPUT',
                                       SimpleActionState('/fs_actionlib/go_to_output_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="output",
                                                                          position=finish_run_output_position),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_finish_run_input = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_finish_run_input:
                smach.StateMachine.add('GO_TO_POS_FINISH_RUN_INPUT_Y',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_y",
                                                                          position="ready"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'GO_TO_POS_FINISH_RUN_INPUT_X',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('GO_TO_POS_FINISH_RUN_INPUT_X',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_x",
                                                                          position="vial_10"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_finish_run = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                  default_outcome='succeeded',
                                                  outcome_map={'succeeded':
                                                               {'FINISH_RUN_OUTPUT':'succeeded',
                                                                'FINISH_RUN_INPUT':'succeeded',
                                                                }})
            with self.sm_finish_run:
                smach.Concurrence.add('FINISH_RUN_OUTPUT',self.sm_finish_run_output)
                smach.Concurrence.add('FINISH_RUN_INPUT',self.sm_finish_run_input)

            smach.StateMachine.add('FINISH_RUN', self.sm_finish_run,
                                   transitions={'succeeded':'PUBLISH_RUN_FINISHED',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('PUBLISH_RUN_FINISHED', PublishRunFinished(),
                                   transitions={'succeeded':'WAIT_TO_RUN_AGAIN_OR_HIBERNATE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('WAIT_TO_RUN_AGAIN_OR_HIBERNATE', WaitToRunAgainOrHibernate(),
                                   transitions={'run_again':'PUBLISH_INITIALIZED',
                                                'hibernate':'HIBERNATE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            self.sm_hibernate_output = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_hibernate_output:
                hibernate_output_position = 'origin'
                smach.StateMachine.add('GO_TO_POS_HIBERNATE_OUTPUT',
                                       SimpleActionState('/fs_actionlib/go_to_output_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="output",
                                                                          position=hibernate_output_position),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_hibernate_input = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_hibernate_input:
                smach.StateMachine.add('GO_TO_POS_HIBERNATE_INPUT_Y',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_y",
                                                                          position="ready"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'GO_TO_POS_HIBERNATE_INPUT_X',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('GO_TO_POS_HIBERNATE_INPUT_X',
                                       SimpleActionState('/fs_actionlib/go_to_input_pos',
                                                         GoToPosAction,
                                                         goal=GoToPosGoal(motor_name="input_x",
                                                                          position="vial_10"),
                                                         server_wait_timeout=rospy.Duration(ACTIONSTATE_WAIT_TIMEOUT)),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_hibernate_other = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
            with self.sm_hibernate_other:
                smach.StateMachine.add('CLEAN_TURNTABLE_HIBERNATE', CleanTurntable(),
                                       transitions={'succeeded':'TURN_OFF_LIGHTS',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('TURN_OFF_LIGHTS', TurnOffLights(),
                                       transitions={'succeeded':'STOP_ALL_MOTORS',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})
                smach.StateMachine.add('STOP_ALL_MOTORS',
                                       SimpleActionState('/fs_actionlib/stop_all_motors',
                                                         EmptyAction),
                                       transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    'preempted':'preempted'})

            self.sm_hibernate = smach.Concurrence(outcomes=['succeeded','aborted','preempted'],
                                                  default_outcome='succeeded',
                                                  outcome_map={'succeeded':
                                                               {'HIBERNATE_INPUT':'succeeded',
                                                                # 'HIBERNATE_OUTPUT':'succeeded',
                                                                'HIBERNATE_OTHER':'succeeded',
                                                                }})
            with self.sm_hibernate:
                # smach.Concurrence.add('HIBERNATE_OUTPUT',self.sm_hibernate_output)
                smach.Concurrence.add('HIBERNATE_INPUT',self.sm_hibernate_input)
                smach.Concurrence.add('HIBERNATE_OTHER',self.sm_hibernate_other)

            smach.StateMachine.add('HIBERNATE', self.sm_hibernate,
                                   transitions={'succeeded':'HIBERNATE_OUTPUT',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('HIBERNATE_OUTPUT', self.sm_hibernate_output,
                                   transitions={'succeeded':'PUBLISH_HIBERNATED',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

            smach.StateMachine.add('PUBLISH_HIBERNATED', PublishHibernated(),
                                   transitions={'succeeded':'WAIT_TO_INITIALIZE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

    def execute(self):
        outcome = self.sm.execute()


if __name__ == '__main__':
    ams = AutomaticModeSmach()
    # sis = smach_ros.IntrospectionServer('fs_smach_automatic_mode', ams.sm, '/SM_ROOT')
    # sis.start()
    ams.execute()
    rospy.spin()
    # sis.stop()
