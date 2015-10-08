#!/usr/bin/env python
from __future__ import print_function, division
import roslib; roslib.load_manifest('fs_actuation')
import rospy
import json
import urllib2

from fs_actuation.srv import AddPuff, AddPuffResponse
from fs_actuation.srv import Cap, CapResponse
from fs_actuation.srv import StartPuffRepeated, StartPuffRepeatedResponse
from fs_actuation.srv import StopPuffRepeated, StopPuffRepeatedResponse
from fs_actuation.srv import StopAllPuffs, StopAllPuffsResponse
from fs_actuation.srv import SetRelayOn, SetRelayOnResponse
from fs_actuation.srv import SetRelayOff, SetRelayOffResponse
from fs_actuation.srv import SetLightsOn, SetLightsOnResponse
from fs_actuation.srv import SetLightsOff, SetLightsOffResponse


class ActuationRosServer(object):

    def __init__(self):
        rospy.init_node('fs_actuation_ros_server',log_level=rospy.INFO)

        actuation_webserver_port = rospy.get_param('/fs_parameters/webserver_port/actuation')
        self.actuation_webserver_url = 'http://localhost:{0}'.format(actuation_webserver_port)

        self.add_puff = rospy.Service('add_puff', AddPuff, self.handle_add_puff)
        self.cap = rospy.Service('cap', Cap, self.handle_cap)
        self.start_puff_repeated = rospy.Service('start_puff_repeated', StartPuffRepeated, self.handle_start_puff_repeated)
        self.stop_puff_repeated = rospy.Service('stop_puff_repeated', StopPuffRepeated, self.handle_stop_puff_repeated)
        self.stop_all_puffs = rospy.Service('stop_all_puffs', StopAllPuffs, self.handle_stop_all_puffs)
        self.set_relay_on = rospy.Service('set_relay_on', SetRelayOn, self.handle_set_relay_on)
        self.set_relay_off = rospy.Service('set_relay_off', SetRelayOff, self.handle_set_relay_off)
        self.set_lights_on = rospy.Service('set_lights_on', SetLightsOn, self.handle_set_lights_on)
        self.set_lights_off = rospy.Service('set_lights_off', SetLightsOff, self.handle_set_lights_off)
        self.main()

    def handle_send_cmd_get_rsp(self,cmd,args):
        url_str = self.actuation_webserver_url + '/sendCmdGetRsp'
        if args != '':
            url_str += "/{0}/{1}".format(cmd,args)
        else:
            url_str += "/{0}".format(cmd)
        rsp = urllib2.urlopen(url_str)
        return rsp

    def handle_add_puff(self,req):
        args_dict = {}
        args_dict['relay'] = req.relay
        args_dict['delay'] = req.delay
        args_dict['duration'] = req.duration
        args = json.dumps(args_dict,sort_keys=True,separators=(',',':'))
        rsp = self.handle_send_cmd_get_rsp('addPuff',args)
        return AddPuffResponse("success")

    def handle_cap(self,req):
        args = ''
        rsp = self.handle_send_cmd_get_rsp('cap',args)
        return CapResponse("success")

    def handle_start_puff_repeated(self,req):
        args_dict = {}
        args_dict['relay'] = req.relay
        args_dict['time_on'] = req.time_on
        args_dict['time_off'] = req.time_off
        args_dict['count'] = req.count
        args = json.dumps(args_dict,sort_keys=True,separators=(',',':'))
        rsp = self.handle_send_cmd_get_rsp('startPuffRepeated',args)
        return StartPuffRepeatedResponse("success")

    def handle_stop_puff_repeated(self,req):
        args_dict = {}
        args_dict['relay'] = req.relay
        args = json.dumps(args_dict,sort_keys=True,separators=(',',':'))
        rsp = self.handle_send_cmd_get_rsp('stopPuffRepeated',args)
        return StopPuffRepeatedResponse("success")

    def handle_stop_all_puffs(self,req):
        args = ''
        rsp = self.handle_send_cmd_get_rsp('stopAllPuffs',args)
        return StopAllPuffsResponse("success")

    def handle_set_relay_on(self,req):
        args_dict = {}
        args_dict['relay'] = req.relay
        args = json.dumps(args_dict,sort_keys=True,separators=(',',':'))
        rsp = self.handle_send_cmd_get_rsp('setRelayOn',args)
        return SetRelayOnResponse("success")

    def handle_set_relay_off(self,req):
        args_dict = {}
        args_dict['relay'] = req.relay
        args = json.dumps(args_dict,sort_keys=True,separators=(',',':'))
        rsp = self.handle_send_cmd_get_rsp('setRelayOff',args)
        return SetRelayOffResponse("success")

    def handle_set_lights_on(self,req):
        args = ''
        rsp = self.handle_send_cmd_get_rsp('setLightsOn',args)
        return SetLightsOnResponse("success")

    def handle_set_lights_off(self,req):
        args = ''
        rsp = self.handle_send_cmd_get_rsp('setLightsOff',args)
        return SetLightsOffResponse("success")

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    actuation_ros_server = ActuationRosServer()
