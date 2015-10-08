#!/usr/bin/env python
from __future__ import print_function, division
import roslib; roslib.load_manifest('fs_bias_interface')
import rospy
import json
import urllib2
import yaml
import time
from datetime import date

from fs_bias_interface.srv import StartCamera, StartCameraResponse
from fs_bias_interface.srv import StopCamera, StopCameraResponse


class BiasInterface(object):

    def __init__(self):
        rospy.init_node('fs_bias_interface',log_level=rospy.INFO)

        webserver_hostname = rospy.get_param('/fs_parameters/bias_interface/hostname')
        webserver_port = rospy.get_param('/fs_parameters/bias_interface/port')
        self.config_filename = rospy.get_param('/fs_parameters/bias_interface/config_filename')
        self.video_base_path = rospy.get_param('/fs_parameters/bias_interface/video_base_path')
        self.video_filename_ext = rospy.get_param('/fs_parameters/bias_interface/video_filename_ext')
        if self.video_filename_ext.find('.') != 0:
            self.video_filename_ext = '.' + self.video_filename_ext
        self.webserver_url = 'http://{0}:{1}'.format(webserver_hostname,
                                                     webserver_port)

        self.start_camera = rospy.Service('start_camera', StartCamera, self.handle_start_camera)
        self.stop_camera = rospy.Service('stop_camera', StopCamera, self.handle_stop_camera)
        self.main()

    def make_video_path(self,gender):
        today = date.today()
        date_str = "{year}-{month}-{day}".format(year=today.year,
                                                 month=today.month,
                                                 day=today.day)
        localtime = time.localtime()
        time_str = "{hour}-{min}-{sec}".format(hour=localtime.tm_hour,
                                               min=localtime.tm_min,
                                               sec=localtime.tm_sec)
        if ('/' in self.config_filename) or ('/' in self.video_base_path):
            import posixpath as path
        else:
            import ntpath as path
        dirname = "TrainingData-" + date_str
        filename = gender + '-' + date_str + '-' + time_str + self.video_filename_ext
        video_path = path.join(self.video_base_path,dirname,gender,filename)
        return video_path

    def handle_send_cmd_get_rsp(self,cmd,args):
        url_str = self.webserver_url
        if args != '':
            url_str += "/?{0}={1}".format(cmd,args)
        else:
            url_str += "/?{0}".format(cmd)
        try:
            rsp = urllib2.urlopen(url_str)
        except urllib2.URLError:
            rospy.logwarn('Unable to communicate with BIAS webserver! Is it running?')
            rsp = None
        if rsp is not None:
            html = rsp.read()
            rsp_list = yaml.safe_load(html)
            if len(rsp_list) > 0:
                rsp = rsp_list[0]
        return rsp

    def handle_start_camera(self,req):
        # rospy.logwarn("video path = " + self.make_video_path(req.gender))

        rospy.set_param('/fs_data/video_path','None')
        rsp = self.handle_send_cmd_get_rsp('get-status','')
        if (rsp is None) or (not rsp['success']):
            return StartCameraResponse("fail")

        # first make sure camera is stopped properly
        if rsp['value']['capturing']:
            rsp = self.handle_send_cmd_get_rsp('stop-capture','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['capturing']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to stop BIAS capturing!')
                return StartCameraResponse("fail")
        if rsp['value']['logging']:
            rsp = self.handle_send_cmd_get_rsp('disable-logging','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['logging']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to disable BIAS logging!')
                return StartCameraResponse("fail")
        if rsp['value']['connected']:
            rsp = self.handle_send_cmd_get_rsp('disconnect','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['connected']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to disconnect BIAS!')
                return StartCameraResponse("fail")

        # start camera
        if not rsp['value']['connected']:
            rsp = self.handle_send_cmd_get_rsp('connect','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (not rsp['value']['connected']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to connect to BIAS webserver!')
                return StartCameraResponse("fail")
        rsp = self.handle_send_cmd_get_rsp('load-configuration',self.config_filename)
        if (rsp is None) or (not rsp['success']):
            rospy.logwarn("Attempted to load configuration file: " + self.config_filename)
            rospy.logwarn(str(rsp))
            rospy.logwarn('Unable to connect to load BIAS configuration!')
            return StartCameraResponse("fail")
        video_path = self.make_video_path(req.gender)
        rsp = self.handle_send_cmd_get_rsp('set-video-file',video_path)
        if (rsp is None) or (not rsp['success']):
            rospy.logwarn("Attempted to set video file: " + video_path)
            rospy.logwarn(str(rsp))
            rospy.logwarn('Unable to connect to set BIAS video file!')
            return StartCameraResponse("fail")
        rsp = self.handle_send_cmd_get_rsp('enable-logging','')
        rsp = self.handle_send_cmd_get_rsp('get-status','')
        if (rsp is None) or (not rsp['success']) or (not rsp['value']['logging']):
            rospy.logwarn(str(rsp))
            rospy.logwarn('Unable to connect to enable BIAS logging!')
            return StartCameraResponse("fail")
        rsp = self.handle_send_cmd_get_rsp('start-capture','')
        rsp = self.handle_send_cmd_get_rsp('get-status','')
        if (rsp is None) or (not rsp['success']) or (not rsp['value']['capturing']):
            rospy.logwarn(str(rsp))
            rospy.logwarn('Unable to connect to start BIAS capturing!')
            return StartCameraResponse("fail")

        rospy.set_param('/fs_data/video_path',video_path)
        rospy.loginfo("Loaded configuration file: " + self.config_filename + ", saving video file: " + video_path)
        return StartCameraResponse("success")

    def handle_stop_camera(self,req):
        rsp = self.handle_send_cmd_get_rsp('get-status','')
        if (rsp is None) or (not rsp['success']):
            return StopCameraResponse("fail")
        if rsp['value']['capturing']:
            rsp = self.handle_send_cmd_get_rsp('stop-capture','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['capturing']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to stop BIAS capturing!')
                return StopCameraResponse("fail")
        if rsp['value']['logging']:
            rsp = self.handle_send_cmd_get_rsp('disable-logging','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['logging']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to disable BIAS logging!')
                return StopCameraResponse("fail")
        if rsp['value']['connected']:
            rsp = self.handle_send_cmd_get_rsp('disconnect','')
            rsp = self.handle_send_cmd_get_rsp('get-status','')
            if (rsp is None) or (not rsp['success']) or (rsp['value']['connected']):
                rospy.logwarn(str(rsp))
                rospy.logwarn('Unable to disconnect BIAS!')
                return StopCameraResponse("fail")
        return StopCameraResponse("success")

    def main(self):
        rospy.spin()


if __name__ == "__main__":
    bias_interface = BiasInterface()
