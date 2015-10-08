#!/usr/bin/env python
import roslib; roslib.load_manifest('fs_actuation')
import rospy
import flask
import json
import threading
import copy
import yaml

from fs_actuation import Actuation


actuation = Actuation()

lock = threading.Lock()
fly_data = []
max_data_len = 1000

# Setup puff webserver
app = flask.Flask(__name__)

@app.route('/')
def home():
    gui_port = rospy.get_param('/fs_parameters/webserver_port/gui')
    return('Actuation webserver running! To access GUI, use port {0} instead.'.format(gui_port))

@app.route('/sendCmdGetRsp')
@app.route('/sendCmdGetRsp/<cmd>')
@app.route('/sendCmdGetRsp/<cmd>/<args>')
def sendCmdGetRsp(cmd=None,args=None):
    global fly_data
    lock.acquire()
    rsp = actuation.handleCmd(cmd,args)
    if str(cmd) == 'sendData':
        args_object = yaml.safe_load(args)
        if len(args_object['detections']) > 0:
            if len(fly_data) < max_data_len:
                fly_data.append(args_object)
            else:
                fly_data = []
    lock.release()
    return rsp

@app.route('/getSavedData')
def getSavedData():
    global fly_data
    saved_data = ''
    acquired = lock.acquire()
    if acquired:
        saved_data = json.dumps(fly_data,sort_keys=True,separators=(',',':'))
        fly_data = []
        lock.release()
    return saved_data

def webserver():
    server = 'remote'
    # server = 'local'
    port = rospy.get_param('/fs_parameters/webserver_port/actuation')
    if server == 'local':
        rospy.loginfo(' * using debug server - localhost only')
        app.run(debug=True,port=port)
    else:
        rospy.loginfo(' * using builtin server - remote access possible')
        app.run(host='0.0.0.0',debug=False,port=port)
        # app.run(host='0.0.0.0',debug=True,port=port)


if __name__ == "__main__":
    webserver()


    # Example:
    #
    # http://fly-sorter:5000/sendCmdGetRsp/sendData/{"ndetections":1,"detections":[{"fly_type":"female","fly_id":305,"x":75.911495,"y":643.757884}],"time_acquired":1370541042.512610,"time_sent":1370541042.951996}
    #
