import roslib; roslib.load_manifest('fs_utilities')
import rospy

import os
import yaml
import subprocess
import time
from datetime import date
import json


class FileTools(object):
    def __init__(self):
        self.config_path = os.environ['FS_CONFIG']
        self.data_path = os.environ['FS_DATA']
        self.run_data = ""
        self.set_data = ""
        self.fly_sorter_data = ""
        self.run_fly_data = {}

    def read_yaml_file(self,filename):
        """
        Reads the given yaml file and returns a dictionary with its contents
        """
        with open(filename,'r') as f:
            yaml_dict = yaml.safe_load(f)
        return yaml_dict

    def write_yaml_file(self,filename,yaml_dict):
        """
        Writes the given dictionary to the specified yaml file.
        """
        with open(filename,'w') as f:
            # f.write('\n# Autogenerated file - do not hand edit\n\n')
            yaml.dump(yaml_dict, f, default_flow_style=False)

    def get_time_str(self):
        today = date.today()
        date_str = "{year}-{month}-{day}".format(year=today.year,
                                                 month=today.month,
                                                 day=today.day)
        localtime = time.localtime()
        time_str = "{hour}-{min}-{sec}".format(hour=localtime.tm_hour,
                                               min=localtime.tm_min,
                                               sec=localtime.tm_sec)
        return date_str + '-' + time_str

    def save_fly_sorter_data(self,run_mode,run_data):
        fly_sorter_data_filename = os.path.join(self.data_path,"fly_sorter_data.yaml")
        fly_sorter_data = {}
        if not os.path.exists(fly_sorter_data_filename):
            fly_sorter_data['training'] = {}
            fly_sorter_data['sorting'] = {}
            fly_sorter_data[run_mode]['run_count'] = 1
            if run_mode == 'training':
                tds = {}
                genders = ['male','female','mixed','total']
                for gender in genders:
                    tds[gender] = {}
                    tds[gender]['vial_count'] = run_data['training_data_summary'][gender]['vial_count']
                fly_sorter_data[run_mode]['training_data_summary'] = tds
            elif run_mode == 'sorting':
                fly_sorter_data[run_mode]['sorting_data_summary'] = run_data['sorting_data_summary']
        else:
            with open(fly_sorter_data_filename,'r') as f:
                fly_sorter_data = yaml.safe_load(f)
                try:
                    fly_sorter_data[run_mode]['run_count'] += 1
                except KeyError:
                    fly_sorter_data[run_mode]['run_count'] = 1
                if run_mode == 'training':
                    genders = ['male','female','mixed','total']
                    try:
                        tds = fly_sorter_data[run_mode]['training_data_summary']
                    except KeyError:
                        tds = {}
                        for gender in genders:
                            tds[gender] = {'vial_count':0}
                    for gender in genders:
                        tds[gender]['vial_count'] += run_data['training_data_summary'][gender]['vial_count']
                    fly_sorter_data[run_mode]['training_data_summary'] = tds

                elif run_mode == 'sorting':
                    try:
                        sds = fly_sorter_data[run_mode]['sorting_data_summary']
                        for detection_type in sds:
                            sds[detection_type] += run_data['sorting_data_summary'][detection_type]
                        fly_sorter_data[run_mode]['sorting_data_summary'] = sds
                    except KeyError:
                        fly_sorter_data[run_mode]['sorting_data_summary'] = run_data['sorting_data_summary']

        self.write_yaml_file(fly_sorter_data_filename,fly_sorter_data)
        self.fly_sorter_data = json.dumps(fly_sorter_data,sort_keys=True,indent=4,separators=(',', ': '))

    def save_training_data(self,run_data):
        run_data_set_path = rospy.get_param('/fs_data/run_data_set_path')
        training_data_filename = os.path.join(run_data_set_path,"training_data.yaml")
        training_data = {}
        if not os.path.exists(training_data_filename):
            training_data['run_count'] = 1
            training_data['training_data_summary'] = run_data['training_data_summary']
        else:
            with open(training_data_filename,'r') as f:
                training_data = yaml.safe_load(f)
                training_data['run_count'] += 1
                tds_old = training_data['training_data_summary']
                tds_new = run_data['training_data_summary']
                genders = ['male','female','mixed','total']
                for gender in genders:
                    tds_old[gender]['vial_count'] += tds_new[gender]['vial_count']
                    tds_old[gender]['videos'].extend(tds_new[gender]['videos'])
        self.write_yaml_file(training_data_filename,training_data)
        self.set_data = json.dumps(training_data,sort_keys=True,indent=4,separators=(',', ': '))
        self.save_fly_sorter_data('training',run_data)

    def save_sorting_data(self,run_data):
        run_data_set_path = rospy.get_param('/fs_data/run_data_set_path')
        sorting_data_filename = os.path.join(run_data_set_path,"sorting_data.yaml")
        sorting_data = {}
        if not os.path.exists(sorting_data_filename):
            sorting_data['run_count'] = 1
            sorting_data['sorting_data_summary'] = run_data['sorting_data_summary']
        else:
            with open(sorting_data_filename,'r') as f:
                sorting_data = yaml.safe_load(f)
                sorting_data['run_count'] += 1
                sds_old = sorting_data['sorting_data_summary']
                sds_new = run_data['sorting_data_summary']
                for detection_type in sds_old:
                    sds_old[detection_type] += sds_new[detection_type]
        self.write_yaml_file(sorting_data_filename,sorting_data)
        self.set_data = json.dumps(sorting_data,sort_keys=True,indent=4,separators=(',', ': '))
        self.save_fly_sorter_data('sorting',run_data)

    def commit_repository_changes(self,run_mode):
        subprocess.check_call('hg add',cwd=self.data_path,shell=True)
        subprocess.check_call('hg commit -m "Automatic commit. {0} {1} {2}"'.format(self.get_time_str(),
                                                                                    run_mode,
                                                                                    self.run_name),
                              cwd=self.data_path,
                              shell=True)
        ssh_key_path = os.path.expanduser("~/.ssh/auto_push_rsa")
        subprocess.check_call('hg push polidorop -e "ssh -i {0}"'.format(ssh_key_path),
                              cwd=self.data_path,
                              shell=True)
        rospy.loginfo("Commiting fs_data repository changes.")

    def save_run_data(self,run_data):
        run_finish_time_str = self.get_time_str()
        run_finish_time = time.time()
        run_data['run_name'] = self.run_name
        run_data['run_start_time'] = self.run_start_time_str
        run_data['run_finish_time'] = run_finish_time_str
        run_data['run_duration'] = run_finish_time - self.run_start_time
        run_data_filename = os.path.join(self.run_data_path,"run_data.yaml")
        #
        run_data_list = os.listdir(self.run_data_path)
        vials_run = [entry for entry in run_data_list if os.path.isdir(os.path.join(self.run_data_path,entry))]
        vials_run.sort()
        run_data['vials_run'] = vials_run
        run_mode = run_data['run_mode']
        if run_mode == 'training':
            training_data_summary = {}
            training_data_summary['male'] = {'vial_count':0,'videos':[]}
            training_data_summary['female'] = {'vial_count':0,'videos':[]}
            training_data_summary['mixed'] = {'vial_count':0,'videos':[]}
            training_data_summary['total'] = {'vial_count':0,'videos':[]}
        elif run_mode == 'sorting':
            sorting_data_summary = {}
            sorting_data_summary['ndetections_0'] = 0
            sorting_data_summary['ndetections_1'] = 0
            sorting_data_summary['ndetections_multiple'] = 0
            sorting_data_summary['male_count'] = 0
            sorting_data_summary['female_count'] = 0
            sorting_data_summary['unknown_count'] = 0
            sorting_data_summary['vial_count'] = len(self.run_fly_data)
            for vial_name in self.run_fly_data:
                vial_fly_data = self.run_fly_data[vial_name]
                sorting_data_summary['ndetections_0'] += vial_fly_data['ndetections_0']
                sorting_data_summary['ndetections_1'] += vial_fly_data['ndetections_1']
                sorting_data_summary['ndetections_multiple'] += vial_fly_data['ndetections_multiple']
                sorting_data_summary['male_count'] += vial_fly_data['male_count']
                sorting_data_summary['female_count'] += vial_fly_data['female_count']
                sorting_data_summary['unknown_count'] += vial_fly_data['unknown_count']

        vials_data_summary = {}
        for vial in vials_run:
            vial_data_summary = {}
            vial_data_path = os.path.join(self.run_data_path,vial,"vial_data.yaml")
            with open(vial_data_path,'r') as f:
                vial_data_dict = yaml.safe_load(f)
                vial_data_summary['vial_duration'] = vial_data_dict['vial_duration']
                if run_mode == 'training':
                    training_data_vial = {}
                    training_data_vial['gender'] = vial_data_dict['training_gender']
                    training_data_vial['video'] = vial_data_dict['video_path']
                    vial_data_summary['training_data'] = training_data_vial
                    training_data_summary[training_data_vial['gender']]['vial_count'] += 1
                    training_data_summary[training_data_vial['gender']]['videos'].append(training_data_vial['video'])
                    training_data_summary['total']['vial_count'] += 1
                    training_data_summary['total']['videos'].append(training_data_vial['video'])
            vials_data_summary[vial] = vial_data_summary
        #
        if run_mode == 'training':
            run_data['training_data_summary'] = training_data_summary
        elif run_mode == 'sorting':
            run_data['sorting_data_summary'] = sorting_data_summary
            vials_data_summary = self.run_fly_data
        run_data['vials_data_summary'] = vials_data_summary
        self.write_yaml_file(run_data_filename,run_data)
        self.run_data = json.dumps(run_data,sort_keys=True,indent=4,separators=(',', ': '))

        if run_mode == 'training':
            self.save_training_data(run_data)
        elif run_mode == 'sorting':
            self.save_sorting_data(run_data)
            self.run_fly_data = {}

        self.commit_repository_changes(run_mode)

    def save_vial_data(self,vial_data):
        vial_finish_time_str = self.get_time_str()
        vial_finish_time = time.time()
        vial_data['run_name'] = self.run_name
        vial_data['vial_name'] = self.vial_name
        vial_data['vial_start_time'] = self.vial_start_time_str
        vial_data['vial_finish_time'] = vial_finish_time_str
        vial_data['vial_duration'] = vial_finish_time - self.vial_start_time
        vial_data_filename = os.path.join(self.vial_data_path,"vial_data.yaml")
        self.write_yaml_file(vial_data_filename,vial_data)
        if vial_data['run_mode'] == 'sorting':
            fly_data = vial_data['fly_data']
            datum_count = len(fly_data)
            vial_fly_data = {}
            vial_fly_data['ndetections_0'] = 0
            vial_fly_data['ndetections_1'] = 0
            vial_fly_data['ndetections_multiple'] = 0
            vial_fly_data['male_count'] = 0
            vial_fly_data['female_count'] = 0
            vial_fly_data['unknown_count'] = 0
            for fly_datum in vial_data['fly_data']:
                if fly_datum['ndetections'] == 0:
                    vial_fly_data['ndetections_0'] += 1
                elif fly_datum['ndetections'] == 1:
                    vial_fly_data['ndetections_1'] += 1
                    fly_type = fly_datum['detections'][0]['fly_type']
                    if fly_type == 'male':
                        vial_fly_data['male_count'] += 1
                    elif fly_type == 'female':
                        vial_fly_data['female_count'] += 1
                    else:
                        vial_fly_data['unknown_count'] += 1
                else:
                    vial_fly_data['ndetections_multiple'] += 1
            self.run_fly_data[vial_data['vial_name']] = vial_fly_data

    def create_vial_data_path(self,vial_name):
        run_data_path = rospy.get_param('/fs_data/run_data_path')
        rospy.set_param('/fs_data/vial_name',vial_name)
        self.vial_name = vial_name
        self.vial_data_path = os.path.join(run_data_path,vial_name)
        os.mkdir(self.vial_data_path)
        rospy.set_param('/fs_data/vial_data_path',self.vial_data_path)
        self.vial_start_time_str = self.get_time_str()
        rospy.set_param('/fs_data/vial_start_time',self.vial_start_time_str)
        self.vial_start_time = time.time()

    def create_run_data_path(self,run_mode):
        run_data_set_path = self.create_run_data_set_path(run_mode)
        run_data_list = os.listdir(run_data_set_path)
        previous_run_count = 0
        for entry in run_data_list:
            if os.path.isdir(os.path.join(run_data_set_path,entry)):
                previous_run_count += 1
        run_data_dir = 'Run' + str(previous_run_count + 1)
        rospy.set_param('/fs_data/run_name',run_data_dir)
        self.run_name = run_data_dir
        self.run_data_path = os.path.join(run_data_set_path,run_data_dir)
        os.mkdir(self.run_data_path)
        rospy.set_param('/fs_data/run_data_path',self.run_data_path)
        self.run_start_time_str = self.get_time_str()
        rospy.set_param('/fs_data/run_start_time',self.run_start_time_str)
        self.run_start_time = time.time()

    def create_run_data_set_path(self,run_mode):
        today = date.today()
        date_str = "{year}-{month}-{day}".format(year=today.year,
                                                 month=today.month,
                                                 day=today.day)
        rospy.set_param('/fs_data/date',date_str)
        run_data_set_path = os.path.join(self.data_path,date_str,run_mode)
        rospy.set_param('/fs_data/run_data_set_path',run_data_set_path)
        try:
            os.makedirs(run_data_set_path)
        except OSError:
            pass
        return run_data_set_path


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    pass