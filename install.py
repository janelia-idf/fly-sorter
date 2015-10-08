#!/usr/bin/env python
from __future__ import print_function
import os
import os.path
import subprocess


LOCAL = True

# Setup ROS dependencies
rosdep_list_default = "/etc/ros/rosdep/sources.list.d/20-default.list"
if os.path.exists(rosdep_list_default):
    shellstr = "sudo rm " + rosdep_list_default
    subprocess.check_call(shellstr,shell=True)
shellstr = "sudo rosdep init"
subprocess.check_call(shellstr,shell=True)
rosdep_list_custom = "/etc/ros/rosdep/sources.list.d/11-custom.list"
if os.path.exists(rosdep_list_custom):
    shellstr = "sudo rm " + rosdep_list_custom
    subprocess.check_call(shellstr,shell=True)
if not LOCAL:
    shellstr = 'echo "yaml https://bitbucket.org/api/1.0/repositories/peterpolidoro/fs/raw/tip/rosdep.yaml" | sudo tee ' + rosdep_list_custom
else:
    rosdep_yaml = 'yaml file://' + os.path.join(os.environ['FS_NAME'],'rosdep.yaml')
    shellstr = 'echo ' + rosdep_yaml + ' | sudo tee ' + rosdep_list_custom
subprocess.check_call(shellstr,shell=True)
shellstr = "rosdep update"
subprocess.check_call(shellstr,shell=True)


# Install command line control scripts into ~/bin
script_dict = {'fs': 'fs_computer_admin/scripts/fs_control'}
bin_dir = os.path.join(os.environ['HOME'],'bin')

print('installing scripts into ~/bin')
for name, script in script_dict.iteritems():
    src = os.path.join(os.environ['FS_NAME'],script)
    dst = os.path.join(bin_dir,name)
    if os.path.islink(dst):
        print('removing symbolic link {0}'.format(dst))
        os.unlink(dst)
    print('  {0} -> {1}'.format(src,dst))
    os.symlink(src,dst)


# Setup virtual env
virtualenv = os.environ['FS_PYTHON_VIRTUALENV']
if os.path.exists(virtualenv):
    print("Removing " + virtualenv)
    shellstr = "rm -rf " + virtualenv
    subprocess.check_call(shellstr,shell=True)
shellstr = "virtualenv --system-site-packages " + virtualenv
subprocess.check_call(shellstr,shell=True)

