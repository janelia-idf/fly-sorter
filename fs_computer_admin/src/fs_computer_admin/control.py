#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('fs_computer_admin')
import rospy
import argparse
import subprocess


def automatic(hardware=True):
    """
    Starts the application in automatic mode
    """
    _common('normal',hardware)

def _common(mode,hardware):
    options = {}
    options['mode'] = mode
    options['hardware'] = hardware
    _roslaunch('common.launch',options)

def _roslaunch(launch_file,options={}):
    """
    Runs a roslaunch file.
    """
    try:
        call_list = ['roslaunch', 'fs_launch', launch_file]
        for option in options:
            call_list.append(option + ":=" + str(options[option]))
        subprocess.call(call_list)
    except KeyboardInterrupt:
        return

def cli():
    parser = argparse.ArgumentParser(description='Fly Sorter Control')
    parser.add_argument('-n','--no-hardware',action="store_true",
                        help='set this option for testing when no USB hardware is attached (arduino relay controller and animatics motors).')

    args = parser.parse_args()

    hardware = True
    if args.no_hardware:
        hardware = False
    automatic(hardware)
