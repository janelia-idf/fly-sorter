#!/usr/bin/env python
import roslib
roslib.load_manifest('fs_computer_admin')
from fs_computer_admin import control
control.cli()

