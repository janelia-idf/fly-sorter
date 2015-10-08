#fs

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

fs contains the ROS packages to control the Janelia fly sorter.

##Starting Up Control Software on the Command Line

###Normal Operation

Open a terminal and run

```shell
fs
```

###Debug Software With No Hardware Attached

Open a terminal and run

```shell
fs -n
```

##Controlling Fly Sorter

Open a browser and navigate to http://{hostname}:5050

Substitute hostname (no brackets {}) of computer running control
software on a properly setup network.
e.g. when running everything on one computer http://localhost:5050
e.g. when running on a typical fly sorter setup http://fly-sorter:5050

##Installation

###Computer Operating System

xubuntu-12.04.2-desktop-amd64
md5 hash: 63ce9035e62412c9e5518386acd14ed6

###Install ROS

Open a terminal and run

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install ros-groovy-desktop-full
```

###Create ROS Workspace and Install Repositories

Open a terminal and run

```shell
sudo apt-get install python-rosinstall
mkdir ~/ros
rosws init ~/ros/fs_ws /opt/ros/groovy
source ~/ros/fs_ws/setup.bash
rosws set fs --hg ssh://hg@bitbucket.org/peterpolidoro/fs
rosws set fs_config --hg ssh://hg@bitbucket.org/peterpolidoro/fs_config
rosws update
python ~/ros/fs_ws/fs_config/bash_setup.py
source ~/.bashrc
python ~/ros/fs_ws/fs/install.py
sudo shutdown -r now
```


###Install Extra ROS Pacakges (manually for now, automatically in the future)

Open a terminal and run

```shell
sudo apt-get install ros-groovy-rosbridge-suite
```


###Install Support Python Packages into Virtualenv (manually for now, automatically in the future)

Open a terminal and run

```shell
source $FS_PYTHON_VIRTUALENV/bin/activate
pip install flask --upgrade
```


##Setup

###Compile the ROS Packages

Open a terminal and run

```shell
rosmake fs
```

