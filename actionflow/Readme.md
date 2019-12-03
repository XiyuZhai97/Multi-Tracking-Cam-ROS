# ROS pkg actionflow for DC demo
   * [Set up Environment](#set-up-environment)
      * [<a href="http://wiki.ros.org/melodic/Installation/Ubuntu" rel="nofollow">Install ROS Melodic desktop-Full</a>](#install-ros-melodic-desktop-full)
      * [Create Catkin Workspace](#create-catkin-workspace)
      * [Make package](#make-package)
      * [Set up network](#set-up-network)
   * [Running the Demo](#running-the-demo)
      * [Realtime cars following](#realtime-cars-following)
      * [Seeing Results](#seeing-results)
      * [Control the Autonomous Car](#control-the-autonomous-car)

# Set up Environment
## [Install ROS Melodic desktop-Full](http://wiki.ros.org/melodic/Installation/Ubuntu)
**git and cmake should be installed**
~~~~
sudo apt-get install git cmake
~~~~
## Create Catkin Workspace
Create a ROS Catkin workspace to contain our ROS packages:
~~~~
  mkdir -p ./workspace/catkin_ws/src
  cd ./workspace/catkin_ws
  catkin_make
  # add catkin_ws path to bashrc
  sudo sh -c 'echo "source ~/workspace/catkin_ws/devel/setup.bash" >> ~/.bashrc'
~~~~
Verify:
~~~~
  echo $ROS_PACKAGE_PATH
~~~~
Install ros-serial
~~~~
  sudo apt-get install ros-melodic-rosserial-server
~~~~
## Make package

clone this repo and make this package
copy actionflow to ~/workspace/catkin_ws/src
catkin_make
~~~~
  cp -r ./actionflow ~/workspace/catkin_ws/src
  cd ~/workspace/catkin_ws
  catkin_make
~~~~

## Set up network
~~~~
sudo gedit .bashrc
~~~~
add
~~~~
export ROS_MASTER_URI=http://ROScore:11311
export ROS_IP=thisdeviceIP
export ROS_HOSTNAME=thisdevice
~~~~

# Running the Demo

## Start roscore and rosserial
~~~~
  roscore
  roslaunch rosserial_server socket.launch
~~~~

## Realtime cars following
  
  Connect to camera, 
  
  rosrun local file (local NumberofCars IndexofCamera RefreshRate)
  
  Click four corner and press ESC after each click
  
~~~~
  rosrun actionflow local 17 0 30
~~~~

## Seeing Results
~~~~
  rqt_image_view
~~~~
to see the realtime video
~~~~
  rosrun actionflow dc_demo_realtime.py
~~~~


## Control the Autonomous Car
Confirm it is connected to ROSCORE by seeing if there is Car_19 in rostopic
~~~~
  rostopic list
~~~~
Use Listener to sub what Car_19 heard
~~~~
  rosrun actionflow listener Car_19
~~~~
Running the Control script
~~~~
  rosrun actionflow mbot_testcontrol_key.py
~~~~
press 0 to 5 to control the speed of the Autonomous Car
