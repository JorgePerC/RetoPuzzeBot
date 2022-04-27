# RetoPuzzeBot
Repo para la clasede Implementación de robótica inteligente


* [Repo](https://github.com/Manchester-Robotics/ROSApril2022_Students) de Manchester para el curso

## Prerequisites:
    Now you can clone the Virtual Env! :D

## Hackerboard commands:

    sudo systemctl restart puzzlebot.service

    roslaunch ros_deep_learning video_viewer.ros1.launch
    roslaunch ros_deep_learning video_source.ros1.launch

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    rosrun rqt_image_view rqt_image_view, and select the image_raw topic

    killall -9 roscore
    killall -9 rosmaster


## Connecto to the Jetson from another computer:

Before trying any of this, you need to make sure your Jetson has the hardware required to connect and stablish a wireless connection. 
* Configure a hotspot. I recommend setting this to be your default connection, so you can controll it without using a monitor
* Install net-tools on the secondary computer `sudo apt-get install net-tools`
* Connecto to the Jetson hotspot
* Use `ifconfig` to get the device IP. Save it. 

### SSH

SSH allows you to run commands from an external device. It's like a wrapper, because the commands execute on the "local" device, you are just promted with the results while being able to send more commands. 
You need to stablish this connection everytime the Jetson is powered on. 

    ssh puzzlebot@10.42.0.1
    Password: Puzzlebot72

Press yes if asked

### ROS Network

ROS Network allows you to run nodes from different devices and communicate them though the netkork.
This implies that there's a single rosMaster, in this case, we want it to be on the Jetson. Having said that, we need to configure our computer to connect to an external rosMaster.

    export ROS_MASTER_URI=http://10.42.0.1:11311
    export ROS_IP=XXXX.XXXX.my.ip

I recommend writing those values on yout `.bashrc`. Bc this project is going to take a while and I don't wanna write them every time. Tho, take in mind that you won't be able to run ros on your local device if you are not connected to the Jetson, so, when you finish, remmember to delete them from the `.bashrc` file.

## Branches:
* manchester -> Pull all the code from the original repo 
    * Update submodule `git submodule update --init` followed by: `git submodule update --remote --merge`
* hwdev -> develop course activities for Maltab/Python/C++


## Create packages: 
* catkin_create_pkg pkgName std_msgs rospy