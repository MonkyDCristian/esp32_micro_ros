# ESP32_MICRO_ROS

## Description
Repository to learn to use plataformio_microros with the ESP32 DEV KIT miccrocontorller.

## Dependencies
### Software
* Operating system: [Ubuntu 22.04](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* Robotic Frameworks:
  * [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  * [Vulcanexus (base)](https://docs.vulcanexus.org/en/humble/rst/installation/linux_binary_installation.html)
* Microcontroller Framework: [Micro-ROS](https://micro.ros.org)
* IDE: [PlatfotmIO on Visual Studio Code](https://platformio.org/install/ide?install=vscode)

#### PlatfotmIO
* [micro-ROS library](https://github.com/micro-ROS/micro_ros_platformio)

### Hardware
* Microcontroller: [ESP32 DEV KIT](https://www.espressif.com/en/products/devkits/esp32-devkitc)

## Install
```
sudo apt-get install python3-venv
```
install the repository as a platformIO project:

<p align="left">
  <img width="380" height="230" src="/docs/imgs/git_clone_pio.png">
</p>

*Note*: if the micro_ros_platformio doesn't install, comment any ROS pkg source in your .bashrc, like this:
```
# Setup ROS2
#source /opt/ros/humble/setup.bash
#source /opt/vulcanexus/humble/setup.bash
```
And delet the /.pio/libdeps/esp32doit-devkit-v1/micro_ros_platformio folder and restart the computer.

<p align="left">
  <img width="380" height="480" src="/docs/imgs/delet_microros.png">
</p>

Open the proyect with VSC again, the microros_ros_platformio should install correctly.

## Tutorials
* [Linux course for robotic](https://app.theconstructsim.com/courses/linux-for-robotics-40/)
* [Basic C++ for robotic](https://app.theconstructsim.com/courses/59)
* [Beginner ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
* [Getting started micro-ROS](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/overview/)

One you are ready with the turorials checks the examples:
* [Publisher, subscriber and timer](/src/examples/pub_sub_timer)
* [Publisher, subscriber and timer with wifi](/src/examples/pub_sub_timer_wifi)
* [Motor control by subscriber](/src/examples/sub_motor)
* [Encoder publisher](/src/examples/pub_encoder)

# Demo
Go to your platformio.ini file to set the build_src_filter:
```
build_src_filter = +<examples/pub_sub_timer/*> -<.git/> -<.svn/>
```
Now compile and upload the firmware.uf2 file to raspberry PI pico, use the one that is inside the .pio/build/pico/ folder.

Find your serial [device name]:
```
ls /dev/serial/by-id/*
```
Add access to USB port
```
sudo chmod a+wr [device name]
```
Start micro_ros_agent:
```
ros2 run micro_ros_agent micro_ros_agent serial --dev [device name]
```
visualice msgs with ros Qt tools
```
rqt 
```
Turn down the LED low
```
ros2 topic pub /micro_ros_subscriber std_msgs/msg/Int16 data:\ 0\
```
Turn down the LED high
```
ros2 topic pub /micro_ros_subscriber std_msgs/msg/Int16 data:\ 1\
```
