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
install the repository as a platformIO project:

<p align="left">
  <img width="380" height="230" src="/docs/git_clone_pio.png">
</p>

## Tutorials
* [Linux course for robotic](https://app.theconstructsim.com/courses/linux-for-robotics-40/)
* [Basic C++ for robotic](https://app.theconstructsim.com/courses/59)
* [Beginner ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
* [Getting started micro-ROS](https://docs.vulcanexus.org/en/humble/rst/microros_documentation/getting_started/getting_started.html)

One you are ready with the turorials checks the examples:
* [Publishertion, subcriber and timer](https://github.com/muru-project/muru_kit/tree/main/src/examples/pub_sub_timer)
* [Motor control by subcriber](https://github.com/muru-project/muru_kit/tree/main/src/examples/sub_motor)
* [Encoder publisher](https://github.com/muru-project/muru_kit/tree/main/src/examples/pub_encoder)
* [bluetooth serial connection](https://github.com/muru-project/muru_kit/tree/main/src/examples/bluetooth)
* [encoder, Motor and bluetooth](https://github.com/muru-project/muru_kit/tree/main/src/examples/enc_motor_blue)

One you are ready with the turorials checks the examples:
* [Publishertion, subcriber and timer](https://github.com/muru-project/muru_kit/tree/main/src/examples/pub_sub_timer)
* [Motor control by subcriber](https://github.com/muru-project/muru_kit/tree/main/src/examples/sub_motor)
* [Encoder publisher](https://github.com/muru-project/muru_kit/tree/main/src/examples/pub_encoder)
* [bluetooth serial connection](https://github.com/muru-project/muru_kit/tree/main/src/examples/bluetooth)

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
ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int32 data:\ 0\
```
Turn down the LED high
```
ros2 topic pub /micro_ros_subcriber std_msgs/msg/Int32 data:\ 1\
```
