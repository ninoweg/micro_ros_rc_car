# micro_ros_rc_receiver
This projects outlines the process of building a ROS 2 RC receiver using ESP32 and micro-ROS. It aims to design a ROS 2 plug-and-play receiver which can be used to replace any standard RC receiver. The receiver can communicate either over serial using USB-C or Wifi.

## Hardware 
### Basic Setup
The receiver is based on a [Adafruit ESP32 Feather V2](https://www.adafruit.com/product/5400) (any board supported by [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino#supported-boards) can be used). In the future, the Adafruit ESP32 Feather V2 might be replaced by the [Arduino Nano RP2040 Connect](https://docs.arduino.cc/hardware/nano-rp2040-connect) to make use of the additional 6-axis IMU. 

A custom PCB is used to connect at least two (more can be added) 3-pin plugs for connecting e.g. the ESC and servo of a standard RC car. This can be built using cheap perfboards. 

This simple hardware setup already allows us to control a RC car using ROS 2. 

### Advanced Setup
For the advanced setup the receiver includes ports for four [hall sensors](https://cdn-reichelt.de/documents/datenblatt/B400/DATASHEET_SERIE_SS400.pdf) which are used for wheel odometry.

## Software
### Setup
To integrate the micro-ROS receiver into a ROS 2 environment, two things are needed: a micro-ROS agent running on a host computer and the micro-ROS client running on the ESP32. 

#### micro-ROS agent
To get the agent running, simply setup [Docker](https://www.docker.com/) on your host computer and run the following command in a shell:
```
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:[YOUR ROS 2 VERSION] serial --dev [YOUR BOARD PORT] -v6
``` 
This command connects to an micro-ROS client over serial. Check your PORT and ROS 2 VERSION.
Find more information [here](https://hub.docker.com/r/microros/esp-idf-microros/). 
There is also an command for connecting the client over Wifi.

When using WSL2 this [article](https://learn.microsoft.com/en-us/windows/wsl/connect-usb) discribes how to connect a USB device to the Linux distribution. 

#### micro-ROS client
The client is based on [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble) reconnection [example](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino). Install Arduino IDE 2, add [Arduino-ESP32 support](https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/installing.html) and download the micro-ROS library for Arduino following the instructions on [micro-ROS for Arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble#how-to-use-the-precompiled-library). Use the board manage to select your board and port.

Clone this repo and upload the micro_ros_rc_receiver.ino file to your device.

Connect the device over USB to the host where the micro-ROS agent is running.
To check the devices port run `ls /dev/ttyACM*` or `ls /dev/ttyUSB*` (linux) or check your device manager (windows). 