# rc_car
The code and instructions found in this repository outline the process of converting an RC car into a mobile platform that can be easily controlled through ROS. Key hardware elements involved in this conversion include a 1/10 scale RC car, a Raspberry Pi 4 8GB acting as the high-level board computer, and an Arduino Mega 2560 utilized as the VCU.

``` 
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## hardware list
* TRAXXAS Slash schwarz RTR 1/10 2WD Short Course Racing Truck
* Raspberry Pi 4 8GB + USB to M.2 Adapter + SSD 
* Arduino Mega 2560 + custom shield
* Luxonis OAK-D
* hall sensor based odometry
* LIPO battery 14.8V + DC/DC to 5V USB-C for power supply of Raspberry and OAK-D
* LIPO battery 7.4V for RC car
* Multiplexers for battery meter: https://cdn-reichelt.de/documents/datenblatt/A200/LS153_REN.pdf
* Bipolar hall sensor for odometry: https://cdn-reichelt.de/documents/datenblatt/B400/DATASHEET_SERIE_SS400.pdf

