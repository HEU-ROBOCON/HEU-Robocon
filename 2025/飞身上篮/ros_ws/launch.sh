#!/bin/bash

chmod 777 /dev/ttyUSB0
source ./devel/setup.bash
roslaunch serial_sender serial_sender.launch