#!/bin/bash
# Copyright (c) 2016-19 Jetsonhacks 
# MIT License
# Download and install the Arduino IDE
# This is a *very* simple script, change the environment 
# variables to your liking

# Edit used for Raspberry Pi 4

echo "Download and Install Arduino for Raspberry PI"
INSTALL_DIR=${HOME}
# Direct Jetson support starts at 1.8.10
ARDUINO_VERSION=1.8.12

# Only download if newer version exists
wget -N https://downloads.arduino.cc/arduino-$ARDUINO_VERSION-linuxarm.tar.xz
tar -C $INSTALL_DIR/ -xvf arduino-${ARDUINO_VERSION}-linuxarm.tar.xz
cd $INSTALL_DIR/arduino-${ARDUINO_VERSION}
sudo ./install.sh
./arduino-linux-setup.sh "$USER"
echo "You can delete the tar file if desired: arduino-"${ARDUINO_VERSION}"-linuxarm.tar.xz"

echo "Install ros_lib into the Arduino Environment"
cd ~/Arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

echo "Installation DONE!"
