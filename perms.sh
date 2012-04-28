#!/bin/bash

DIR=/sys/bus/usb/drivers/SenorRobot/
FILE=`ls $DIR | grep [0-9].*`

echo $FILE

chmod a+rw $DIR$FILE/motor*
ln -s $DIR$FILE/motor_r /dev/motor_r
ln -s $DIR$FILE/motor_l /dev/motor_l

chmod a+r /dev/input/event*
