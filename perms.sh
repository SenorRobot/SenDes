#!/bin/bash


DIR=/sys/bus/usb/drivers/SenorRobot/
FILE=`ls $DIR | grep [0-9].*`

echo $FILE
rm /dev/motor_r
rm /dev/motor_l
rm /dev/gyro_yaw
rm /dev/sonars

chmod a+rw $DIR$FILE/motor*
chmod a+r $DIR$FILE/gyro_yaw
chmod a+r /dev/input/event*
chmod a+r $DIR$FILE/sonars


ln -s $DIR$FILE/motor_r /dev/motor_r
ln -s $DIR$FILE/motor_l /dev/motor_l
ln -s $DIR$FILE/sonars /dev/sonars
ln -s $DIR$FILE/gyro_yaw /dev/gyro_yaw


