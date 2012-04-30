#!/bin/bash

insmod /home/dread/Platform/driver/src/senorusb.ko

DIR=/sys/bus/usb/drivers/SenorRobot/
FILE=`ls $DIR | grep [0-9].*`

echo $FILE
rm /dev/motor_r
rm /dev/motor_l
rm /dev/gyro_yaw

chmod a+rw $DIR$FILE/motor*
chmod a+rw $DIR$FILE/gyro_yaw
ln -s $DIR$FILE/motor_r /dev/motor_r
ln -s $DIR$FILE/motor_l /dev/motor_l
ln -s $DIR$FILE/gyro_yaw /dev/gyro_yaw

chmod a+r /dev/input/event*
