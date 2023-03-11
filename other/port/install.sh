#!/bin/bash

CURDIR=`pwd`
echo "Your current directory is $CURDIR. This is where the MVSDK software will be installed..."
A=`whoami`
B=`arch`

if [ $A != 'root' ]; then
   echo "You have to be root to run this script"
   echo "Fail !!!"
   exit 1;
fi


cp vision_port.rules /etc/udev/rules.d/

echo "Successful"
echo "Please  restart system  now!!!"
