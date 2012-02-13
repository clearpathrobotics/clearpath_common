#!/bin/bash
#
# Run this as root, from the directory containing it!
#
# USAGE: install.bash [interface] [user] [launch_pkg]
#
#    Ex: sudo ./install.bash eth0 administrator husky_bringup
#

interface=wlan0

if [ $# -gt 0 ]; then
    if [ "$1" != "" ]; then
        interface=$1
    fi
fi

user=administrator

if [ $# -gt 1 ]; then
    if [ "$2" != "" ]; then
        user=$2
    fi
fi

launch_pkg=clearpath_minimal

if [ $# -gt 2 ]; then
    if [ "$3" != "" ]; then
        launch_pkg=$3
    fi
fi


echo "Installing using network interface $interface for user $user."
replacements="s/wlan0/$interface/g;s/administrator/$user/g"
sed "$replacements" < clearpath-start > /usr/sbin/clearpath-start
chmod +x /usr/sbin/clearpath-start
sed "$replacements" < clearpath-stop > /usr/sbin/clearpath-stop
chmod +x /usr/sbin/clearpath-stop
sed "$replacements" < clearpath.conf > /etc/init/clearpath.conf

# Copy files into /etc/ros/electric/clearpath
mkdir -p /etc/ros
mkdir -p /etc/ros/electric
sed "s/clearpath_bringup/$launch_pkg/g" < clearpath.launch > /etc/ros/electric/clearpath.launch

echo ". /opt/ros/electric/setup.bash; export ROS_PACKAGE_PATH=/home/$user/ros:\$ROS_PACKAGE_PATH" > /etc/ros/setup.bash

