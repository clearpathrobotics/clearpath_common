#!/bin/bash
#
# Run this as root, from the directory containing it!
#
# USAGE: install.bash [interface] [user] [launch_pkg] [release]
#
#    Ex: sudo ./install.bash eth0 administrator husky_bringup fuerte
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

release=$(ls /opt/ros/ | tail -n1)

if [ $# -gt 3 ]; then
    if [ "$4" != "" ]; then
        release=$4
    fi
fi

echo "Installing using network interface $interface for user $user."
replacements="s/wlan0/$interface/g;s/administrator/$user/g;s/electric/$release/g"
sed "$replacements" < clearpath-start > /usr/sbin/clearpath-start
chmod +x /usr/sbin/clearpath-start
sed "$replacements" < clearpath-stop > /usr/sbin/clearpath-stop
chmod +x /usr/sbin/clearpath-stop
sed "$replacements" < clearpath.conf > /etc/init/clearpath.conf


# Copy files into /etc/ros/${release}/clearpath
mkdir -p /etc/ros/${release}
sed "s/clearpath_bringup/$launch_pkg/g" < clearpath.launch > /etc/ros/${release}/clearpath.launch

echo ". /opt/ros/${release}/setup.bash; export ROS_PACKAGE_PATH=/home/$user/ros:\$ROS_PACKAGE_PATH" > /etc/ros/setup.bash
