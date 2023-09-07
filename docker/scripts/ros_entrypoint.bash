#!/bin/bash
set -e

ROS_DISTRO=$(echo $ROS_DISTRO)
source /opt/ros/$ROS_DISTRO/install/setup.bash

exec "$@"