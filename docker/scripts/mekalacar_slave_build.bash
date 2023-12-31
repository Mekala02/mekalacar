#!/usr/bin/env bash

ROS_DISTRO="foxy"
BASE_IMAGE=osrf/ros:$ROS_DISTRO-desktop

cd ..
docker build --network=host -t mekala02/mekalacar:ros2_${ROS_DISTRO}_slave -f Dockerfile.mekalacar_slave \
	--build-arg BASE_IMAGE="$BASE_IMAGE" \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    .