#!/usr/bin/env bash

R="r35.3.1"
ROS_DISTRO="humble"
BASE_IMAGE="dustynv/ros:$ROS_DISTRO-pytorch-l4t-$R"
MID_IMAGE="mekala02/mekalacar:ros2_middle"
MID_R="r35.4.1"


cd ../jetson-containers


./build.sh --base=$BASE_IMAGE --name=$MID_IMAGE zed


cd ..


docker build --network=host -t mekala02/mekalacar:ros2 -f Dockerfile.mekalacar \
	--build-arg BASE_IMAGE="$MID_IMAGE-$MID_R" \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    .