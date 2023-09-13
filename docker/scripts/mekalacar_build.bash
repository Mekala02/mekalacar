#!/usr/bin/env bash

R="r35.1.0"
ROS_DISTRO="foxy"
BASE_IMAGE="dustynv/ros:$ROS_DISTRO-pytorch-l4t-$R"
MID_IMAGE="mekala02/mekalacar:ros2_middle"
MID_R="r35.1.0"


cd ../jetson-containers


./build.sh --base=$BASE_IMAGE --name=$MID_IMAGE zed_wrapper


cd ..


docker build --network=host -t mekala02/mekalacar:ros2_$ROS_DISTRO -f Dockerfile.mekalacar \
	--build-arg BASE_IMAGE="$MID_IMAGE-$MID_R" \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    .