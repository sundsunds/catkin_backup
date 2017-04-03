#!/bin/bash

datestr=`date +%F-%H-%M-%S`
mkdir $datestr
pushd $datestr

top -b -d 1 > top-$datestr.log &
rosbag record /capture /imu_raw /imufuse/pose_camera /imufuse/pose_body \
	/imufuse/pose_var /imufuse/orientation_drift /set_pose /rosout_agg \
	/attitude /ptam/pose /ptam/pose_var /planepose/pose /planepose/pose_var \
	/waypoints/visualization

popd
