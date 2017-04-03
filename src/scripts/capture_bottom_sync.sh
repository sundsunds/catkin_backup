#!/bin/bash

rosrun nodelet nodelet manager &
roslaunch capture capture_double_stereo.launch &
rosrun nodelet nodelet standalone capture/CaptureNodelet -t -g -s -C2,3

killall nodelet

