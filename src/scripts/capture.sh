#!/bin/bash
#rosrun nodelet nodelet standalone capture/CaptureNodelet -a "0.1,0.9,1.5" -g "$@"
rosrun nodelet nodelet standalone capture/CaptureNodelet -a "0.5,0.5,1.5" "$@"
