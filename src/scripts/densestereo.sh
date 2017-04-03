#!/bin/bash

rosrun nodelet nodelet standalone densestereo/DenseStereoNodelet "$@"

#params=$@
#gdb --eval-command "run standalone densestereo/DenseStereoNodelet $params" $ROS_ROOT/../../stacks/nodelet_core/nodelet/bin/nodelet

#valgrind $ROS_ROOT/../../stacks/nodelet_core/nodelet/bin/nodelet standalone densestereo/DenseStereoNodelet "$@"
