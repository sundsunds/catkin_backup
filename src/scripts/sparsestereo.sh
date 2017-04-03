#!/bin/bash

rosrun nodelet nodelet standalone sparsestereo/SparseStereoNodelet "$@"

#params=$@
#gdb --eval-command "run standalone sparsestereo/SparseStereoNodelet $params" $ROS_ROOT/../../stacks/nodelet_core/nodelet/bin/nodelet

#valgrind $ROS_ROOT/../../stacks/nodelet_core/nodelet/bin/nodelet standalone sparsestereo/SparseStereoNodelet "$@"
