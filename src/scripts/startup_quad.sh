#!/bin/bash

waitForNode() {
	sleep 1
	while [ `rosnode list | grep $1 | wc -l` -eq 0 ]; do
		sleep 1
	done
}

# Create detached screen
if [ x$1 = x"exec_commands" ]; then
	sleep 1; # Wait for screen

	# Start roscore
	screen -X register 1 "`echo -e 'roscore\r'`"
	screen -X paste 1

	# Start pximu
	waitForNode rosout
	screen -X screen
	screen -X register 1 "`echo -e 'roslaunch pximu stereo_pximu.launch\r'`"
	screen -X paste 1

	# Set to guided
	waitForNode pximu
	screen -X screen
	screen -X register 1 "`echo -e 'rosservice call /set_mode GUIDED_DISARMED\r'`"
	screen -X paste 1

	# Launch ptam
	sleep 1
	screen -X screen
	#screen -X register 1 "`echo -e 'roslaunch ptam ptam_stereo_quad.launch\r'`"
	screen -X register 1 "`echo -e 'roslaunch ptam ptam_stereo_planepose_quad.launch\r'`"
	screen -X paste 1

	# Waypoints
	waitForNode manager
	screen -X screen
	screen -X register 1 "`echo -e 'roscd waypoints\rroslaunch waypoints waypoints_quad.launch'`"
	screen -X paste 1
	
	# Logging
	sleep 1
	screen -X screen
	screen -X register 1 "`echo -e 'cd ~/data\r~/scripts/log_quad.sh'`"
	screen -X paste 1
else
	killall screen
	$0 exec_commands &
	screen -p0
fi
