#!/bin/bash

# Create detached screen
if [ x$1 = x"exec_commands" ]; then
	sleep 1; # Wait for screen

	# Start roscore
	screen -X register 1 "`echo -e 'roscore\r'`"
	screen -X paste 1
	
	# Start pximu
	sleep 1
	screen -X screen
	screen -X register 1 "`echo -e 'roslaunch pximu stereo_pximu.launch\r'`"
	screen -X paste 1

	# Set to manual
	sleep 1
	screen -X screen
	screen -X register 1 "`echo -e 'rosservice call /set_mode MANUAL_DISARMED\r'`"
	screen -X paste 1

	# Start capture
	sleep 1
	screen -X screen
	screen -X register 1 "`echo -e 'capture.sh -d\r'`"
	screen -X paste 1

	# Logging
	sleep 1
	screen -X screen
	screen -X register 1 "`echo -e 'cd ~/data\rrosbag record -b 1500 /capture /imu_raw'`"
	screen -X paste 1
else
	killall screen
	$0 exec_commands &
	screen -p0
fi
