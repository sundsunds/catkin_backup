#!/bin/bash
launchDir="$HOME/code/nodes/planepose/launch"


if [ $# -ne 1 ]; then
	echo Usage: $0 BAG-FILE
	exit 1
fi

tmpFile=`mktemp`
roslaunch planepose planepose_calib.launch | tee $tmpFile &

while [ `rosservice list | grep "/planepose/set_calibration_mode" | wc -l` -eq 0 ]
do
	sleep 1; # Wait for node to startup
done

rosservice call /planepose/set_calibration_mode true
rosbag play $1
rosservice call /planepose/set_calibration_mode false

killall roslaunch
sleep 3 # Wait for program to terminate

rollOffset=`grep "Roll offset" $tmpFile | sed -e 's/[^\:]*: //g' | sed -e 's/\x1b.*//'`
pitchOffset=`grep "Pitch offset" $tmpFile | sed -e 's/[^\:]*: //g' | sed -e 's/\x1b.*//'`
rm $tmpFile

echo "Roll offset: $rollOffset"
echo "Pitch offset: $pitchOffset"
echo "Updating launch file..."

updateLaunchFile() {
	pitchTmp=`mktemp`
	negRoll=`echo "$rollOffset*(-1)" | bc -l`
	negPitch=`echo "$pitchOffset*(-1)" | bc -l`
	
	sed $launchDir/$1 -e "s/AUTO_PITCH_OFFSET\\\" *value=\\\"[^\\\"]*/AUTO_PITCH_OFFSET\\\" value=\\\"$negPitch/" > $pitchTmp
	sed $pitchTmp -e "s/AUTO_ROLL_OFFSET\\\" *value=\\\"[^\\\"]*/AUTO_ROLL_OFFSET\\\" value=\\\"$negRoll/" > $launchDir/$1
	rm $pitchTmp
}

updateLaunchFile planepose_quad.launch
updateLaunchFile planepose_offline.launch
updateLaunchFile planepose_standalone.launch
