#!/bin/bash
if [ $# -ne 1 ]; then
	echo Usage: $0 BAG-FILE
	exit 1
fi

#topic="/imufuse/pose_camera"
topic="/imufuse/pose_body"
#topic="/planepose/pose"
#topic="/Trackable/pose" 

rosbag filter $1 temp1.bag "topic == '$topic' and (
	m.pose.orientation.x * m.pose.orientation.x + 
	m.pose.orientation.y * m.pose.orientation.y + 
	m.pose.orientation.z * m.pose.orientation.z + 
	m.pose.orientation.w * m.pose.orientation.w ) > 0.001" > /dev/null
	 
rosbag filter temp1.bag temp2.bag --print "'%f %f %f %f %f' % (t.secs + t.nsecs*1e-9,
	m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)" "0" | \
	sed -e 's/NO MATCH //' | grep -v temp2.bag | awk '
	BEGIN{print "#time yaw pitch roll"}
	function asin(x) { return atan2(x, sqrt(1-x*x)) }
	{
		qx=$3
		qy=$4
		qz=$2
		qw=$5
		print $1, atan2(2*qy*qw-2*qx*qz, 1 - 2*qy*qy - 2*qz*qz), asin(2*qx*qy + 2*qz*qw), atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)
	}'

rm temp1.bag
rm temp2.bag
