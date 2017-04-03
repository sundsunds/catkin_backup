#!/bin/bash
if [ $# -ne 1 ]; then
	echo Usage: $0 BAG-FILE
	exit 1
fi


echo -e "#Time\tX\tY\tZ\tRX\tRY\tRZ\tRW"

#topic1="/planepose/pose"
#topic1="/planepose/pose_var"
topic1="/imufuse/pose_body"
#topic1="/imufuse/pose_camera"
#topic1="/ptam/pose"
topic2="/Trackable/pose"

rosbag filter $1 temp1.bag "topic == '$topic1' or topic == '$topic2'" > /dev/null
rosbag filter --print "'%.10f\t%g\t%g\t%f\t%g\t%g\t%g\t%g' % (m.header.stamp.secs + m.header.stamp.nsecs*1e-9,\
	m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w)" \
	temp1.bag temp2.bag "0" | sed -e 's/NO MATCH //' | grep -v temp2.bag

rm temp1.bag
rm temp2.bag
