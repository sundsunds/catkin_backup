#!/bin/bash
if [ $# -ne 1 ]; then
	echo Usage: $0 BAG-FILE
	exit 1
fi

rosbag filter $1 temp1.bag "topic == '/rosout_agg'"
rosbag filter temp1.bag temp2.bag --print "'%f: %s' % (t.secs + t.nsecs*1e-9, m.msg)" "0" \
	| sed -e 's/NO MATCH //' | grep -v temp2.bag

rm temp1.bag
rm temp2.bag
