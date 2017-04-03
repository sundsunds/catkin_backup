#!/bin/bash

if [ $# -ne 1 ]; then
	echo usage $0 INPUT_DIR
	exit 1
fi

#categories="rubbish hover double-hover rectangle circle triangle turn"
categories="rubbish circle triangle"

capture.sh -d -t -g &

for dir in $1/201?-*; do
	echo $dir
	file=`ls $dir/201*.bag`
	rosbag play -r 2 --topics /capture -- $file
	
	i=1
	for cat in $categories; do
		echo $i: $cat
	((i++))
	done

	chosenCat="x"
	while [ $chosenCat == "x" ]; do
		echo -n ">"
		read choice
		i=1
		for cat in $categories; do
			if [ x$i == x$choice ]; then
				chosenCat=$cat
			fi
			((i++))
		done
	done
	
	echo "Choice: $chosenCat"
	
	if [ ! -d $chosenCat ]; then
		mkdir $chosenCat
	fi
	mv $dir $chosenCat
	
	echo
done

wait
