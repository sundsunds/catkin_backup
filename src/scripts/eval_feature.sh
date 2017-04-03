#!/bin/bash

if [ $# -ne 1 ]; then
	echo Usage: $0 DIR
	exit 1
fi

for val in `ls $1/feature* | awk '{print gensub("(^.*\\\/)|(feature-)|(\\\.dat)","","g")}' | sort -g`; do
	res=`evalfeature -a -r gauglitz.xml -w groundtruth_warps/unconstrained.warps -s unconstrained $1/feature-$val.dat`
	echo $val $res
done
