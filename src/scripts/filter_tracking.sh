#!/bin/bash

if [ $# -ne 1 ]; then
	echo "Usage: $0 FILE"
	exit 1
fi

awk '
BEGIN {
	lastT = -1
	t = 0
	x = 0
	y = 0
	z = 0
	count = 0
	valid = 1
}
{
	t += $1
	x += $2
	y += $3
	z += $4	
	count++
	
	if(lastT != -1 && $1 - lastT > 0.1)
		valid = 0
	lastT = $1
	
	if(count >= 3) {
		print sprintf("%.3f", t/count), x/count, y/count, z/count, valid;
		t = x = y = z = count = 0;
		valid = 1;
	}
}' $1
