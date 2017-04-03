#!/bin/bash
if [ $# -ne 1 ]; then
	echo Usage: $0 INPUT-FILE
	exit 1
fi

awk '
BEGIN {filtered = 0; total = 0}

{
	total++;

	dx1=lastX2 - lastX1;
	dy1=lastY2 - lastY1;
	dz1=lastZ2 - lastZ1;
	
	dx2=lastX2 - $2;
	dy2=lastY2 - $3;
	dz2=lastZ2 - $4;
	
	if(sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1) > sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2))
		filtered++;
	else print lastT1, lastX1, lastY1, lastZ1
		
	lastT2 = lastT1;
	lastX2 = lastX1;
	lastY2 = lastY1;
	lastZ2 = lastZ1;
		
	lastT1 = $1;
	lastX1 = $2;
	lastY1 = $3;
	lastZ1 = $4;
}

END { print "# Filtered " filtered " of " total " (" (100*filtered/total) "%)"; }
' $1
