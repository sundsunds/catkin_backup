#!/bin/bash

if [ $# -ne 6 ]; then
	echo Usage: $0 file dt x0 y0 z0 dr
	exit 1
fi

dt=$2
x0=$3
y0=$4
z0=$5
r=($6/180.0*pi)

awk "
BEGIN {pi=3.14159;}
{
	if(\$1 > 0)
		print \$1+$dt, cos($r)*\$2 - sin($r)*\$3 - $x0, sin($r)*\$2 + cos($r)*\$3 - $y0, \$4 - $z0;
}" $1
