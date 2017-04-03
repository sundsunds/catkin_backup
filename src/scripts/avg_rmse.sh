#!/bin/bash

minTime=-1e100
maxTime=1e100

if [ $# -eq 3 ]; then
	minTime=$2
	maxTime=$3
elif [ $# -ne 1 ]; then
	echo Usage: $0 error-file [min-time max-time]
	exit 1
fi

awk "
BEGIN {
	sum = sqSum = ctr = 0;
}

{
	if(\$1 > $minTime && \$1 < $maxTime) {
		sum+=\$2;
		sqSum += \$2*\$2;
		ctr++;
	}
}

END {
	print \"Average:\", (sum / ctr);
	print \"RMSE:\", sqrt(sqSum/ctr);
}
" $1