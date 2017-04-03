#!/bin/bash

multithread=1

#name="exfast"
#name="exfast-sparse"
#name="exfast-denseR"
name="exfast-denseBM"

start=0.50
end=1.5
inc=0.02

#name="fast"
#start=12
#end=20
#inc=1

#name="harris"
#start=.000002
#end=.000015
#inc=.0000002

step=2
uniq=0.7

function runStereo {
	name=$1
	adapt=$2
	uniq=$3
	dataset=$4
	setname=$5
	firstIter=$6
	
	matchOps=`sparsestereo -R -a $adapt -m 115 -s $step -u $uniq -p results/${name}_a${adapt}_$setname.dat -i $dataset | tail -1 | awk '/Matching/{print $3}'`
	if [ $matchOps -gt 0 ] 2>/dev/null; then
		matchFile=results/${name}_${setname}_matchops.dat
		if [ $firstIter -eq 1 ]; then
			echo -n > $matchFile
			export first=0
		fi
		echo $adapt $matchOps >> $matchFile
	fi

}

i=`echo "scale=15; $start" | bc`
j=0
firstIter=1
while true; do
	echo adapt: $i

	if [ $multithread -ne 0 ]; then
		runStereo $name $i $uniq halfres orig $firstIter &
		runStereo $name $i $uniq filtered filt $firstIter &
	else
		runStereo $name $i $uniq halfres orig $firstIter
		runStereo $name $i $uniq filtered filt $firstIter
	fi
	
	((j++))
	
	if((j%4 == 0)); then
		wait
	fi

	i=`echo "scale=15; $i + $inc" | bc`
		
	if [ `echo "$i > $end"| bc` -eq 1 ]; then
		break
	fi
done

wait
