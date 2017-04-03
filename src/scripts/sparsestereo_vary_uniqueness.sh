#!/bin/bash

multithread=1

start=0.20
sec1=0.10
sec2=0.30
end=1.0

inc1=0.005
inc2=0.01
inc3=0.025

#start=0.30
#end=1.0
#inc=0.05

step=2
parallel=1 # Cannot be used when collecting matching operations

prefix="harris-%s"
#prefix="exfast-%s-denseR"
#prefix="exfast-%s-denseBM"
#prefix="exfast-%s"
#prefix="exfast-s2-%s"
#prefix="fast-%s"
#prefix="exfast-%s-sparse"

function runStereo {
	name=$1
	adapt=$2
	uniq=$3
	dataset=$4
	setname=$5
	firstIter=$6
	
	matchOps=`sparsestereo -R -a $adapt -m 115 -s $step -u $uniq -p results/${name}_u${uniq}_$setname.dat -i $dataset | tail -1 | awk '/Matching/{print $3}'`
	if [ $matchOps -gt 0 ] 2>/dev/null; then
		matchFile=results/${name}_${setname}_matchops.dat
		if [ $firstIter -eq 1 ]; then
			echo -n > $matchFile
			export first=0
		fi
		echo $uniq $matchOps >> $matchFile
	fi
}


adapt=1.0
#for adapt in 0.5 1.0 1.5; do
#for adapt in 12 15 20; do 
for adapt in .000002 .000005 .000015; do
	name=`printf $prefix $adapt`
	echo $name

	i=`echo "scale=4; $start" | bc`
	j=0
	firstIter=1
	while true; do
		if [ $parallel -eq 1 ]; then
			if [ $multithread -ne 0 ]; then
				runStereo $name $adapt $i halfres orig $firstIter &
				runStereo $name $adapt $i filtered filt $firstIter &
			else
				runStereo $name $adapt $i halfres orig $firstIter
				runStereo $name $adapt $i filtered filt $firstIter
			fi
			
			((j++))
			
			if((j%4 == 0)); then
				wait
			fi
		else
			if [ $multithread -ne 0 ]; then
				runStereo $name $adapt $i halfres orig $firstIter &
				runStereo $name $adapt $i filtered filt $firstIter
			else
				runStereo $name $adapt $i halfres orig $firstIter
				runStereo $name $adapt $i filtered filt $firstIter
			fi
			
			wait
		fi
		
		firstIter=0
		if [ `echo "$i < $sec1"| bc` -eq 1 ]; then
			i=`echo "scale=4; $i + $inc1" | bc`
		elif [ `echo "$i < $sec2"| bc` -eq 1 ]; then
			i=`echo "scale=4; $i + $inc2" | bc`
		else
			i=`echo "scale=4; $i + $inc3" | bc`
		fi
			
		if [ `echo "$i > $end"| bc` -eq 1 ]; then
			break
		fi
	done
	
	wait
done
