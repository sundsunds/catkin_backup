#!/bin/bash

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

parallel=1


i=`echo "scale=4; $start" | bc | awk '{printf "%.3f\n", $0}'`

while true; do
	if [ $parallel -eq 1 ]; then
		mkdir -p dense/$i || exit
		stereo -u $i -d dense/$i halfres &
		
		((j++))
		
		if((j%4 == 0)); then
			wait
		fi
	else
		stereo -u $i -d dense/$u halfres &
		
		wait
	fi
	
	if [ `echo "$i < $sec1"| bc` -eq 1 ]; then
		i=`echo "scale=4; $i + $inc1" | bc | awk '{printf "%.3f\n", $0}'`
	elif [ `echo "$i < $sec2"| bc` -eq 1 ]; then
		i=`echo "scale=4; $i + $inc2" | bc | awk '{printf "%.3f\n", $0}'`
	else
		i=`echo "scale=4; $i + $inc3" | bc | awk '{printf "%.3f\n", $0}'`
	fi
		
	if [ `echo "$i > $end"| bc` -eq 1 ]; then
		break
	fi
done
