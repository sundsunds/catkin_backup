#!/bin/bash

#FAST
start=10
end=50
inc=3

#exFAST
#start=0.1
#end=3
#inc=0.1

#Harris
#start=.0000005
#end=.0005
#mult=1.3

#Shi Tomasi
#start=0.01
#end=0.2
#mult=1.1

i=$start
j=0
while true; do
	feature -s $i -p feature-$i.dat unconstrained &
	((j++))
	if((j%4 == 0)); then
		wait
	fi	
	
	# Fast / exFAST
	i=`echo "scale=4; $i + $inc" | bc`
	
	# Harris / Shi Tomasi
	#i=`echo "scale=7; $i * $mult" | bc`
	
	if [ `echo "$i > $end"| bc` -eq 1 ]; then
		break
	fi
done
