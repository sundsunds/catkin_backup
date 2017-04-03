#!/bin/bash
#command="sparsestereo -r $HOME/data/calib-current/front/calib.xml -i $HOME/data/performance -P -f"

#command="sparsestereo -r $HOME/data/2010-10-16_daimler/calib.xml -i $HOME/data/2010-10-16_daimler/2010-10-18_24949_unrect -R -P"
command="sparsestereo -r $HOME/data/2010-10-16_daimler/calib.xml -i $HOME/data/2010-10-16_daimler/2010-10-18_24949_unrect -R -P -f"

start=0.1
end=3
inc=0.1

i=$start
j=0
while true; do
	line=`$command -a $i | sparsestereo_performance.sh -v quiet=1`
	echo $i $line
	
	i=`echo "scale=4; $i + $inc" | bc`
	
	if [ `echo "$i > $end"| bc` -eq 1 ]; then
		break
	fi
done
