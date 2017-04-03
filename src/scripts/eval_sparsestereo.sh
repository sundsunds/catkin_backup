#!/bin/bash

onlyUpdate=0

#datasets=orig
datasets="orig filt"
epsilon=1

function evalFile() {
	run=$1
	dataset=$2
	letter=$3
	suffix=$4
	
	values=`find results -maxdepth 1 -name ${run}_$letter*$dataset.dat | awk 'BEGIN{FS="_"}{print substr($2, 2)}' | sort -u | sort -n`
	
	resFile=eval/${run}_${dataset}$suffix.dat
	if [ \( $onlyUpdate -eq 0 \) -o \( ! -e $resFile \) -o \
		 \( `find results -maxdepth 1 -name ${run}_$letter*$dataset.dat | head -1` -nt $resFile \) ]; then
		echo -n > $resFile
		
		for val in $values; do
			{
				file="results/${run}_$letter${val}_$dataset.dat"
				bpp=`evalstereo -e $epsilon -a -p disp $file 2>/dev/null`
				count=`awk '{count+=$2} END {print count}' $file`
				echo "$val $count $bpp" > tmp_$val.tmp
			} &
		done
		
		wait
		
		for val in $values; do
			cat tmp_$val.tmp >> $resFile
			rm tmp_$val.tmp
		done
		
	else
		echo skipped
	fi	
}

runs=`ls results/*_u*.dat | awk 'BEGIN{FS="_"} /matchops/{next} {print $1}' | sort -u`
 
for run in $runs; do
	run=`basename $run` 

	for dataset in $datasets; do 
		echo $run $dataset
		
		# Evaluate by algorithm and uniqueness value
		evalFile $run $dataset "u" ""
		
		# Join results with matching operations file
		#resFile=eval/${run}_${dataset}.dat
		#matchFile=results/${run}_${dataset}_matchops.dat
		#if [ -f $matchFile ]; then
		#	join $resFile $matchFile > temp.dat
		#	mv -f temp.dat $resFile
		#fi

		# Evaluate feature count with chosen uniqueness values
		if [ $dataset = orig ]; then
			u=.6
		else
			u=.7
		fi
		
		shopt -s extglob
		file=`ls results/${run}_u${u}*_$dataset.dat`
		resFile=eval/${run}_${dataset}_counts.dat
		awk '{print $1, $2}' $file > $resFile
	done
done

runs=`ls results/*_s*.dat | awk 'BEGIN{FS="_"} /matchops/{next} {print $1}' | sort -u`
# Evaluate by algorithm and step width
for run in $runs; do
	run=`basename $run`
	
	for dataset in $datasets; do 
		echo $run $dataset
		
		evalFile $run $dataset "s" "_step"
	done
done

runs=`ls results/*_a*.dat | awk 'BEGIN{FS="_"} /matchops/{next} {print $1}' | sort -u`
# Evaluate by algorithm and adaptivity
for run in $runs; do
	run=`basename $run`
	
	for dataset in $datasets; do 
		echo $run $dataset
		
		evalFile $run $dataset "a" "_adapt"
	done
done
