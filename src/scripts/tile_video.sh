#!/bin/bash

numThreads=4

if [ $# -ne 2 ]; then
	echo "Usage: $0 INPUT-DIR VIDEO-FILE"
	exit 1;
fi

mkdir tiles
tmpdir="tiles"
#tmpdir=`mktemp -d -p .`

process=0
double=0

if [ `ls $1/*c3*.png $1/*c3*.pgm 2>/dev/null | wc -l` -gt 0 ]; then
	double=1
fi

i=0
for file in `ls $1/*.png $1/*.pgm 2>/dev/null`; do
	echo $file

	# Collect files
	if [ x$file1 = x ]; then
		file1=$file
	elif [ x$file2 = x ]; then
		file2=$file
		if [ $double -ne 1 ]; then
			process=1
		fi
	elif [ $double -eq 1 ]; then
		if [ x$file3 = x ]; then
			file3=$file
		elif [ x$file4 = x ]; then
			file4=$file
			process=1
		fi
	fi
	
	if [ $process -eq 1 ]; then
		if [ $double -eq 1 ]; then
			montage -background black -geometry +2+2 $file1 $file2 $file3 $file4 $tmpdir/$i.bmp &
		else
			#montage -background black -geometry +2+1 $file1 $file2 $tmpdir/$i.bmp &
			montage -background black -geometry 655x480 $file1 $file2 $tmpdir/$i.bmp &
		fi
		
		echo Frame: $i: $file1 $file2
		
		unset file1 file2 file3 file4
		process=0
		((i++))
		
		((thread=i%numThreads))
		if [ $thread -eq 0 ]; then
			wait
		fi
	fi
done

wait

ffmpeg -r 30 -i "$tmpdir/%d.bmp" -qmax 5 -r 30 $2

#rm -Rf $tmpdir
