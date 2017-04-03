#!/bin/bash

if [ $# -ne 2 ]; then
	echo "Usage: $0 INPUT-DIR OUTPUT-DIR"
	exit 1
fi

sigma=1
radius=3
noise=5 #percentage

for file in `ls $1/*.png`
do
	echo "Processing $file"
	convert -depth 8 -blur $radiusx$sigma $file $2/temp1.png || exit 1
	convert +noise Random $2/temp1.png $2/temp2.png || exit 1
	composite -blend $noise $2/temp2.png $2/temp1.png $2/temp3.png || exit 1
	convert -colorspace Gray $2/temp3.png $2/`basename $file` || exit 1
done

rm $2/temp[1-3].png
