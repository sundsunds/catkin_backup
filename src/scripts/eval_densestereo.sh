#!/bin/bash

echo -n > dense/result.txt

for dir in `ls dense`; do
	if [ -d dense/$dir ]; then
		echo $dir
		echo -n "$dir " >> dense/result.txt
		evalstereo -ad disp dense/$dir >> dense/result.txt
	fi
done
