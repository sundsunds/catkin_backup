#!/bin/bash

for((t=50;t>=5;t--)); do
	procResult=`feature -P -s $t ~/data/lab/lab3_short/0 | tail -1`
	temp=`echo $procResult  | sed -e 's/^[^:]*: //'`
	procTime=`echo $temp  | sed -e 's/;.*//'`
	temp=`echo $procResult  | sed -e 's/^[^;]*; //'`
	procFeatures=`echo $temp  | sed -e 's/^[^:]*: //'`
	
	echo -e $t\\t$procTime\\t$procFeatures
done
