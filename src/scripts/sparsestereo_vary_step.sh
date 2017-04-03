#!/bin/bash

for adapt in 0.5 1.0 1.5; do
#for adapt in 12 15 20; do 
	start=1
	end=5

	prefix="exfast-$adapt"

	for((i=1; i<=end; i++)); do
		sparsestereo -a $adapt -m 115 -u 0.5 -s $i -p results/${prefix}_s${i}_orig.dat -i halfres -R
		#sparsestereo -a $adapt -m 115 -u 0.5 -s $i -p results/${prefix}_s${i}_filt.dat -i filtered -R
	done
done
