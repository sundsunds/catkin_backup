#!/bin/bash

. gnuplot_common.sh

gnuplot << EOF
	`gnuplotCommon`
	#set term postscript enhanced eps color dashed linewidth 3 size 6.5cm, 6cm
	#set term wxt persist
	set key bottom right
	
	set xlabel "Average Number of Features"
	set ylabel "Average Repeatability" #2.0, 0.0
	
	set xrange [*:449]
	
	plot "exfast/exfast/eval.dat" using 2:4 with lines ls 1 title "exFAST", \
		"fast/eval.dat" using 2:4 with lines ls 2 title "FAST", \
		"harris/eval.dat" using 2:4 with lines ls 3 lt 5 title "Harris", \
		"exfast/no_avg/eval.dat" using 2:4 with lines ls 6 lt 3 title "FAST with adaptive threshold", \
		"exfast/no_adapt/eval.dat" using 2:4 with lines ls 5 lt 6  title "FAST with averaged center"
		
EOF

convertOutput repeatability.pdf

gnuplot << EOF
	`gnuplotCommon`
	#set term postscript enhanced eps color dashed linewidth 3 size 6.8cm, 6cm
	#set term wxt persist
	
	set xlabel "Average Number of Features"
	set ylabel "Average Clusteredness {/Times-Italic s}" #2.0, 0.0
	
	set xrange [*:449]
	
	plot "exfast/exfast/eval.dat" using 2:3 with lines ls 1 title "exFAST", \
		"fast/eval.dat" using 2:3 with lines ls 2 title "FAST", \
		"harris/eval.dat" using 2:3 with lines ls 3 lt 5 title "Harris", \
		"exfast/no_avg/eval.dat" using 2:3 with lines ls 6 lt 3 title "FAST with adaptive threshold", \
		"exfast/no_adapt/eval.dat" using 2:3 with lines ls 5 lt 6 title "FAST with averaged center"
		
EOF

convertOutput clustering.pdf

#openPDFs
