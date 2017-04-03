#!/bin/bash

. gnuplot_common.sh

#for set in orig filt; do
for set in orig; do

if [ $set = orig ]; then
	uniq=0.5
else
	uniq=0.5
fi

uniq=0.5

u1=0.5
u2=1.0
u3=1.5

############################# Bad Matches Percentage ############################

gnuplot << EOF
	`gnuplotCommon`
	#set term wxt persist
	
	set style line 1 lt 1 pt 7 lc rgb "#ff0000"
	set style line 2 lt 1 pt 7 lc rgb "#ff0000"
	set style line 3 lt 1 pt 7 lc rgb "#ff0000"
	
	set style line 4 lt 2 pt 7 lc rgb "#0010a5"
	set style line 5 lt 2 pt 7 lc rgb "#0010a5"
	set style line 6 lt 2 pt 7 lc rgb "#0010a5"
	
	set style line 7 lt 3 pt 7 lc rgb "#0fad00"
	set style line 8 lt 3 pt 7 lc rgb "#0fad00"
	set style line 9 lt 3 pt 7 lc rgb "#0fad00"
	
	set xlabel "Average Number of Matched Features"
	set ylabel "Average Bad Matches Percentage / %"
	
	#`if [ $set = orig ]; then echo set key left top; else echo set key bottom right; fi`
	
	set key bottom right
		
	`if [ $set = orig ]; then echo 'set xrange [500:3000]'; fi`
	`if [ $set = orig ]; then echo 'set yrange [0:5]'; fi`
	
	plot "eval/exfast-${u1}_$set.dat" using (\$2/21 > 1750 ? \$2/21 : 1/0):(100*\$3) with lines ls 1 title "exFAST", \
		"eval/exfast-${u2}_$set.dat" using (\$2/21 > 1100 ? \$2/21 : 1/0):(100*\$3) with lines ls 2 notitle, \
		"eval/exfast-${u3}_$set.dat" using (\$2/21):(100*\$3) with lines ls 3 notitle, \
		"eval/fast-12_$set.dat" using (\$2/21 > 1750 ? \$2/21 : 1/0):(100*\$3) with lines ls 4 title "FAST", \
		"eval/fast-15_$set.dat" using (\$2/21 > 1200 ? \$2/21 : 1/0):(100*\$3) with lines ls 5 notitle, \
		"eval/fast-20_$set.dat" using (\$2/21):(100*\$3) with lines ls 6 notitle, \
		"eval/harris-.000002_$set.dat" using (\$2/21 > 1750 ? \$2/21 : 1/0):(100*\$3) with lines ls 7 title "Harris", \
		"eval/harris-.000005_$set.dat" using (\$2/21 > 1200 ? \$2/21 : 1/0):(100*\$3) with lines ls 8 notitle, \
		"eval/harris-.000015_$set.dat" using (\$2/21):(100*\$3) with lines ls 9 notitle, \
		"" using (1/0):(1/0) lt 0 pt 7 title "u = $uniq", \
		"eval/exfast-${u1}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3)  ls 1 notitle, \
		"eval/exfast-${u2}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 2 notitle, \
		"eval/exfast-${u3}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 3 notitle, \
		"eval/fast-12_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 4 notitle, \
		"eval/fast-15_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 5 notitle, \
		"eval/fast-20_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 6 notitle, \
		"eval/harris-.000002_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 7 notitle, \
		"eval/harris-.000005_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 8 notitle, \
		"eval/harris-.000015_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):(100*\$3) ls 9 notitle
		
EOF

epstopdf output.eps
mv output.pdf bpp_$set.pdf

############################### Clustering ###################################

gnuplot << EOF
	`gnuplotCommon`
	#set term wxt persist
	
	set style line 1 lt 1 pt 7 lc rgb "#ff0000"
	set style line 2 lt 1 pt 7 lc rgb "#ff0000"
	set style line 3 lt 1 pt 7 lc rgb "#ff0000"
	
	set style line 4 lt 2 pt 7 lc rgb "#0010a5"
	set style line 5 lt 2 pt 7 lc rgb "#0010a5"
	set style line 6 lt 2 pt 7 lc rgb "#0010a5"
	
	set style line 7 lt 3 pt 7 lc rgb "#0fad00"
	set style line 8 lt 3 pt 7 lc rgb "#0fad00"
	set style line 9 lt 3 pt 7 lc rgb "#0fad00"
	
	set xlabel "Average Number of Matched Features"
	set ylabel "Average Clusteredness {/Times-Italic s}"
	
	set yrange [*:0.021]
	set xrange [200:*]
	
	plot "eval/exfast-${u1}_$set.dat" using (\$2/21):4 with lines ls 1 title "exFAST", \
		"eval/exfast-${u2}_$set.dat" using (\$2/21):4 with lines ls 2 notitle, \
		"eval/exfast-${u3}_$set.dat" using (\$2/21):4 with lines ls 3 notitle, \
		"eval/fast-12_$set.dat" using (\$2/21):4 with lines ls 4 title "FAST", \
		"eval/fast-15_$set.dat" using (\$2/21):4 with lines ls 5 notitle, \
		"eval/fast-20_$set.dat" using (\$2/21):4 with lines ls 6 notitle, \
		"eval/harris-.000015_$set.dat" using (\$2/21):4 with lines ls 9 title "Harris", \
		"eval/harris-.000005_$set.dat" using (\$2/21):4 with lines ls 8 notitle, \
		"eval/harris-.000002_$set.dat" using (\$2/21):4 with lines ls 7 notitle, \
		"" using (1/0):(1/0) lt 0 pt 7 title "u = $uniq", \
		"eval/exfast-${u1}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4  ls 1 notitle, \
		"eval/exfast-${u2}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 2 notitle, \
		"eval/exfast-${u3}_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 3 notitle, \
		"eval/fast-12_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 4 notitle, \
		"eval/fast-15_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 5 notitle, \
		"eval/fast-20_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 6 notitle, \
		"eval/harris-.000015_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 9 notitle, \
		"eval/harris-.000005_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 8 notitle, \
		"eval/harris-.000002_$set.dat" using (\$1 == $uniq ? \$2/21: 1/0):4 ls 7 notitle
EOF

epstopdf output.eps
mv output.pdf cluster_$set.pdf

################################# Counts #####################################

gnuplot << EOF
	`gnuplotCommon`
	set output "output.eps"
	#set term wxt persist
	
	set xlabel "Stereo Pair"
	set ylabel "Average Number of Matched Features"
	
	plot "eval/exfast-${u1}_${set}_counts.dat" using 1:2 with lines ls 1 title "exFAST $u1", \
		"eval/exfast-${u2}_${set}_counts.dat" using 1:2 with lines ls 2 title "exFAST $u2", \
		"eval/exfast-${u3}_${set}_counts.dat" using 1:2 with lines ls 3 title "exFAST $u3", \
		"eval/fast-12_${set}_counts.dat" using 1:2 with lines ls 4 title "FAST 12", \
		"eval/fast-15_${set}_counts.dat" using 1:2 with lines ls 5 title "FAST 15", \
		"eval/fast-20_${set}_counts.dat" using 1:2 with lines ls 6 title "FAST 20"
EOF

convertOutput counts_$set.pdf

############################### Step width ####################################

gnuplot << EOF
	`gnuplotCommon`
	set output "output.eps"
	#set term wxt persist
	
	set key top left
	
	set xlabel "Consistency / Uniqueness Check Step Width {/Times-Italic w}"
	set ylabel "Average Bad Matches Percentage / %"
	
	set xtics 1
	set yrange [0:5]
	
	plot "eval/exfast-${u1}_${set}_step.dat" using 1:(100*\$3) with lines ls 1 title "exFAST $u1", \
		"eval/exfast-${u2}_${set}_step.dat" using 1:(100*\$3) with lines ls 2 title "exFAST $u2", \
		"eval/exfast-${u3}_${set}_step.dat" using 1:(100*\$3) with lines ls 3 title "exFAST $u3"
EOF

convertOutput step_$set.pdf

#################### Matching Algorithms by BPP ##########################

gnuplot << EOF
	`gnuplotCommon`
	#set term wxt persist
	
	set xlabel "Average Number of Matched Features"
	set ylabel "Average Bad Matches Percentage / %"
	
	`if [ $set = orig ]; then echo set key left top; else echo set key bottom right; fi`
	`if [ $set = orig ]; then echo set yrange [0:27]; fi`
	set xtics 300
	
	plot "eval/exfast-1.0_$set.dat" using (\$2/21):(100*\$3) with lines ls 1 title "Dense Consistency", \
		"eval/exfast-1.0-denseBM_$set.dat" using (\$2/21):(100*\$3) with lines ls 2 title "Block Matching", \
		"eval/exfast-1.0-denseR_$set.dat" using (\$2/21):(100*\$3) with lines ls 3 title "Dense Right", \
		"eval/exfast-1.0-sparse_$set.dat" using (\$2/21):(100*\$3) with lines ls 4 title "Sparse"
		
EOF

convertOutput algo_$set.pdf

#################### Matching Algorithms by Matching ops ##########################

if [ $set = orig ]; then
	ymax=420000
else
	ymax=320000
fi

pref=eval_stereo_paper_old
#pref=eval
gnuplot << EOF
	`gnuplotCommon`
	#set term wxt persist
	
	set ylabel "Average Number of Window Matching Operations"
	set xlabel "Uniqueness Factor {/Times-Italic u}"
	
	set yrange [0:$ymax]
	set xrange [0.3:*]
	
	plot "$pref/exfast-s2-1.0_$set.dat" using 1:(\$5/21) with lines ls 1 title "Dense Consistency; w=2", \
		"$pref/exfast-1.0_$set.dat" using 1:(\$5/21) with lines ls 2 title "Dense Consistency; w=1", \
		"$pref/exfast-1.0-denseR_$set.dat" using 1:(\$5/21) with lines ls 3 title "Dense Right", \
		"$pref/exfast-1.0-sparse_$set.dat" using 1:(\$5/21) with lines ls 4 title "Sparse"
		
		
		
EOF

convertOutput matches_$set.pdf

done

#openPDFs
