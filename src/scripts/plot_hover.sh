#!/bin/bash
. `dirname $0`/gnuplot_common.sh

#x0=0.051682
#y0=-0.197869
#z0=0.194580

#-0.158841 -0.400900 0.372327
#-0.28050767696 -0.208743455573 0.241650317972

#x0=(-.12166667696)
#y0=(.192156544427)
x0=(-.118)
y0=(.19)
z0=(-.129)

dt=-0.31
t1=16.3
t2=50.2
r=(-1/180.0*pi)

### Transform

awk "
BEGIN {pi=3.14159;}
{
	print \$1+$dt, cos($r)*\$2 - sin($r)*\$3 - $x0, sin($r)*\$2 + cos($r)*\$3 - $y0, \$4 - $z0;
}" position_ptam_rel.txt > position_ptam_transformed.txt

### Pos Diff
`dirname $0`/position_error.sh position_filtered.txt position_ptam_transformed.txt $t1 $t2 > position_error.txt

#######################

gnuplot << EOF
`gnuplotCommon`
#set term postscript enhanced eps color dashed linewidth 3 size 10cm, 8cm
#set term postscript enhanced eps color dashed linewidth 3 size 8cm, 8.5cm
set term postscript enhanced eps color dashed linewidth 3 size 11cm, 12.5cm

set size ratio -1

set ticslevel 0
set zrange [0:*]

set xtic offset -0.5,-0.5
set xtic 0.2

set ytic offset 1
set ytic 0.2

set xtic rangelimited
set ytic rangelimited

set ztic 0.1

#set view 65,20

set grid lc rgb "#000000"
#set key at 0.45,0.0,0.65
set key at 0.45,0.0,0.55
#set key at 0.45,0.0,0.22

set view 70, 30

splot "position_filtered.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3):(0) with lines lc rgb '#B0B0B0' lt 1 notitle, \
	"position_ptam_transformed.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3):(0) with lines lc rgb '#B0B0B0' lt 2 notitle, \
	"position_filtered.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3):4 with lines ls 1 title "Ground Truth", \
	"position_ptam_transformed.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3):4 with lines ls 2 title "Estimate"

EOF

convertOutput position3d.pdf

##########################

gnuplot << EOF
`gnuplotCommon`
#set term postscript enhanced eps color dashed linewidth 3 size 6cm, 6cm
set term postscript enhanced eps color dashed linewidth 3 size 7cm, 7cm

set size ratio -1

plot "position_filtered.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3) with lines ls 1 notitle, \
	"position_ptam_transformed.txt" using (\$1>$t1 && \$1<$t2 ? \$2 : 1/0):(\$3+0.3) with lines ls 2 notitle
	
EOF

convertOutput position_top.pdf

##########################

gnuplot << EOF
`gnuplotCommon`
#set term postscript enhanced eps color dashed linewidth 3 size 12cm, 4cm
set term postscript enhanced eps color dashed linewidth 3 size 10cm, 3.5cm

set ylabel "Position Error / m"
set xlabel "Time / s"
plot "position_error.txt" using 1:2 with lines notitle
	
EOF

convertOutput position_error.pdf

###################################
pids=`awk '{if($1 != "PID" && ($8 == "S" || $8 == "R")) print $1}' top-2012-04-11-17-22-38_cropped.log | sort -u`

echo -n > cpu_stat.csv

for pid in $pids; do
	awk "BEGIN{count=0;sum=0;}{if(\$1 == $pid){count++;sum+=\$9;name=\$12}}END{print name \",\" sum/count/2;}" top-2012-04-11-17-22-38_cropped.log >> cpu_stat.csv
done

totalCPU=`grep Cpu top-2012-04-11-17-22-38_cropped.log | awk '
BEGIN{count=0;sum=0;}
{
	count++;
	
	split($2, buf, "%");
	sum+=buf[1];
	
	split($3, buf, "%");
	sum+=buf[1];
	
	split($4, buf, "%");
	sum+=buf[1];
}
END{print sum/count;}'`

echo "Total,$totalCPU" >> cpu_stat.csv

### RMSE
`dirname $0`/hover_error.sh position_filtered.txt
