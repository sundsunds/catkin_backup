#!/bin/bash

# Outputs common gnuplot settings
function gnuplotCommon() {
cat << EOF
	#set term postscript eps color dashed linewidth 3 size 8cm, 6.5cm
	#set term postscript eps color dashed linewidth 3 size 9cm, 7cm
	
	set term postscript enhanced eps color dashed linewidth 3 size 7cm, 6cm
	
	set style line 1 lt 1 pt 7 lc rgb "#ff0000"
	set style line 2 lt 2 pt 7 lc rgb "#0010a5"
	set style line 3 lt 3 pt 7 lc rgb "#0fad00"
	set style line 4 lt 5 pt 7 lc rgb "#fec500"
	
	set style line 5 lt 6 pt 7 lc rgb "#c5007c"
	set style line 6 lt 4 pt 7 lc rgb "#0064b5"
	set style line 7 lt 7 pt 7 lc rgb "#8cc700"
	set style line 8 lt 8 pt 7 lc rgb "#ff9400"
	
	set style line 9 lt 9 pt 7 lc rgb "#6300a5"
	set style line 10 lt 10 pt 7 lc rgb "#00a3c7"
	set style line 11 lt 11 pt 7 lc rgb "#ffff00"
	set style line 12 lt 12 pt 7 lc rgb "#ff6600"
	
	set style line 99 lw 0.5 lt 11 pt 7 lc rgb "#d0d0d0"
	
	set border lw 0.5
	set pointsize 1.2
	
	set grid ls 99
	
	#set lmargin 10
	
	set output "output.eps"
EOF
}

# Converts the output eps file to a pdf
function convertOutput() {
	epstopdf output.eps
	mv output.pdf $1
}

# Opens all pdfs in the current directory
function openPDFs() {
	if [ `uname` = Darwin ]; then
		openCmd=open
	else
		openCmd=acroread #gnome-open
	fi

	$openCmd *.pdf &
}