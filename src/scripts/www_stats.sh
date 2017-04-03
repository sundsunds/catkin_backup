#!/bin/bash

if [ $# -ne 1 ]; then
	echo Usage: $0 [exfast,iros,ams,icuas,jint]
	exit 1;
fi

if [ $1 = "exfast" ]; then
	pattern=exFAST
elif [ $1 = "ams" ]; then
	pattern=schauwecker_ams
elif [ $1 = "iros" ]; then
	pattern=schauwecker_iros
elif [ $1 = "icuas" ]; then
	pattern=schauwecker_icuas
elif [ $1 = "jint" ]; then
	pattern=schauwecker_jint
elif [ $1 = "occmapping" ]; then
	pattern=occmapping.tar.bz2
elif [ $1 = "icra" ]; then
	pattern=schauwecker_icra
else
	echo "Invalid argument: $1"
fi

ssh www.ra.cs.uni-tuebingen.de "tail -200000 /rawww/www-admin/www-ra/apache/logs/httpd_www-ra_access.log | grep $pattern | grep -v u-172-c070.cs.uni-tuebingen.de |
	awk '{if(\"\" \$7 \$11 != prev){prev=\"\" \$7 \$11; print \"\\033[01;31m\" substr(\$4,2) \"\\033[00m\n\" \$1 \"\\n\" \$7 \"\\n\" \$11; for(i=12;i<=NF;i++) printf \"%s \", \$i; print \"\\n\";}}'"
