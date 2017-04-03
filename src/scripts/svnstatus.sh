#!/bin/bash

sources=".cpp .h .cc .hpp CMaleLists.txt .tex .bib .xml .launch"
hideFiles=".log .aux .bbl .blg .brf Makefile CMakeCache.txt CMakeFiles .cmake Thumbs.db old .bak /lib /bin /msg_gen"

tmpfile1=`mktemp` 
tmpfile2=`mktemp`
status=`svn stat ~ > $tmpfile1`

for pattern in $hideFiles; do
	escaped=`echo $pattern | sed -e 's/\./\\\\./g'`
	cat $tmpfile1 | grep -v $escaped\$ > $tmpfile2
	mv -f $tmpfile2 $tmpfile1
done

expr=`echo $sources | sed -e 's/\./\\\\./g' | sed -e 's/ /\$\|/g'`

echo New non-source files:
cat $tmpfile1 | grep ^? | grep -v /\\. | egrep -v $expr\$

echo
echo New source files:
cat $tmpfile1 | grep ^? | grep -v /\\. | egrep $expr\$

echo
echo Modified files:
cat $tmpfile1 | grep -v ^?

rm $tmpfile1
