#!/usr/bin/python

import sys
import subprocess

#################### main ###################

if len(sys.argv) != 2:
	print "Usage: ", sys.argv[0], " SVN-Directoy";
	sys.exit(1)


pipe = subprocess.Popen(["ls", "."], stdout=subprocess.PIPE,
	stderr=subprocess.PIPE)
	
(stdout, stderr) = pipe.communicate();
print stdout
print stderr