#!/usr/bin/python
import sys

if len(sys.argv) != 5 and len(sys.argv) != 3:
	print "Usage: ", sys.argv[0], "file1 file2 [t1 t2]";
	sys.exit(1)

file1=open(sys.argv[1])
file2=open(sys.argv[2])
if len(sys.argv) == 5:
	t1=float(sys.argv[3])
	t2=float(sys.argv[4])
else:
	t1=0
	t2=-1

data1 = []
data2 = []

# Read files
while file1:
	line = file1.readline()
	if not line:
		break
	fields = line.split()
	if len(fields) < 4 or fields[0][0] == '#':
		continue;
	data1.append([float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])])

while file2:
	line = file2.readline()
	if not line:
		break
	fields = line.split()
	if len(fields) < 4 or fields[0][0] == '#':
		continue;
	data2.append([float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])])

# Calc diffs
j=0
for i in range(0, len(data2)):
	if data2[i][0] <= t1:
		continue
	
	if data2[i][0] >= t2 and t2 >= 0:
		break
		
	while data1[j][0] < data2[i][0]:
		j+=1
	
	dt1 = data2[i][0] - data1[j-1][0]
	dt2 = data1[j][0] - data1[j-1][0]
	
	if max(dt1, dt2) > 0.02:
		continue;
	
	# Interpolate
	x = data1[j-1][1] + dt1*(data1[j][1] - data1[j-1][1])/dt2
	y = data1[j-1][2] + dt1*(data1[j][2] - data1[j-1][2])/dt2
	z = data1[j-1][3] + dt1*(data1[j][3] - data1[j-1][3])/dt2
	
	dx = data2[i][1] - x
	dy = data2[i][2] - y
	dz = data2[i][3] - z
	
	print data2[i][0]-t1, (dx*dx + dy*dy + dz*dz)**(0.5)

file1.close()
file2.close()
