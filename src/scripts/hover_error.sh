#!/usr/bin/python
import sys

t1=-1
t2=-1

if len(sys.argv) == 4:
	t1 = float(sys.argv[2])
	t2 = float(sys.argv[3])
elif len(sys.argv) != 2:
	print "Usage: ", sys.argv[0], "file1 [ t1 t2 ]";
	sys.exit(1)

file=open(sys.argv[1])

positions = []

# Read files
while file:
	line = file.readline()
	if not line:
		break
	fields = line.split()
	if len(fields) < 4 or fields[0][0] == '#' or \
	(t1 != -1 and float(fields[0])<t1) or (t2 != -1 and float(fields[0])>t2):
		continue
	positions.append([float(fields[1]), float(fields[2]), float(fields[3])])
	
avgPos = [0, 0, 0]
for p in positions:
	avgPos[0] += p[0]
	avgPos[1] += p[1]
	avgPos[2] += p[2]

avgPos[0] /= len(positions)
avgPos[1] /= len(positions)
avgPos[2] /= len(positions)

sumSquare=0
sumLen=0

for p in positions:
	dx=avgPos[0] - p[0]
	dy=avgPos[1] - p[1]
	dz=avgPos[2] - p[2]
	sumSquare += dx*dx + dy*dy + dz*dz
	sumLen += (dx*dx + dy*dy + dz*dz)**(0.5)

print "Average hover pos: ",avgPos
print "RMSE: ", (sumSquare / len(positions))**(0.5)
print "Avg Err: ", (sumLen / len(positions))

file.close()
