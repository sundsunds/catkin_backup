#!/usr/bin/python
import sys
import math
import random

auto = float('Inf')
initTf = [
	auto, #dt
	auto, #dx
	auto, #dy
	auto, #dz
	0,#math.pi, #dr	
]
#initTf = [16332.61804541883, 0.2969362783385985, -0.41986237745566074, -0.21239968570297332, -0.10007515031532903]


startStepWidth = 0.01
minDelta = 1e-5
maxDelta = 1e+5
minErrChange = 1e-6
#minErrChange = 1e-10

acc=1.2
dec=0.5

## Function Definitions

def optimize(tf, idx, data1, data2, evalDT):
	# Get base line
	transformed2 = []
	transform(data2, transformed2, tf, False)
	prevErr = calcDiffs(data1, transformed2, False, evalDT)

	# Go positive direction
	inc = startStepWidth
	
	while True:
		tf[idx] += inc
		
		transformed2 = []
		transform(data2, transformed2, tf, False)
		currErr = calcDiffs(data1, transformed2, False, evalDT)
		
		if currErr[0] > prevErr[0]:
			inc = -inc*dec
			if inc*inc < minDelta*minDelta:
				break
		else:
			inc*=acc
			if inc*inc > maxDelta*maxDelta:
				break

		prevErr = currErr	
	
	return currErr

def transform(input, output, tf, transQuat):
	dt = tf[0]
	dx = tf[1]
	dy = tf[2]
	dz = tf[3]
	dr = tf[4]
	
	del output[:]
	
	for i in range(0, len(input)):
		if transQuat:
			output.append([
				input[i][0] - dt,
				math.cos(dr) * input[i][1] - math.sin(dr) * input[i][2] - dx,
				math.sin(dr) * input[i][1] + math.cos(dr) * input[i][2] - dy,
				input[i][3] - dz,
				math.cos(dr) * input[i][4] - math.sin(dr) * input[i][5],
				math.sin(dr) * input[i][4] + math.cos(dr) * input[i][5],
				input[i][6],
				input[i][7],
			])
		else:
			output.append([
				input[i][0] - dt,
				math.cos(dr) * input[i][1] - math.sin(dr) * input[i][2] - dx,
				math.sin(dr) * input[i][1] + math.cos(dr) * input[i][2] - dy,
				input[i][3] - dz,
			])
	
def calcDiffs(data1, data2, calcAvg, evalDT):
	rmse = 0
	avg = 0
	j = 0
	count = 0

	startTime = -1;

	for i in range(0, len(data1)):
		while j < len(data2) and data2[j][0] < data1[i][0]:
			j+=1
		
		if j == len(data2):
			break
		if j == 0:
			continue
		
		dt1 = data1[i][0] - data2[j-1][0]
		dt2 = data2[j][0] - data2[j-1][0]
		
		if dt2 > 0.1:
			continue # Gap in the file
		if startTime < 0:
			startTime = data1[i][0]
		
		if evalDT > 0 and data1[i][0] > startTime + evalDT:
			continue; # Limited to evaluation period
		
		# Interpolate
		x = data2[j-1][1] + dt1*(data2[j][1] - data2[j-1][1])/dt2
		y = data2[j-1][2] + dt1*(data2[j][2] - data2[j-1][2])/dt2
		z = data2[j-1][3] + dt1*(data2[j][3] - data2[j-1][3])/dt2
		
		dx = data1[i][1] - x
		dy = data1[i][2] - y
		dz = data1[i][3] - z
		
		sqrError = dx*dx + dy*dy + dz*dz
		if calcAvg:
			avg += sqrError ** 0.5
		rmse += sqrError
		count += 1
	
	if evalDT > 0 or count > len(data1)/2: # We want at least half the points being matched
		return [
			(rmse / count) ** (0.5),
			avg / count
		]
	else: return [float('Inf'), float('Inf')]

## Main

if len(sys.argv) < 4 or len(sys.argv) > 5:
	print "Usage: ", sys.argv[0], "input-file1 input-file2 output-file [eval-time]";
	sys.exit(1)

file1=open(sys.argv[1])
file2=open(sys.argv[2])

data1 = []
data2 = []

evalDT = -1
if len(sys.argv) == 5:
	evalDT = float(sys.argv[4])

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
	
	if len(fields) >= 8:
		data2.append([float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3]),
			float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7])])
	else: data2.append([float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3]),
		0.0, 0.0, 0.0, 0.0])

file1.close()
file2.close()

tf = initTf

# initial guess for transform
if tf[0] == auto:
	tf[0] = data2[0][0] - data1[0][0]
else: tf[0] += data2[0][0] - data1[0][0]
	
for i in range(1, 4):
	if tf[i] == auto:
		tf[i] = data2[0][i] - data1[0][i]

# Should optimize time first
lastErr = optimize(tf, 0, data1, data2, evalDT)

while True:
	order = [ 0, 1, 2, 3, 4 ]
	random.shuffle(order)
	
	err = lastErr
	for i in range(0, len(order)):
		oldTf = tf
		oldErr = err
		idx = order[i]
		err = optimize(tf, idx, data1, data2, evalDT)
		print "Transform:", tf, "; error: ", err[0]
		if err[0] > oldErr[0]:
			tf = oldTf
			err = oldErr
	
	dE = math.fabs(lastErr[0] - err[0])
	print "Error change:", dE
	if dE < minErrChange:
		break
	lastErr = err

transformed = []
transform(data2, transformed, tf, True)
err = calcDiffs(data1, transformed, True, evalDT)

print "Final error:", err, " (rmse, avg)"
print "Final transform:", tf

outFile=open(sys.argv[3], 'w')
outFile.write('#Time\tX\tY\tZ\tRX\tRY\tRZ\tRW\n')
for i in range(0, len(transformed)):
	outFile.write(('%.3f' % transformed[i][0]) + '\t' + str(transformed[i][1]) + '\t' + str(transformed[i][2]) + '\t' +
		str(transformed[i][3]) + '\t' + str(transformed[i][4]) + '\t' + str(transformed[i][5]) + '\t' + str(transformed[i][6]) + '\t' +
		str(transformed[i][7]) + '\n')
outFile.close()
