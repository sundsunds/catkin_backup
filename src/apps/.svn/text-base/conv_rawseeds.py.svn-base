#!/usr/bin/python
import sys
import roslib
roslib.load_manifest('apps')
import rospy
import tf
import time
import math
import cv
import rosbag
import re
import os

from geometry_msgs.msg import PoseStamped
from libks.msg import ImageSet
from cv_bridge import CvBridge

if len(sys.argv) < 5:
	print "Usage: ", sys.argv[0], "GT-File Left-Dir Right-Dir Output-Bag";
	sys.exit(1)

# Initialize bag writing if selected
writeBag = True
bag = rosbag.Bag(sys.argv[4], "w")
	
# Initialize publishers
rospy.init_node("conv_rawseeds")

bridge = CvBridge()
gtFile = open(sys.argv[1])
seq = 0

poseMsgs = []

# Read ground truth
while gtFile:
	# Parse current pose
	line = gtFile.readline()
	fields = line.split(" ")
	
	#if fields[0] != "ROBOTLASER1" or rospy.is_shutdown():
	#	break
	
	#numReadings = int(fields[8])
	#numRemissions = int(fields[9 + numReadings])
	#offset = numReadings + numRemissions
	#x = float(fields[13 + offset])
	#y = float(fields[14 + offset])
	#theta = float(fields[15 + offset])
	#strTime = fields[21 + offset]

	if len(fields) < 4 or rospy.is_shutdown():
		break
		
	x = float(fields[1])
	y = float(fields[2])
	theta = float(fields[3])
	strTime = fields[0]
	
	floatTime = float(strTime)
	
	print "Pose", strTime

	# Compose pose message
	
	# See http://answers.ros.org/question/53688/euler-angle-convention-in-tf/
	# tough I don't know if the data file follows the same convention
	quat = tf.transformations.quaternion_from_euler(theta - math.pi/2, 0, 0, axes='rzyx')
	
	poseMsg = PoseStamped()
	poseMsg.header.frame_id = "/world"
	poseMsg.header.stamp = rospy.Time.from_sec(floatTime)
	seq += 1
	poseMsg.header.seq = seq
	
	poseMsg.pose.orientation.x = quat[0]
	poseMsg.pose.orientation.y = quat[1]
	poseMsg.pose.orientation.z = quat[2]
	poseMsg.pose.orientation.w = quat[3]
	
	poseMsg.pose.position.x = x
	poseMsg.pose.position.y = y
	poseMsg.pose.position.z = 0
	
	#poseMsgs.append(poseMsg)
	bag.write("/pose", poseMsg, poseMsg.header.stamp)

seq = 0
poseIdx = 0

# Get list of let camera images
leftFilesList = []
for leftFile in os.listdir(sys.argv[2]):
	leftFilesList.append(leftFile)
	
# Get timestamps for gith camera images
rightTimestampsList = []
for rightFile in os.listdir(sys.argv[3]):
	strTime = re.sub(r"SVS_R_([0-9]+\.[0-9]+)\.png", "\g<1>", rightFile)
	rightTimestampsList.append(strTime)
	
leftFilesList.sort()
rightTimestampsList.sort()

# Iterate over all left camera images
for iL in range(len(leftFilesList)):
	if rospy.is_shutdown():
		break
	
	leftFile = leftFilesList[iL]
	strTime = re.sub(r"SVS_L_([0-9]+\.[0-9]+)\.png", "\g<1>", leftFile)
	floatTime = float(strTime)
	
	# Find best right image
	iR = iL if iL < len(rightTimestampsList) else len(rightTimestampsList) -1
	lastDiff = 1e10
	bestIR = iR;
	while True:
		diff = float(rightTimestampsList[iR]) - floatTime

		if math.fabs(diff) < math.fabs(lastDiff):
			bestIR = iR
			lastDiff = diff
		
		if diff > 0:
			iR-=1
		else: iR+=1
		
		if iR < 0 or iR == len(rightTimestampsList) or math.fabs(diff) > math.fabs(lastDiff):
			break
	
	
	rightFile = "SVS_R_" + rightTimestampsList[bestIR] + ".png"
	print leftFile, rightFile, (float(rightTimestampsList[bestIR]) - floatTime)
	
	if math.fabs(lastDiff) < 0.005:
		# Write left and right image to bag
		leftFullPath = sys.argv[2] + "/" + leftFile
		rightFullPath = sys.argv[3] + "/" + rightFile
		
		leftImg = cv.LoadImage(leftFullPath, cv.CV_LOAD_IMAGE_GRAYSCALE)
		rightImg = cv.LoadImage(rightFullPath, cv.CV_LOAD_IMAGE_GRAYSCALE)
		
		captureMsg = ImageSet()
		captureMsg.header.frame_id = "/world"
		captureMsg.header.stamp = rospy.Time.from_sec(floatTime)
		seq += 1
		captureMsg.header.seq = seq
		captureMsg.images.append(bridge.cv_to_imgmsg(leftImg, "mono8"))
		captureMsg.images.append(bridge.cv_to_imgmsg(rightImg, "mono8"))
		
		bag.write("/capture", captureMsg, captureMsg.header.stamp)
	else:
		skippedFrames.apped(leftFile)

print "Skipped frames:"
skippedFrames = []
for skipped in skippedFrames:
	print skipped

bag.close()
