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
	print "Usage: ", sys.argv[0], "Times-File Pose-File Image-Dir Output-Bag";
	sys.exit(1)

# Initializations
bag = rosbag.Bag(sys.argv[4], "w")
rospy.init_node("conv_rawseeds")
bridge = CvBridge()
seq = 0

times = []
poseMsgs = []

timesFile = open(sys.argv[1])
while timesFile:
	line = timesFile.readline()
	try:
		floatTime = float(line)
		times.append(floatTime) 
	except ValueError:
		break

# Read poses and convert
poseFile = open(sys.argv[2])
while poseFile:
	# Parse current pose
	line = poseFile.readline()
	fields = line.split(" ")
	
	if len(fields) < 12 or rospy.is_shutdown():
		break
	
	mat = [
		[float(fields[0]), float(fields[1]), float(fields[2]), float(fields[3])],
		[float(fields[4]), float(fields[5]), float(fields[6]), float(fields[7])],
		[float(fields[8]), float(fields[9]), float(fields[10]), float(fields[11])],
		[0.0, 0.0, 0.0, 1.0]
	];
	
	time = times[seq]
	
	print "Pose ", time

	# Compose pose message
	
	# See http://answers.ros.org/question/53688/euler-angle-convention-in-tf/
	# tough I don't know if the data file follows the same convention
	#quat = tf.transformations.quaternion_from_euler(theta - math.pi/2, 0, 0, axes='rzyx')
	quat = tf.transformations.quaternion_from_matrix(mat)
	
	poseMsg = PoseStamped()
	poseMsg.header.frame_id = "/world"
	poseMsg.header.stamp = rospy.Time.from_sec(time)
	seq += 1
	poseMsg.header.seq = seq
	
	poseMsg.pose.orientation.x = quat[0]
	poseMsg.pose.orientation.z = -quat[1]
	poseMsg.pose.orientation.y = quat[2]
	poseMsg.pose.orientation.w = quat[3]
	
	poseMsg.pose.position.x = mat[0][3]
	poseMsg.pose.position.z = -mat[1][3]
	poseMsg.pose.position.y = mat[2][3]
	
	#poseMsgs.append(poseMsg)
	bag.write("/pose", poseMsg, poseMsg.header.stamp)

seq = 0
poseIdx = 0

# Get list of let camera images
leftFile=True
leftFileName=""
fileNames=[]

for file in os.listdir(sys.argv[3]):
	fileNames.append(file)
	
fileNames.sort()

for file in fileNames:
	if rospy.is_shutdown():
		break

	if leftFile:
		leftFileName = file
		leftFile=False
		continue
	
	time = times[seq]
	
	print time, " ", leftFileName, " ", file
	
	leftImg = cv.LoadImage(sys.argv[3]+"/"+leftFileName, cv.CV_LOAD_IMAGE_GRAYSCALE)
	rightImg = cv.LoadImage(sys.argv[3]+"/"+file, cv.CV_LOAD_IMAGE_GRAYSCALE)
		
	captureMsg = ImageSet()
	captureMsg.header.frame_id = "/world"
	captureMsg.header.stamp = rospy.Time.from_sec(time)
	seq += 1
	captureMsg.header.seq = seq
	captureMsg.images.append(bridge.cv_to_imgmsg(leftImg, "mono8"))
	captureMsg.images.append(bridge.cv_to_imgmsg(rightImg, "mono8"))
		
	bag.write("/capture", captureMsg, captureMsg.header.stamp)
	leftFile=True

bag.close()
