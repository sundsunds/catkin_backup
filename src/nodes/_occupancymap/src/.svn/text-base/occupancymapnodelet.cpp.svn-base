#include <iostream>
#include <limits>
#include <pluginlib/class_list_macros.h>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <libks/base/crashguard.h>
#include <libks/base/timer.h>
#include <libks/ImageSet.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <libks/base/taitbryan.h>
#include <libks/base/exception.h>
#include <signal.h>
#include "occupancymapnodelet.h"

PLUGINLIB_DECLARE_CLASS(occupancymap, OccupancyMapNodelet, occupancymap::OccupancyMapNodelet, nodelet::Nodelet)

namespace occupancymap {
	using namespace ks;
	using namespace std;
	using namespace boost;
	using namespace cv;
	using namespace ros;
	using namespace tf;
	using namespace octomap;
	
	OccupancyMapNodelet::OccupancyMapNodelet() {
		CrashGuard::setup();
	}
	
	OccupancyMapNodelet::~OccupancyMapNodelet() {
		offlineProcThread.join();
		
		cout << "Octree memory usage: " << ocTree->memoryUsage() / 1000000.0 << " MB" << endl;
		if(ocTree != NULL && parameters->mapSavePath != "") {
			cout << "Saving octree: " << parameters->mapSavePath << endl;
			fstream strm(parameters->mapSavePath.c_str(), ios::out | ios::binary | ios::trunc);
			ocTree->prune();
			ocTree->write(strm);
			cout << "done" << endl;
		}
	}
	
	void OccupancyMapNodelet::loadOctree() {
		if(parameters->mapLoadPath == "")
			return; // No path specified
		
		cout << "Loading octree: " << parameters->mapLoadPath << endl;
		fstream file(parameters->mapLoadPath.c_str(), ios::in);
		
		if(file.good()) {
			char line[512];
			do {
				file.getline(line, sizeof(line));
			} while(file.good() && string("data") != line);
			ocTree->readData(file);
			cout << "done" << endl;
			
			// Stupid rviz needs it twice
			visualization->visualizeOctree(*ocTree, ros::Time::now());
			sleep(1);
			visualization->visualizeOctree(*ocTree, ros::Time::now());
		} else {
			cout << "Reading failed!" << endl;
		}
	}

	void OccupancyMapNodelet::onInit() {
		parameters.reset(new Parameters(getName()));
		calib = CalibrationResult(parameters->calibFile.c_str());
		ocTree.reset(new OcTreeType(parameters));
		visualization.reset(new Visualization(getNodeHandle(), parameters));
			
		stereoSubscriber.reset(new message_filters::Subscriber<libks::SharedImageSet>(getNodeHandle(), "/densestereo/disparity_map", 1));
		poseSubscriber.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(getNodeHandle(), "/imufuse/pose_camera", 1));
	
		stereoSynchronizer.reset(new message_filters::Synchronizer<StereoPoseSyncPolicy>(
			StereoPoseSyncPolicy(2), *stereoSubscriber, *poseSubscriber));
		stereoSynchronizer->registerCallback(boost::bind(&OccupancyMapNodelet::stereoSynchronizerCallback, this, _1, _2));
		
		loadOctree();
		
		processedLastFrame = true;
		if(parameters->offlineProcessBag != "")
			offlineProcThread = thread(bind(&OccupancyMapNodelet::processBag, this));
	}
	
	void OccupancyMapNodelet::processBag() {
		try {
			cout << "Opening bag" << endl;
			rosbag::Bag bag(parameters->offlineProcessBag);
			rosbag::View view(bag);
			
			geometry_msgs::PoseStampedPtr pose;
			libks::ImageSetPtr imgs;
			
			Publisher posePub(getNodeHandle().advertise<geometry_msgs::PoseStamped>("/imufuse/pose_camera", 0)),
				camPub(getNodeHandle().advertise<libks::ImageSet>("/capture", 0));
			
			double fileStartTime = -1;
			double replayStartTime = -1;
			double lastPrintTime = -1;
			bool terminateProcess = true;
			
			BOOST_FOREACH(rosbag::MessageInstance const m, view) {
				if(!ros::ok()) {
					terminateProcess = false;
					break;
				}
				
				// Handle all time related tasks		
				if(fileStartTime == -1)
					fileStartTime = m.getTime().toSec();
				if((m.getTime().toSec() - fileStartTime) < parameters->bagTimeOffset)
					continue;
				if(replayStartTime == -1)
					replayStartTime = m.getTime().toSec();
				if(parameters->bagReplayDuration > 0 &&
					m.getTime().toSec() > replayStartTime + parameters->bagReplayDuration)
					break;
			
				// Wait for current frame to be processed
				while(!processedLastFrame) {
					if(!ros::ok())
						break;
					system_time timeout = boost::get_system_time() + boost::posix_time::milliseconds(500);
					unique_lock<mutex> lock(offlineProcMutex);
					offlineProcCond.timed_wait(lock, timeout);
				}
				
				// Publish next frame
				if(m.getDataType() == "geometry_msgs/PoseStamped") {
					pose = m.instantiate<geometry_msgs::PoseStamped>();
				} else if(m.getDataType() == "libks/ImageSet") {
					imgs = m.instantiate<libks::ImageSet>();
				}
				
				// Code for map growth evaluation
				/*static bool started = false;
				if(!started && m.getDataType() == "geometry_msgs/PoseStamped") {
					pose = m.instantiate<geometry_msgs::PoseStamped>();
				} else if(!started && m.getDataType() == "libks/ImageSet") {
					imgs = m.instantiate<libks::ImageSet>();
				}*/ /////////
				
				
				if(pose.get() != NULL && imgs.get() != NULL &&
					pose->header.stamp == imgs->header.stamp &&
					imgs->images.size() >=2) {
					
					// Code for map growth evaluation
					/*started = true;
					static int update = 0;
					double maxDist = 0;
					if(pose.get() != NULL) {
						point3d sensorPos(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
						for(OcTreeType::leaf_iterator it = ocTree->begin_leafs(); it!= ocTree->end_leafs(); it++) {
							if(!ocTree->isNodeOccupied(*it)) {
								point3d p = ocTree->keyToCoord(it.getKey());
								double dist = (sensorPos - p).norm();
								if(dist > maxDist)
									maxDist = dist;
							}
						}
						
						static fstream output("/home/schauwecker/octree_growth.txt", ios::out | ios::trunc);
						output << update++ << "\t" << maxDist << endl;
					}*/ /////////////////

					processedLastFrame = false;
					
					// Apply 45 Deg. rotation
					/*Matrix3x3 rotM;
					rotM.setEulerYPR(M_PI_4, 0, 0);
					tf::Transform trans(rotM);
					
					tf::Quaternion q = trans * tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y,
						pose->pose.orientation.z, pose->pose.orientation.w);
					pose->pose.orientation.x = q.x(); pose->pose.orientation.y = q.y();
					pose->pose.orientation.z = q.z(); pose->pose.orientation.w = q.w();
					
					tf::Vector3 newPos = trans * Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z);
					pose->pose.position.x = newPos.x(); pose->pose.position.y = newPos.y(); pose->pose.position.z = newPos.z();*/
					
					posePub.publish(pose);
					camPub.publish(imgs);
					
					if(int(lastPrintTime) != int(m.getTime().toSec())) {
						cout << "Time: " << (m.getTime().toSec() - fileStartTime) << endl;
						lastPrintTime = m.getTime().toSec();
					}
				}
			}
			bag.close();
			cout << "Done" << endl;
			
			if(terminateProcess)
				raise(SIGINT);
		} catch(const std::exception& ex) {
			NODELET_ERROR_STREAM("Exception in offline bag processing thread: " << ex.what());
		}
	}
	
	void OccupancyMapNodelet::stereoSynchronizerCallback(const libks::SharedImageSetConstPtr& multiCam,
		const geometry_msgs::PoseStampedConstPtr& pose) {

		if(multiCam->imagePtrs.size() == 0 || multiCam->imagePtrs[0] == 0)
			return; // No images in this message
		
		static ks::Timer timer("Occupancy Map");
		timer.start();
		
		// Convert the pose
		Vector3 position;
		Quaternion orientation;
		convertPose(*pose, position, orientation);	
		
		// Convert the point cloud
		Mat_<float> dispMap = ((Mat*)multiCam->imagePtrs[0])->clone();
		disparityMapToScan(dispMap);
		
		// Update
		octomath::Pose6D octomapPose(
			octomath::Vector3(position.x(), position.y(), position.z()),
			octomath::Quaternion(orientation.w(), orientation.x(), orientation.y(), orientation.z())
		);
		ocTree->insertPointCloud(currentScan, octomap::point3d(0, 0, 0), octomapPose, parameters->maxPointDist);
	
		timer.stop();
		
		// Visualize
		visualization->visualizeAll(dispMap, pointsMap, *ocTree, position, orientation, multiCam->header.stamp);
		
		processedLastFrame = true;
		offlineProcCond.notify_all();
	}
	
	void OccupancyMapNodelet::convertPose(const geometry_msgs::PoseStamped& pose, Vector3& position, Quaternion& orientation) {
		position = Vector3(pose.pose.position.x, pose.pose.position.y,
			pose.pose.position.z);
		orientation = Quaternion(pose.pose.orientation.x,
			pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
	}
	
	void OccupancyMapNodelet::disparityMapToScan(const Mat_<float>& dispMap) {
		if(dispMap.size() != pointsMap.size())
			pointsMap = Mat_<Point3f>(dispMap.size());

		currentScan.clear();
		currentScan.reserve(dispMap.rows * dispMap.cols);
		
		cv::Mat_<double> q = calib.Q.clone();
		for(int i=0; i<4; i++) {
			q(i, 0) *= parameters->subsampling;
			q(i, 1) *= parameters->subsampling;
		}
		
		reprojectImageTo3D(dispMap, pointsMap, q);
	
		if(pointsMap.rows%2 != 0)
			throw ks::Exception("Disparitiy map must have even size");
		
		// Swap x/z and mark outliers
		// Process disp-map in zig-zag pattern which is better for
		// further processing performance
		for(int y = 0; y < pointsMap.rows; y++) {
			for(int x = 0; x < pointsMap.cols; x++)
				if(dispMap(y,x) > 0 && finite(pointsMap(y,x).z))
					currentScan.push_back(pointsMap(y,x).x, pointsMap(y,x).z, -pointsMap(y,x).y);
			y++;
			for(int x = pointsMap.cols-1; x >=0; x--)
				if(dispMap(y,x) > 0 && finite(pointsMap(y,x).z))
					currentScan.push_back(pointsMap(y,x).x, pointsMap(y,x).z, -pointsMap(y,x).y);
		}
	}
}
 