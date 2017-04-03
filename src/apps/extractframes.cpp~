#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

// Supported messages
#include <libks_msgs/MultiCameraImage.h>
#include <libks_msgs/ImageSet.h>

using namespace std;

int main(int argc, char** argv) {
	if(argc != 3) {
		cout << "Usage: " << argv[0] << " INPUT-FILE OUTPUT-DIR" << endl;
		return 1;
	}

	rosbag::Bag inBag(argv[1]);
	rosbag::View view(inBag);
	unsigned int frameCtr = 0;
	string outPath = argv[2];
	fstream* strm = NULL;
	
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		vector<cv_bridge::CvImageConstPtr> images;
		ros::Time msgTime;
		
		if(m.getDataType() == "libks_msgs/ImageSet") {
			libks_msgs::ImageSetPtr msgIn = m.instantiate<libks_msgs::ImageSet>();
			msgTime = msgIn->header.stamp;
			for(unsigned int i=0; i<msgIn->images.size(); i++) {
				if(msgIn->images[i].encoding != "" && msgIn->images[i].width > 0)
					images.push_back(cv_bridge::toCvCopy(msgIn->images[i]));
				else images.push_back(cv_bridge::CvImageConstPtr());
			}
		}
		else if(m.getDataType() == "libks_msgs/MultiCameraImage") {
			libks_msgs::MultiCameraImagePtr msgIn = m.instantiate<libks_msgs::MultiCameraImage>();
			msgTime = msgIn->header.stamp;
			
			if(msgIn->cam0.encoding != "" && msgIn->cam0.width > 0)
				images.push_back(cv_bridge::toCvCopy(msgIn->cam0));
			else images.push_back(cv_bridge::CvImageConstPtr());
			
			if(msgIn->cam1.encoding != "" && msgIn->cam1.width > 0)
				images.push_back(cv_bridge::toCvCopy(msgIn->cam1));
			else images.push_back(cv_bridge::CvImageConstPtr());
			
			if(msgIn->cam2.encoding != "" && msgIn->cam2.width > 0)
				images.push_back(cv_bridge::toCvCopy(msgIn->cam2));
			else images.push_back(cv_bridge::CvImageConstPtr());
			
			if(msgIn->cam3.encoding != "" && msgIn->cam3.width > 0)
				images.push_back(cv_bridge::toCvCopy(msgIn->cam3));
			else images.push_back(cv_bridge::CvImageConstPtr());
		}
		
		if(images.size() != 0) {
			if(strm == NULL)
				strm = new fstream((outPath + "/times.csv").c_str(), ios::out);
			
			(*strm) << setw(5) << setfill('0') << frameCtr << setw(0) << " " << msgTime << endl;
			
			for(int i=0; i<images.size(); i++) {
				if(images[i] != NULL) {
					char fileName[30];
					snprintf(fileName, sizeof(fileName), "/image%05d_c%d.png", frameCtr, i);
					cv::imwrite(outPath + fileName, images[i]->image);
				}
			}
			
			frameCtr++;
		}
	}
	
	inBag.close();
	
	if(strm != NULL) {
		strm->close();
		delete strm;
	}
	
	return 0;
}
