#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>

// Supported messages
#include <libks_msgs/MultiCameraImage.h>
#include <libks_msgs/ImageSet.h>
//#include <pximu/RawIMUData.h>
//#include <pximu/AttitudeData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

int main(int argc, char** argv) {
	if(argc != 3) {
		cout << "Usage: " << argv[0] << " INPUT-FILE OUTPUT-FILE" << endl;
		return 1;
	}

	rosbag::Bag inBag(argv[1]);
	rosbag::Bag outBag(argv[2], rosbag::bagmode::Write);
	
	rosbag::View view(inBag);
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		//if(m.getDataType() == "pximu/RawIMUData")
		//	outBag.write(m.getTopic(), m.getTime(), m.instantiate<pximu::RawIMUData>(), m.getConnectionHeader());
	 	 if(m.getDataType() == "geometry_msgs/PoseStamped")
			outBag.write(m.getTopic(), m.getTime(), m.instantiate<geometry_msgs::PoseStamped>(), m.getConnectionHeader());
		else if(m.getDataType() == "geometry_msgs/Vector3Stamped")
			outBag.write(m.getTopic(), m.getTime(), m.instantiate<geometry_msgs::Vector3Stamped>(), m.getConnectionHeader());
		else if(m.getDataType() == "sensor_msgs/PointCloud")
			outBag.write(m.getTopic(), m.getTime(), m.instantiate<sensor_msgs::PointCloud>(), m.getConnectionHeader());
		//else if(m.getDataType() == "pximu/AttitudeData")
		//	outBag.write(m.getTopic(), m.getTime(), m.instantiate<pximu::AttitudeData>(), m.getConnectionHeader());
		else if(m.getDataType() == "libks_msgs/ImageSet")
			outBag.write(m.getTopic(), m.getTime(), m.instantiate<libks_msgs::ImageSet>(), m.getConnectionHeader());
		else if(m.getDataType() == "libks_msgs/MultiCameraImage") {
			// This type we convert
			libks_msgs::MultiCameraImagePtr msgIn = m.instantiate<libks_msgs::MultiCameraImage>();
			libks_msgs::ImageSet msgOut;
			msgOut.header.seq = msgIn->header.seq;
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = msgIn->header.frame_id;
			
			int count = 0;
			if(msgIn->cam3.encoding != "")
				count = 4;
			else if(msgIn->cam2.encoding != "")
				count = 3;
			else if(msgIn->cam1.encoding != "")
				count = 2;
			else if(msgIn->cam0.encoding != "")
				count = 1;
				
			msgOut.images.resize(count);
			
			if(count >=1) {
				msgOut.images[0] = msgIn->cam0;
				if(count >=2) {
					msgOut.images[1] = msgIn->cam1;
					if(count >=3) {
						msgOut.images[2] = msgIn->cam2;
						if(count == 4)
							msgOut.images[3] = msgIn->cam3;
					}
				}
			}
			
			outBag.write(m.getTopic(), m.getTime(), msgOut/*, m.getConnectionHeader()*/);
		}
		else cout << "Unknown type: " << m.getDataType() << endl;
	}
	
	inBag.close();
	outBag.close();
	return 0;
}
