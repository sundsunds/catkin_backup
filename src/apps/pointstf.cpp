#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;
using namespace ros;
using namespace tf;

TransformBroadcaster* tfBroadcaster = NULL;
Time lastPublish = Time(0);
string toFrame, fromFrame;
bool firstPublish = true;
Transform trans;

void handlePoints(const sensor_msgs::PointCloud::ConstPtr& pointsMsg) {
	if((pointsMsg->header.stamp - lastPublish).toSec() > 0.5) {
		Time futureStamp = pointsMsg->header.stamp;
		if(!firstPublish) {
			lastPublish = futureStamp;
			futureStamp.sec += 1; // Time in the future
		} else firstPublish = false;
		tfBroadcaster->sendTransform(tf::StampedTransform(trans, futureStamp, fromFrame, toFrame));
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "futuretf");
	
	if(argc != 5) {
		cout << "Usage: " << argv[0] << "tx,ty,tz,rx,ry,rz,rw fromFrame toFrame topic" << endl;
		return 1;
	}
	
	NodeHandle nh;
	tfBroadcaster = new TransformBroadcaster;

	double x=0, y=0, z=0;
	double rx=0, ry=0, rz=0, rw=1;
	sscanf(argv[1], "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &rx, &ry, &rz, &rw);
	
	trans.setOrigin(Vector3(x, y, z));
	trans.setRotation(Quaternion(rx, ry, rz, rw));
	
	fromFrame = argv[2];
	toFrame = argv[3];
	string topic = argv[4];
	
	Subscriber pointsSub = nh.subscribe(topic, 0, &handlePoints);
	
	ros::spin();
			
	return 0;
}
