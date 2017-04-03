#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <octomap/OcTree.h>
#include <boost/smart_ptr.hpp>
#include <libks/base/basewindow.h>
#include <libks/imageproc/colorcoder.h>
#include <visualization_msgs/MarkerArray.h> 
#include "parameters.h"
#include "robustoctree.h"
#include "octreewrapper.h"
#include "settings.h"

namespace occupancymap {
	// Performs all visualization tasks
	class Visualization {
	public:
		Visualization(ros::NodeHandle& nh, boost::shared_ptr<Parameters> parameters);
		
		void visualizeAll(const cv::Mat_<float>& dispMap, const cv::Mat_<cv::Point3f>& pointsMap,
			OcTreeType& ocTree, const tf::Vector3& position, const tf::Quaternion& orientation, ros::Time stamp);
		
		void visualizeOctree(OcTreeType& ocTree, ros::Time stamp);
			
	private:
		boost::shared_ptr<Parameters> parameters;
		ros::Publisher publisher;
		int seqNum;
		int visualizationCalls;
		
		boost::scoped_ptr<ks::BaseWindow> window;
		boost::scoped_ptr<ks::ColorCoder> stereoColCoder, octomapColCoder;
		cv::Mat_<cv::Vec3b> screen;
		
		// Sets common properties for all messages
		void initMarkerMsg(visualization_msgs::Marker& marker, ros::Time stamp);
		
		// Renders a disparity map visualization in a debugging window
		void visualizeDisparities(const cv::Mat_<float>& dispMap, const cv::Mat_<cv::Point3f>& pointsMap);
		
		// Methods for publishing visualization messages for ROS
		void createPointsMarker(const cv::Mat_<cv::Point3f>& pointsMap, const cv::Mat_<float>& dispMap,
			visualization_msgs::MarkerArray& markers, const tf::Vector3& position, const tf::Quaternion& orientation,
			ros::Time stamp);
		void createOctreeMarkers(OcTreeType& ocTree, visualization_msgs::MarkerArray& markers, ros::Time stamp);
	};
}

#endif
