#ifndef KS_SHAREDIMAGESETPUBLISHER_H
#define KS_SHAREDIMAGESETPUBLISHER_H

#include <opencv2/opencv.hpp>
#include <queue>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "libks_msgs/SharedImageSet.h"

namespace ks {
	// Publishes images as a ROS message with shared ptrs
	class SharedImageSetPublisher {
	public:
		SharedImageSetPublisher(ros::NodeHandle& nh, const char* topic,
			unsigned int queueSize = 0, unsigned int sharedMemSize = 100);


        libks_msgs::SharedImageSetConstPtr createMessage(const std_msgs::Header& header, const cv::Mat& img1, const cv::Mat& img2,
            const cv::Mat& img3, const cv::Mat& img4);

        libks_msgs::SharedImageSetConstPtr createMessage(const std_msgs::Header& header, const cv::Mat& img1);


		void publish(const std_msgs::Header& header, const cv::Mat& img1);
		void publish(const std_msgs::Header& header, const cv::Mat& img1, const cv::Mat& img2);
		void publish(const std_msgs::Header& header, const cv::Mat& img1, const cv::Mat& img2,
			const cv::Mat& img3, const cv::Mat& img4);
		void publish(const std_msgs::Header& header, const std::vector<cv::Mat>& imgs);

        void publish(libks_msgs::SharedImageSetConstPtr msg);

        unsigned int sharedMemSize;
        std::queue<cv::Mat*> sharedMemHistory;

	private:

		ros::Publisher publisher;
        libks_msgs::SharedImageSetPtr message;
		
        libks_msgs::SharedImageSetConstPtr createMessage(const std_msgs::Header& header, cv::Mat** imgs, unsigned int numImgs);
		void publish(const std_msgs::Header& header, cv::Mat** imgs, unsigned int numImgs);
	};
}
#endif
