#ifndef OCCUPANCYMAPNODELET_H
#define OCCUPANCYMAPNODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/tf.h>
#include "libks_msgs/SharedImageSet.h"
#include "libks/stereo/cameracalibration.h"
#include "visualization.h"
#include "parameters.h"
#include "robustoctree.h"
#include "octreewrapper.h"
#include "settings.h"

namespace occupancymap {
	class OccupancyMapNodelet: public nodelet::Nodelet {
	public:
		OccupancyMapNodelet();
		~OccupancyMapNodelet();

		// Performs general initializations
		virtual void onInit();


        void stereoSynchronizerCallback(const libks_msgs::SharedImageSetConstPtr& multiCam,
            const geometry_msgs::PoseStampedConstPtr& pose);

        void stereoSynchronizerCallback_2(cv::Mat_<float> dispMap,
            const geometry_msgs::PoseStamped pose);

        void set_is_elas(bool is_elas_stereo);

        boost::shared_ptr<Parameters> parameters;
        boost::scoped_ptr<OcTreeType> ocTree;


	private:
		typedef message_filters::sync_policies::ExactTime<libks_msgs::SharedImageSet,
        geometry_msgs::PoseStamped> StereoPoseSyncPolicy;

		boost::scoped_ptr<message_filters::Subscriber<libks_msgs::SharedImageSet> > stereoSubscriber;
		boost::scoped_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped> > poseSubscriber;
		boost::scoped_ptr<message_filters::Synchronizer<StereoPoseSyncPolicy> > stereoSynchronizer;
		
        //boost::shared_ptr<Parameters> parameters;
		boost::scoped_ptr<Visualization> visualization;
        //boost::scoped_ptr<OcTreeType> ocTree;
		octomap::Pointcloud currentScan;
		
		bool processedLastFrame;
		boost::thread offlineProcThread;
		boost::mutex offlineProcMutex;
		boost::condition_variable offlineProcCond;
		
		ks::CalibrationResult calib;
		cv::Mat_<cv::Point3f> pointsMap;
        bool is_elas;

		void loadOctree();
		void processBag();

		void convertPose(const geometry_msgs::PoseStamped& pose, tf::Vector3& position, tf::Quaternion& orientation);
		void disparityMapToScan(const cv::Mat_<float>& dispMap);
	};
}

#endif
