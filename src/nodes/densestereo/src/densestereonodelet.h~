#ifndef KS_DENSESTEREONODELET_H
#define KS_DENSESTEREONODELET_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//#include <libelas/elas.h>
#include "elas.h"
//#include "/home/ait_jellal/projects/ws_mav/src/libelas/src/elas.h"
#include "libks/imageio/filequeue.h"
#include "libks/imageproc/colorcoder.h"
#include "libks/base/basewindow.h"
#include "libks/stereo/stereorectification.h"
#include "libks/imageio/sharedimagesetpublisher.h"

namespace densestereo {
	class DenseStereoNodelet: public nodelet::Nodelet {
	public:
		DenseStereoNodelet();
		~DenseStereoNodelet();

		// Performs general initializations
		virtual void onInit();

	private:
		std::string calibFile;
		std::string inputDir;
		std::string videoFile;
		double maxDisp;
		bool interactive;
		bool capture;
		bool graphicalCV, graphicalSDL;
		bool performanceTest;
		std::string writePath;
		int camera1, camera2;
		bool noROS;
		std::string worldFrame;
		bool shared;
		double rate;
		int subsampling;
		
		int currentFrame;
				
		boost::scoped_ptr<ks::BaseWindow> window;
		boost::scoped_ptr<ks::StereoImageQueue8U> imgQueue;
		ks::StereoFrame8U::ConstPtr stereoPair;
		cv::Mat_<cv::Vec3b> screen;
		boost::scoped_ptr<ks::ColorCoder> colCoder;
		boost::scoped_ptr<cv::VideoWriter> video;
		boost::scoped_ptr<ks::StereoRectification> rectification;
		boost::scoped_ptr<ks::SharedImageSetPublisher> stereoPublisher;
		
		boost::shared_ptr<Elas> elas;
		boost::shared_ptr<cv::StereoSGBM> sgbm;

		ros::Time messageTime;
		bool rosInput;
		boost::thread mainThread;

		// Runs the processing loop
		void mainLoop();

		// Parses all program options
		bool parseOptions();
		
		// Reads the next stereo pair and stores it in a member variable
		bool readStereoPair();
		
		// Displays the matches as colored boxes
		void visualizeMatches(const ks::StereoFrame8U::Type& pair, const cv::Mat_<float>& matches);
		
		// Publishes a ros message containing the dense disparity map
		void publishDisparities(const cv::Mat_<float>& disparityMap);
		
		// Methods for running different stereo algorithms
		void elasStereo(const ks::StereoFrame8U::Type rectPair, cv::Mat_<float> output);
		void sgbmStereo(const ks::StereoFrame8U::Type rectPair, cv::Mat_<float> output);
	};
}

#endif
