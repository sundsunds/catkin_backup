#ifndef KS_CAPTUREBASE_H
#define KS_CAPTUREBASE_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include "libks/imageio/filequeue.h"
#include "libks/imageio/rosqueue.h"
#include "libks/base/sdlwindow.h"
#include "libks/imageio/sharedimagesetpublisher.h"

namespace capture {
	class CaptureBase  {
	public:
		CaptureBase();
		
		// Performs neccessary initializations
		virtual void onInit(int argc, char** argv, ros::NodeHandle* nh);
		
		// Main nodelet loop
		void mainLoop();
		
	private:
		// Stores a mono, stereo or double stereo frame
		struct Frame {
			ks::MonoFrame8U::ConstPtr mono;
			ks::StereoFrame8U::ConstPtr stereo;
			ks::DoubleStereoFrame8U::ConstPtr doubleStereo;
		};
		
		boost::scoped_ptr<ros::NodeHandle> nodeHandle;
		boost::scoped_ptr<ros::AsyncSpinner> spinner;
		boost::thread writingThread;
		boost::mutex writingMutex;
		boost::condition_variable writingCond;
		boost::scoped_ptr<ks::SDLWindow> window;

		// Configuraton settings
		bool stereo;
		bool doubleStereo;
		int camera1, camera2, camera3, camera4;
		bool writePgm;
		bool grabFiles;
		bool graphical;
		bool noROS;
		double rate;
		std::string replayDir;
		bool rosTopic;
		float autoShutterMinDiff, autoShutterMin,
			autoShutterMax;
		bool alternating;
		bool secondHalfRate;
		
		bool countDown;
		
		// Capture sources
		boost::scoped_ptr<ks::MonoImageQueue8U> monoQueue;
		boost::scoped_ptr<ks::StereoImageQueue8U> stereoQueue;
		boost::scoped_ptr<ks::DoubleStereoImageQueue8U> doubleStereoQueue;
		
		boost::scoped_ptr<ros::Publisher> publisherDefault;
		boost::scoped_ptr<ks::SharedImageSetPublisher> publisherShared;
		boost::scoped_ptr<ros::Rate> playbackRate;
		
		
		// Queue for frames that are scheduled for writing 
		std::queue<std::pair<std::string, cv::Mat_<unsigned char> > > writeQueue;
		
		// Parses the command line options
		bool parseOptions(int argc, char** argv);

		// Loop waiting for grabbed frames	
		void writeLoop();
		
		// Captures the next frame
		bool captureNextFrame(Frame* dst);
		
		// Displays the grabbed frame in a window
		void displayFrame(const Frame& frame);
		
		// Schedules a frame for writing
		void scheduleWrite(const Frame& frame, int fileIndex);
		
		// Publishes the captured frame through ROS
		void sendDefaultRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp);
		void sendSharedRosMessage(const Frame& frame, unsigned int frameNum, ros::Time stamp);
		
		// Converts an OpenCV image to a ROS image
		void convertCVImage(const cv::Mat_<unsigned char>& in, sensor_msgs::Image* out,
			unsigned int frameNum, const ros::Time& rosTime);
	};
}

#endif
