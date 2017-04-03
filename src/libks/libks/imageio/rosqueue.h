#ifndef KS_ROSQUEUE_H
#define KS_ROSQUEUE_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include <utility>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <libks/imageio/imagequeue.h>
#include <libks_msgs/ImageSet.h>
#include <libks_msgs/SharedImageSet.h>

namespace ks {
	// Base class for all queues reading from ROS messages
	template <class T>
	class BaseRosQueue  {
	public:
		BaseRosQueue(ros::NodeHandle& handle, const std::vector<int>& cameraIndices, bool shared, double rate);
		virtual ~BaseRosQueue() {}
			
		// Reads the images for the next frame
		bool readNextImages(cv_bridge::CvImageConstPtr images[]);
		// Waits until a buffer is filled
		void waitForCompleteFrame();
		// Returns the sequence Id of the last read frame
		unsigned int getLastSequenceId() const {return readSeqId;}
		// Returns the last received message time
		ros::Time getLastMsgTime() const {return msgTime;}
		
	private:
		// Helper class for callbacks
		class CallbackHelper {
		public:
			CallbackHelper(BaseRosQueue* obj, double rate): obj(obj),
				rate(rate), nextCapture(0) {}
            void callbackDefault(const libks_msgs::ImageSet::ConstPtr& msg);
            void callbackShared(const libks_msgs::SharedImageSet::ConstPtr& msg);
			
		private:
			BaseRosQueue* obj;
			double rate;
			ros::Time nextCapture;

			// Checks if a frame with the given time stamp shall be captured
			bool checkCapture(const ros::Time& stamp);
		};
		
		friend class CallbackHelper;
		
		boost::condition_variable frameCondition;
		boost::mutex bufferMutex;
		std::vector<cv_bridge::CvImageConstPtr> imageBuffers;
		std::vector<int> cameraIndices;
		
		CallbackHelper callbackHelper;
		boost::shared_ptr<ros::Subscriber> subscriber;
		unsigned int bufferSeqId, readSeqId;
		ros::Time msgTime;
		bool shared;
	};
	
	// Template for monocular image queues
	template <class T>
	class MonoRosQueue: public ImageQueue<typename MonoFrame<T>::Type> {
	public:
		MonoRosQueue(ros::NodeHandle& handle, bool shared, bool getIncompete, int cam0=0,
			double rate=-1);
		virtual ~MonoRosQueue(){}
		// Returns the sequence ID of the last popped frame
		unsigned int getLastSequenceId() const {return baseQueue->getLastSequenceId();}
		// Returns the last received message time
		ros::Time getLastMsgTime() const {return baseQueue->getLastMsgTime();}
	
	protected:
		boost::scoped_ptr<BaseRosQueue<T> > baseQueue;
		bool warned;
		bool getIncomplete;
		
		virtual void queueFrame();
	};
	
	// Template for stereo image queues
	template <class T>
	class StereoRosQueue: public ImageQueue<typename StereoFrame<T>::Type> {
	public:
		StereoRosQueue(ros::NodeHandle& handle, bool shared, bool getIncomplete,
			int cam0=0, int cam1=1, double rate=-1);
		virtual ~StereoRosQueue(){}
		// Returns the sequence ID of the last popped frame
		unsigned int getLastSequenceId() const {return baseQueue->getLastSequenceId();}
		// Returns the last received message time
		ros::Time getLastMsgTime() const {return baseQueue->getLastMsgTime();}
	
	protected: 
		boost::scoped_ptr<BaseRosQueue<T> > baseQueue;
		bool warned;
		bool getIncomplete;
		
		virtual void queueFrame();
	};
	
	// Template for double stereo image queues
	template <class T>
	class DoubleStereoRosQueue: public ImageQueue<typename DoubleStereoFrame<T>::Type> {
	public:
		DoubleStereoRosQueue(ros::NodeHandle& handle, bool shared, bool getIncomplete, int cam0=0,
			int cam1=1, int cam2=2, int cam3=3, double rate=-1);
		virtual ~DoubleStereoRosQueue(){}
		// Returns the sequence ID of the last popped frame
		unsigned int getLastSequenceId() const {return baseQueue->getLastSequenceId();}
		// Returns the last received message time
		ros::Time getLastMsgTime() const {return baseQueue->getLastMsgTime();}
	
	protected: 
		boost::scoped_ptr<BaseRosQueue<T> > baseQueue;
		bool warned;
		bool getIncomplete;
		
		virtual void queueFrame();
	};
	
	
	typedef MonoRosQueue<unsigned char> MonoRosQueue8U;
	typedef MonoRosQueue<unsigned short> MonoRosQueue16U;
	typedef StereoRosQueue<unsigned char> StereoRosQueue8U;
	typedef StereoRosQueue<unsigned short> StereoRosQueue16U;
	typedef DoubleStereoRosQueue<unsigned char> DoubleStereoRosQueue8U;
	typedef DoubleStereoRosQueue<unsigned short> DoubleStereoRosQueue16U;
}
#endif
