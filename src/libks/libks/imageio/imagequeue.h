#ifndef KS_IMAGEQUEUE_H
#define KS_IMAGEQUEUE_H

#include <vector>
#include <string>
#include <utility>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

namespace ks {
	// Loads sequence of stereo images through a backround thread
	template <class T>
	class ImageQueue {
	public:
		ImageQueue(bool useRos, bool allowFrameSkip, int queueSize, bool multiThreaded);
		~ImageQueue();
		
		// Pops a new image from the queue
		boost::shared_ptr<const T> pop();
	
	protected:
		// Pushes a new frame to the que
		void push(const boost::shared_ptr<const T>& frame);
		// Signals that the queue has reached its end
		void closeQueue();
		// Waits until there's a free slot in the queue
		void waitForFreeSlot();
		
		// Method for loading the next frame. Has to be overloaded.
		virtual void queueFrame() =  0;	
	
	private:
		bool useRos;
		boost::thread imageThread; // Queue thread
		bool allowFrameSkip;
		int queueSize;
		boost::scoped_array<boost::shared_ptr<const T> > queue; // The image queue
		int readIndex, writeIndex; // Current index of the queue
		boost::mutex queueMutex;
		boost::condition_variable popCond, pushCond;
		volatile bool done; // True if we reached the end
		bool multiThreaded; // If enabled, images are prefetched asynchroneously
		
		// Main method for the queue thread
		void threadMain();
	};
	
	
	// Frame types
	template <class T>
	struct MonoFrame {
		typedef cv::Mat_<T> Type;
		typedef boost::shared_ptr<Type> Ptr;
		typedef boost::shared_ptr<const Type> ConstPtr;
	};
	
	template <class T>
	struct StereoFrame {
		typedef std::pair<cv::Mat_<T>, cv::Mat_<T> > Type;
		typedef boost::shared_ptr<Type> Ptr;
		typedef boost::shared_ptr<const Type> ConstPtr;
	};
	
	template <class T>
	struct DoubleStereoFrame {
		typedef std::pair<std::pair<cv::Mat_<T>, cv::Mat_<T> >,
			std::pair<cv::Mat_<T>, cv::Mat_<T> > > Type;
		typedef boost::shared_ptr<Type> Ptr;
		typedef boost::shared_ptr<const Type> ConstPtr;
	};
	
	typedef MonoFrame<unsigned char> MonoFrame8U;
	typedef MonoFrame<unsigned short> MonoFrame16U;
	typedef MonoFrame<float> MonoFrame32F;
	typedef StereoFrame<unsigned char> StereoFrame8U;
	typedef StereoFrame<unsigned short> StereoFrame16U;
	typedef StereoFrame<float> StereoFrame32F;
	typedef DoubleStereoFrame<unsigned char> DoubleStereoFrame8U;
	typedef DoubleStereoFrame<unsigned short> DoubleStereoFrame16U;
	typedef DoubleStereoFrame<float> DoubleStereoFrame32F;
	
	
	// Typedefs for common queues
	typedef ImageQueue<MonoFrame<unsigned char>::Type> MonoImageQueue8U;
	typedef ImageQueue<StereoFrame<unsigned char>::Type> StereoImageQueue8U;
	typedef ImageQueue<DoubleStereoFrame<unsigned char>::Type> DoubleStereoImageQueue8U;
	typedef ImageQueue<MonoFrame<unsigned short>::Type> MonoImageQueue16U;
	typedef ImageQueue<StereoFrame<unsigned short>::Type> StereoImageQueue16U;
	typedef ImageQueue<DoubleStereoFrame<unsigned short>::Type> DoubleStereoImageQueue16U;
	typedef ImageQueue<MonoFrame<float>::Type> MonoImageQueue32F;
	typedef ImageQueue<StereoFrame<float>::Type> StereoImageQueue32F;
	typedef ImageQueue<DoubleStereoFrame<float>::Type> DoubleStereoImageQueue32F;
}

#endif
