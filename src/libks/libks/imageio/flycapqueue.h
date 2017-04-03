#ifndef KS_FLYCAPQUEUE_H
#define KS_FLYCAPQUEUE_H

#include "libks/imageio/imagequeue.h"
#include "libks/base/exception.h"
//#include "/home/ait_jellal/projects/ws_mav/src/libks/libks/imageio/imagequeue.h"
//#include "/home/ait_jellal/projects/ws_mav/src/libks/libks/base/exception.h"

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

#ifdef HAS_FLYCAPTURE
#include <flycapture/FlyCapture2.h>
#endif

namespace ks {
#ifndef HAS_FLYCAPTURE
	// Dummy classes when compiled without flycapture

	template <class T>
	class MonoFlyCapQueue: public ImageQueue<cv::Mat_<T> > {
	public:
		MonoFlyCapQueue(int cameraIndex = 0, bool force16Bit = false)
			: ImageQueue<cv::Mat_<T> >(true, 1, false, false) {
			throw Exception("Flycapure not available!");
		}
		
		void adjustAutoShutter(float min, float max, float minDiff) {}

	protected:
		void queueFrame() {}
	};
	
	template <class T>
	class StereoFlyCapQueue: public ImageQueue<std::pair<cv::Mat_<T>, cv::Mat_<T> > > {
	public:
		typedef std::pair<cv::Mat_<T>, cv::Mat_<T> > StereoPair;
		
		StereoFlyCapQueue(int leftCameraIndex = 0, int rightCameraIndex = 1, bool force16Bit = false)
			: ImageQueue<std::pair<cv::Mat_<T>, cv::Mat_<T> > >(true, 1, true, false) {
			throw Exception("Flycapture not available!");
		}
		
		void adjustAutoShutter(float min, float max, float minDiff) {}

	protected:
		void queueFrame() {}
	};
	
	template <class T>
	class DoubleStereoFlyCapQueue: public ImageQueue<std::pair<std::pair<cv::Mat_<T>, cv::Mat_<T> >,
		 std::pair<cv::Mat_<T>, cv::Mat_<T> > > > {
	public:
		typedef std::pair<cv::Mat_<T>, cv::Mat_<T> > StereoPair;
		typedef std::pair<StereoPair, StereoPair> DoubleStereoPair;
		
		DoubleStereoFlyCapQueue(bool alternating, bool secondHalfRate,
			int index1 = 0, int index2 = 1, int index3 = 2, int index4 = 3, 
			bool force16Bit = false)
			: ImageQueue<DoubleStereoPair>(true, 1, true, false) {
			throw Exception("Flycapture not available!");
		}
		
		void adjustAutoShutter(float min, float max, float minDiff) {}

	protected:
		void queueFrame() {}
	};
#else

	// Class for capturing camera frames
	template <class T>
	class FlyCapture {
	public:
		FlyCapture(int cameraIndex = 0, bool force16Bit = false, bool strobe=false,
			 bool externalTrigger = false, bool halfFrameRate = false);
		~FlyCapture(); 
		
		cv::Mat_<T> grabFrame();
		void waitForNextFrame();
		
		int updateShutterTime(const cv::Mat_<T>& img);
		void setShutterTime(int time);
		
		void adjustAutoShutter(float min, float max, float minDiff) {
			autoShutterMin = min;
			autoShutterMax = max;
			autoShutterMinDiff = minDiff;
		}
		
		FlyCapture2::Camera& getCamera() {return camera;}
			
	private:
		static const int shutterMax;
		static FlyCapture2::BusManager busManager;
		
		int cameraIndex;
		FlyCapture2::Image grabBuffer, conversionBuffer;
		FlyCapture2::Camera camera;
		int shutterTime;
		int shutterSkip;
		int shutterAdjustSpeed;
		float autoShutterMin, autoShutterMax, autoShutterMinDiff;
		
		// Finds a camera with the given index
		FlyCapture2::PGRGuid getCameraId(int index);
	};

	// Class for capturing monocular images
	template <class T>
	class MonoFlyCapQueue: public ImageQueue<cv::Mat_<T> > {
	public:
		MonoFlyCapQueue(int cameraIndex = 0, bool force16Bit = false, bool strobe = false, bool externalTrigger = false,
			bool softwareAutoShutter = true);
		virtual ~MonoFlyCapQueue(){}
		
		void adjustAutoShutter(float min, float max, float minDiff) {
			capture.adjustAutoShutter(min, max, minDiff);
		}
			
	protected:
		virtual void queueFrame();
		
	private:
		FlyCapture<T> capture;
		bool softwareAutoShutter;
	};
	
	// Class for capturing stereo images
	template <class T>
	class StereoFlyCapQueue: public ImageQueue<std::pair<cv::Mat_<T>, cv::Mat_<T> > > {
	public:
		typedef std::pair<cv::Mat_<T>, cv::Mat_<T> > StereoPair;
		StereoFlyCapQueue(int leftCameraIndex = 0, int rightCameraIndex = 1, bool force16Bit = false,
			bool softwareAutoShutter = true);
		virtual ~StereoFlyCapQueue() {}
		
		void adjustAutoShutter(float min, float max, float minDiff) {
			leftCap.adjustAutoShutter(min, max, minDiff);
		}
	
	protected:
		virtual void queueFrame();
	
	private:
		FlyCapture<T> rightCap, leftCap;
		bool softwareAutoShutter;
	};
	
	// Class for capturing two stereo pairs
	template <class T>
	class DoubleStereoFlyCapQueue: public ImageQueue<std::pair<std::pair<cv::Mat_<T>, cv::Mat_<T>  >,
		 std::pair<cv::Mat_<T>, cv::Mat_<T> > > > {
	public:
		typedef std::pair<cv::Mat_<T>, cv::Mat_<T> > StereoPair;
		typedef std::pair<StereoPair, StereoPair> DoubleStereoPair;
		
		DoubleStereoFlyCapQueue(bool alternating, bool secondHalfRate, int left1CameraIndex = 0,
			int right1CameraIndex = 1, int left2CameraIndex = 2, int right2CameraIndex = 3,
			bool force16Bit = false, bool softwareAutoShutter = true);
		virtual ~DoubleStereoFlyCapQueue() {}
	
		void adjustAutoShutter(float min, float max, float minDiff) {
			left1Cap.adjustAutoShutter(min, max, minDiff);
			left2Cap.adjustAutoShutter(min, max, minDiff);
		}
	
	protected:
		virtual void queueFrame();
	
	private:
		bool alternating;
		bool secondHalfRate;
		bool toggle;
		FlyCapture<T> right2Cap, left2Cap, right1Cap, left1Cap;
		bool softwareAutoShutter;
		
		void enableTrigger(FlyCapture<T>& camera, bool enabled);
	};
#endif
	
	typedef MonoFlyCapQueue<unsigned char> MonoFlyCapQueue8U;
	typedef MonoFlyCapQueue<unsigned short> MonoFlyCapQueue16U;
	typedef StereoFlyCapQueue<unsigned char> StereoFlyCapQueue8U;
	typedef StereoFlyCapQueue<unsigned short> StereoFlyCapQueue16U;
	typedef DoubleStereoFlyCapQueue<unsigned char> DoubleStereoFlyCapQueue8U;
	typedef DoubleStereoFlyCapQueue<unsigned short> DoubleStereoFlyCapQueue16U;
}
#endif
