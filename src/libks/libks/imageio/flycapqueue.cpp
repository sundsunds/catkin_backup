//#include "/home/ait_jellal/projects/ws_mav/src/libks/libks/imageio/flycapqueue.h"
//#include "/home/ait_jellal/projects/ws_mav/src/libks/libks/base/exception.h"
#include "libks/imageio/flycapqueue.h"
#include "libks/base/exception.h"

#include <boost/bind.hpp>
#include <boost/timer.hpp>
#include <utility>
#include <algorithm>
#include <ros/ros.h>
#include <limits>

#include <iostream>

namespace ks {
	using namespace std;
	using namespace cv;
	using namespace boost;
	using namespace FlyCapture2;
	
	// Definitions of capturing formats
	template <typename T>
	struct CaptureFormat {
		enum {
			pixelFormat = PIXEL_FORMAT_MONO8,
			videoMode = VIDEOMODE_640x480Y8
		};
	};
	
	template <>
	struct CaptureFormat<unsigned short> {
		enum {
			pixelFormat = PIXEL_FORMAT_MONO16,
			videoMode = VIDEOMODE_640x480Y16
		};
	};
	
	template <typename T>
	FlyCapture2::BusManager FlyCapture<T>::busManager;
	
	template <typename T>
	const int FlyCapture<T>::shutterMax = 200; // Actual max is 533, but we need to leave some extra time
	
	template <typename T>
	FlyCapture<T>::FlyCapture(int cameraIndex, bool force16Bit, bool strobe, bool externalTrigger, bool halfFrameRate)
		: cameraIndex(cameraIndex), autoShutterMin(0.15), autoShutterMax(0.98),
		autoShutterMinDiff(4.0) {
		
		unsigned int cameras;
		if(busManager.GetNumOfCameras(&	cameras) != PGRERROR_OK)
			throw Exception("Querying number of cameras failed!");
		
		if(cameras == 0)
			throw Exception("No cameras connected!");
			
		PGRGuid cameraId = getCameraId(cameraIndex);
		if(camera.Connect(&cameraId) != PGRERROR_OK)
			throw Exception("Error connecting to camera!");
			
		FrameRate rate;
		if(halfFrameRate)
			rate = externalTrigger ? FRAMERATE_30 : FRAMERATE_15;
		else rate = externalTrigger ? FRAMERATE_60 : FRAMERATE_30;
		
		if(camera.SetVideoModeAndFrameRate(
			force16Bit ? VIDEOMODE_640x480Y16 : (VideoMode)CaptureFormat<T>::videoMode, rate) != PGRERROR_OK)
			throw Exception("Error setting video mode and frame rate!");
			
		// Setting camera to little endian
		unsigned int value = 0;
		if(camera.ReadRegister(0x1048, &value) != PGRERROR_OK)
			throw Exception("Error reading camera register!");
		if(camera.WriteRegister(0x1048, value & (~31)) != PGRERROR_OK)
			throw Exception("Error writing camera register!");
		
		if(camera.StartCapture() != PGRERROR_OK)
			throw Exception("Error starting capturing!");
		
		FC2Config config;
		if(externalTrigger)
			// Set a timeout for frame grabbing for all except the master camera
			config.grabTimeout = 10;
		config.numBuffers = 2;// Enable double buffering
		if(camera.SetConfiguration(&config) != PGRERROR_OK)
			throw Exception("Error setting camera configuration!");
		
		// Enable/disable strobe on pin 1-3
		if(strobe) {
			for(int pin=0; pin<4; pin++) {
				camera.SetGPIOPinDirection(pin, 1); // Make output
				
				StrobeControl strobeCtrl;
				strobeCtrl.source = pin;
				strobeCtrl.onOff = 1;
				strobeCtrl.delay = 0;
				strobeCtrl.duration = 1.0;
				
				strobeCtrl.polarity = 1;
				camera.SetStrobe(&strobeCtrl);
			}
		}
		
		if(externalTrigger) {		
			// Enable/disable external trigger on pin 0
			camera.SetGPIOPinDirection(0, 0); // Make input
			
			TriggerMode mode;
			mode.onOff = 1;
			mode.source = 0;
			mode.mode = 0;
			mode.parameter = 0;
			mode.polarity = 1;
			camera.SetTriggerMode(&mode);
		}
		
		shutterSkip = 0;
		shutterTime = -1;
		shutterAdjustSpeed = 0;
	}	
	
	template <typename T>
	FlyCapture<T>::~FlyCapture() {
		camera.Disconnect();
	}
	
	template <typename T>
	PGRGuid FlyCapture<T>::getCameraId(int index) {
		// Our index is the camera number in camera list sorted
		// by serial numbers
		
		unsigned int num;
		vector<unsigned int> cameras;
		
		if(busManager.GetNumOfCameras(&num) != PGRERROR_OK)
			throw Exception("Error obtaining cameras number");
		
		for(unsigned int i=0; i<num; i++) {
			unsigned int serial;
			if(busManager.GetCameraSerialNumberFromIndex(i, &serial) != PGRERROR_OK)
				throw Exception("Error optaining camera serial number!");
			cameras.push_back(serial);
		}
		
		if(index < 0 || index >= (int)cameras.size())
			throw Exception("Invalid camera index");
		
		sort(cameras.begin(), cameras.end());
		
		PGRGuid cameraId;
		if(busManager.GetCameraFromSerialNumber(cameras[index], &cameraId) != PGRERROR_OK)
			throw Exception("Error optaining camera GUID!");
		return cameraId;
	}
	
	
	template <typename T>
	Mat_<T> FlyCapture<T>::grabFrame() {
		if(camera.RetrieveBuffer(&grabBuffer) != PGRERROR_OK) {
			ROS_ERROR("Frame grabbing timeout! (Possibly out of sync)");
			return Mat_<T>();
		}
			
		T* data = NULL;
		size_t stride = 0;
		
		if(grabBuffer.GetPixelFormat() != (unsigned int)CaptureFormat<T>::pixelFormat) {
			// Format conversion is neccessary
			// Using this feature for stereo is dangerous as it might destroy
			// the synchronization due to extra latency
			if(grabBuffer.Convert((PixelFormat)CaptureFormat<T>::pixelFormat, &conversionBuffer) != PGRERROR_OK)
				throw Exception("Error converting captured image!");
			data = (T*) conversionBuffer.GetData();
			stride = (size_t) conversionBuffer.GetStride();
		} else {
			// No conversion required
			data = (T*) grabBuffer.GetData();
			stride = (size_t) grabBuffer.GetStride();
		}
		
		Mat_<T> cvImage((int)grabBuffer.GetRows(), (int)grabBuffer.GetCols(), data, stride);
		return cvImage;
	}
	
	template <typename T>
	void FlyCapture<T>::waitForNextFrame() {
		// Not so efficient but ther does not seem to be another way
		camera.RetrieveBuffer(&grabBuffer);
	}
	
	template <typename T>
	void FlyCapture<T>::setShutterTime(int time) {
		if(time != shutterTime) {
			shutterTime = time;
			Property prop(SHUTTER);
			prop.autoManualMode = false; // Manual mode
			prop.valueA = shutterTime;
			if(camera.SetProperty(&prop) != PGRERROR_OK)
				throw Exception("Error adjusting shutter time!");
		}
	}
	
	template <typename T>
	int FlyCapture<T>::updateShutterTime(const cv::Mat_<T>& img) {
		if(shutterTime == -1)
			return 64; // Start with 4.0 ms
	
		// Only update every N frames
		shutterSkip--;
		if(shutterSkip > 0)
			return shutterTime;
		shutterSkip = 5;
	
		unsigned int overExp = 0, underExp = 0;
		
		const T maxI = numeric_limits<T>::max() * autoShutterMax,
			minI = numeric_limits<T>::max() * autoShutterMin;
		
		// TODO: Optimize implementation with SSE if neccessary
		for(int y=0; y<img.rows; y+=2)
			for(int x=0; x<img.cols; x+=2) {
				underExp += img(y,x) <= minI;
				overExp += img(y,x) >= maxI;
			}
		
		// Update shutter speed
		if(overExp > autoShutterMinDiff*underExp)
			shutterAdjustSpeed = max(min(-1, shutterAdjustSpeed-1), -16);
		else if(underExp > autoShutterMinDiff*overExp)
			shutterAdjustSpeed = min(max(1, shutterAdjustSpeed+1), 16);
		else shutterAdjustSpeed = 0;
		
		// Update and clamp to valid range for 30 Hz
		return max(1, min(shutterTime + shutterAdjustSpeed, shutterMax));
	}

	template <typename T>
	MonoFlyCapQueue<T>::MonoFlyCapQueue(int cameraIndex, bool force16Bit, bool strobe, bool externalTrigger,
		bool softwareAutoShutter)
		:ImageQueue<Mat_<T> >(false, true, 1, true), capture(cameraIndex, force16Bit, strobe, externalTrigger),
		softwareAutoShutter(softwareAutoShutter)
	{}
	
	template <typename T>
	void MonoFlyCapQueue<T>::queueFrame() 
	{
		typename MonoFrame<T>::Ptr frame(new Mat_<T>(capture.grabFrame().clone()));
		if(frame->data != NULL)
		this->push(frame);
		
		if(softwareAutoShutter)
			capture.setShutterTime(capture.updateShutterTime(*frame));
	}
	
	template <typename T>
	StereoFlyCapQueue<T>::StereoFlyCapQueue(int leftCameraIndex, int rightCameraIndex, bool force16Bit,
		bool softwareAutoShutter)
		:ImageQueue<StereoPair>(false, true, 1, true), rightCap(rightCameraIndex, force16Bit, false, true),
			leftCap(leftCameraIndex, force16Bit, true, false), softwareAutoShutter(softwareAutoShutter) {
	}
	
	template <typename T>
	void StereoFlyCapQueue<T>::queueFrame() {
		Mat_<T> left = leftCap.grabFrame();
		if(left.data == NULL)
			return;
		
		Mat_<T> right = rightCap.grabFrame();
		if(right.data == NULL)
			return;
		
		typename StereoFrame<T>::Ptr frame(new typename StereoFrame<T>::Type(
			left.clone(), right.clone())); 
		this->push(frame);
		
		if(softwareAutoShutter) {
			int shutter = leftCap.updateShutterTime(frame->first);
			leftCap.setShutterTime(shutter);
			rightCap.setShutterTime(shutter*2); // Double frame rate, hence double value
		}
	}
	
	template <typename T>
	DoubleStereoFlyCapQueue<T>::DoubleStereoFlyCapQueue(bool alternating, bool secondHalfRate,
		int left1CameraIndex, int right1CameraIndex, int left2CameraIndex, int right2CameraIndex,
		bool force16Bit, bool softwareAutoShutter)
		:ImageQueue<DoubleStereoPair>(false, true, 1, true), alternating(alternating), secondHalfRate(secondHalfRate),
			toggle(false), right2Cap(right2CameraIndex, force16Bit, false, true, secondHalfRate),
			left2Cap(left2CameraIndex, force16Bit, false, true, secondHalfRate), right1Cap(right1CameraIndex, force16Bit, false, true),
			left1Cap(left1CameraIndex, force16Bit, true, false), softwareAutoShutter(softwareAutoShutter) {
	}
	
	template <typename T>
	void DoubleStereoFlyCapQueue<T>::enableTrigger(FlyCapture<T>& camera, bool enabled) {
		TriggerMode mode;
		mode.onOff = 1;
		mode.source = enabled ? 0 /*valid*/ : 1 /*invalid*/;
		mode.mode = 0;
		mode.parameter = 0;
		mode.polarity = 1;
		camera.getCamera().SetTriggerMode(&mode);
	}
	
	template <typename T>
	void DoubleStereoFlyCapQueue<T>::queueFrame() {
		Mat_<T> left1, right1, left2, right2;
	
		// Capture front stereo pair
		if(toggle || !alternating) {
			left1 = left1Cap.grabFrame();
			if(left1.data == NULL)
				return;
				
			right1 = right1Cap.grabFrame();
			if(right1.data == NULL)
				return;
		}
		
		toggle = !toggle;

		// Capture bottom stereo pair
		if(!toggle || (!alternating && !secondHalfRate)) {
			if(alternating)
				left1Cap.waitForNextFrame(); // Sync to master
		
			left2 = left2Cap.grabFrame();
			if(left2.data != NULL)
				right2 = right2Cap.grabFrame();
			
			if(secondHalfRate) {
				enableTrigger(left2Cap, false);
				enableTrigger(right2Cap, false);
			}
			
			if(right1.data == NULL || right2.data == NULL)
				return;
		} else if(secondHalfRate) {
			enableTrigger(left2Cap, true);
			enableTrigger(right2Cap, true);
		}
		
		typename DoubleStereoFrame<T>::Ptr frame(new typename DoubleStereoFrame<T>::Type(
			typename StereoFrame<T>::Type(left1.clone(), right1.clone()),
			typename StereoFrame<T>::Type(left2.clone(), right2.clone())));
		this->push(frame);

		if(softwareAutoShutter) {
			if(left1.data != NULL) {
				int pair1Shutter = left1Cap.updateShutterTime(frame->first.first);
				left1Cap.setShutterTime(pair1Shutter);
				right1Cap.setShutterTime(pair1Shutter *2); // Double frame rate, hence double value
			}

			if(left2.data != NULL) {
				int pair2Shutter = left2Cap.updateShutterTime(frame->second.first);
				left2Cap.setShutterTime(pair2Shutter);
				right2Cap.setShutterTime(pair2Shutter);
			}
		}
	}
	
	// Explicit template instantiation
	template class MonoFlyCapQueue<unsigned char>;
	template class MonoFlyCapQueue<unsigned short>;
	template class StereoFlyCapQueue<unsigned char>;
	template class StereoFlyCapQueue<unsigned short>;
	template class DoubleStereoFlyCapQueue<unsigned char>;
	template class DoubleStereoFlyCapQueue<unsigned short>;
}
