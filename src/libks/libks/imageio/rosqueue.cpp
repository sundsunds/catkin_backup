#include "libks/imageio/rosqueue.h"
#include "libks/base/exception.h"
#include "libks/base/dependentsharedptr.h"
#include <boost/bind.hpp>
#include <climits>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace ks {
	using namespace std;
	using namespace boost;
	using namespace boost::posix_time;
	using namespace cv;
	using namespace ros;
	using namespace cv_bridge;

	template <class T>
	BaseRosQueue<T>::BaseRosQueue(NodeHandle& handle, const vector<int>& cameraIndices, bool shared, double rate)
		: cameraIndices(cameraIndices), callbackHelper(this, rate), bufferSeqId(0), readSeqId(0), shared(shared) {
		
		if(!shared) {
			subscriber.reset(new Subscriber(
				handle.subscribe("capture", 1, &CallbackHelper::callbackDefault, &callbackHelper)));
		} else {
			subscriber.reset(new Subscriber(
				handle.subscribe("capture_shared", 1, &CallbackHelper::callbackShared, &callbackHelper)));
		}
		
		imageBuffers.resize(cameraIndices.size());
	}

	template <class T>
	bool BaseRosQueue<T>::CallbackHelper::checkCapture(const ros::Time& stamp) {
		if(rate <=0)
			return true; // No rate limit
		
		ros::Duration rateDuration(1.0/rate);
		
		// Reset if we exceeded twice the waiting time
		if(fabs((nextCapture - stamp).toSec()) > 2.0/rate) {
			nextCapture = stamp + rateDuration;
			return true;
		}
		else if(stamp >= nextCapture) {
			nextCapture += rateDuration;
			return true;
		} else return false;
	}
	
	template <class T>
    void BaseRosQueue<T>::CallbackHelper::callbackDefault(const libks_msgs::ImageSet::ConstPtr& msg) {
		if(!checkCapture(msg->header.stamp))
			return;
		
		{
			unique_lock<mutex> lock(obj->bufferMutex);
			obj->bufferSeqId = msg->header.seq;
			obj->msgTime = msg->header.stamp;
			
			// Reset all buffers
			for(unsigned int i=0; i<obj->imageBuffers.size(); i++)
				obj->imageBuffers[i].reset();
			
			// Fill buffers
			for(unsigned int i=0; i<obj->cameraIndices.size(); i++) {
				if(obj->cameraIndices[i] >= (int)msg->images.size())
					return; // Not enough images in message
					
				obj->imageBuffers[i] = toCvShare(msg->images[obj->cameraIndices[i]], msg);
			}
		}
		
		obj->frameCondition.notify_all();
	}
	
	template <class T>
    void BaseRosQueue<T>::CallbackHelper::callbackShared(const libks_msgs::SharedImageSet::ConstPtr& msg) {
		if(!checkCapture(msg->header.stamp))
			return;
		
		{
			unique_lock<mutex> lock(obj->bufferMutex);
			obj->bufferSeqId = msg->header.seq;
			obj->msgTime = msg->header.stamp;
			
			// Reset all buffers
			for(unsigned int i=0; i<obj->imageBuffers.size(); i++)
				obj->imageBuffers[i].reset();
			
			// Fill buffers
			for(unsigned int i=0; i<obj->cameraIndices.size(); i++) {
				if(obj->cameraIndices[i] >= (int)msg->imagePtrs.size())
					return; // Not enough images in message
					
				// Create a fake cv_bridge::CvImage with shared image data
				obj->imageBuffers[i] = cv_bridge::CvImageConstPtr(
					new CvImage(msg->header, "mono8", *((Mat*)msg->imagePtrs[obj->cameraIndices[i]])));
			}
		}
		
		obj->frameCondition.notify_all();
	}
	
	template <class T>
	bool BaseRosQueue<T>::readNextImages(cv_bridge::CvImageConstPtr images[]) {
		unique_lock<mutex> lock(bufferMutex);
		
		for(unsigned int i=0; i<imageBuffers.size(); i++) {
			if(imageBuffers[i] == NULL)
				return false; // Frame is not complete
				
			images[i] = imageBuffers[i];
			imageBuffers[i].reset();
		}
		
		readSeqId = bufferSeqId;
		return true;
	}
	
	template <class T>
	void BaseRosQueue<T>::waitForCompleteFrame() {
		unique_lock<mutex> lock(bufferMutex);
		frameCondition.timed_wait(lock, seconds(1));
	}
	
	// Mono Queue
	
	template <typename T>
	MonoRosQueue<T>::MonoRosQueue(NodeHandle& handle, bool shared, bool getIncomplete, int cam0, double rate)
		: ImageQueue<typename MonoFrame<T>::Type>(true, true, 1, false), warned(false),
		getIncomplete(getIncomplete) {
		
		vector<int> cameraIndices;
		cameraIndices.push_back(cam0);
		baseQueue.reset(new BaseRosQueue<T>(handle, cameraIndices, shared, rate));
	}
	
	template <typename T>
	void MonoRosQueue<T>::queueFrame() {
		cv_bridge::CvImageConstPtr images[4];
		while(true) {
			if(!ros::ok()) {
				//this->closeQueue();
				return;
			}
		
			baseQueue->readNextImages(images);
			if(images[0] != NULL && (getIncomplete || images[0]->image.data != NULL))
				break;
			baseQueue->waitForCompleteFrame();
		}
		
		typename MonoFrame<T>::Ptr framePtr;
		if((size_t(images[0]->image.data) & 0xF) == 0)
			framePtr = DependentSharedPtr<typename MonoFrame<T>::Type>::create1(
				images[0], new typename MonoFrame<T>::Type(images[0]->image));
		else {
			// Image is not 16-byte aligned. Might happen when a bag is replayed.
			// We have to clone the image to get proper alignment
			if(!warned) {
				ROS_WARN("Image data not 16-byte aligned!");
				warned = true;
			}
			framePtr.reset(new typename MonoFrame<T>::Type(images[0]->image.clone()));
		}
        this->push(framePtr);
	}
	
	// Stereo Queue
	
	template <typename T>
	StereoRosQueue<T>::StereoRosQueue(NodeHandle& handle, bool shared, bool getIncomplete,
		int cam0, int cam1, double rate)
		: ImageQueue<typename StereoFrame<T>::Type>(true, true, 1, false), warned(false),
		getIncomplete(getIncomplete) {
		
		vector<int> cameraIndices;
		cameraIndices.push_back(cam0);
		cameraIndices.push_back(cam1);
		baseQueue.reset(new BaseRosQueue<T>(handle, cameraIndices, shared, rate));
	}
	
	template <typename T>
	void StereoRosQueue<T>::queueFrame() {
		cv_bridge::CvImageConstPtr images[4];
		while(true) {
			if(!ros::ok()) {
				//this->closeQueue();
				return;
			}
			
			baseQueue->readNextImages(images);
			if(images[0] != NULL && images[1] != NULL &&
				(getIncomplete || (images[0]->image.data != NULL && images[1]->image.data != NULL)))
				break;
			baseQueue->waitForCompleteFrame();
		}
		
		typename StereoFrame<T>::Ptr framePtr;
		if((size_t(images[0]->image.data) & 0xF) == 0 && (size_t(images[1]->image.data) & 0xF) == 0)
			framePtr = DependentSharedPtr<typename StereoFrame<T>::Type>::create2(
				images[0], images[1], new typename StereoFrame<T>::Type(images[0]->image, images[1]->image));
		else {
			// Image is not 16-byte aligned. Might happen when a bag is replayed.
			// We have to clone the image to get proper alignment
			if(!warned) {
				ROS_WARN("Image data not 16-byte aligned!");
				
				warned = true;
			}
			framePtr.reset(new typename StereoFrame<T>::Type(images[0]->image.clone(), images[1]->image.clone()));
		}
        this->push(framePtr);
	}

	// Double Stereo Queue
	
	template <typename T>
	DoubleStereoRosQueue<T>::DoubleStereoRosQueue(NodeHandle& handle, bool shared, bool geIncomlete,
		int cam0, int cam1, int cam2, int cam3, double rate)
		: ImageQueue<typename DoubleStereoFrame<T>::Type>(true, true, 1, false), warned(false),
		getIncomplete(getIncomplete) {
		
		vector<int> cameraIndices;
		cameraIndices.push_back(cam0);
		cameraIndices.push_back(cam1);
		cameraIndices.push_back(cam2);
		cameraIndices.push_back(cam3);
		baseQueue.reset(new BaseRosQueue<T>(handle, cameraIndices, shared, rate));
	}
	
	template <typename T>
	void DoubleStereoRosQueue<T>::queueFrame() {
		cv_bridge::CvImageConstPtr images[4];
		while(true) {
			if(!ros::ok()) {
				this->closeQueue();
				return;
			}
			
			baseQueue->readNextImages(images);
			if(images[0] != NULL && images[1] != NULL && images[2] != NULL && images[3] != NULL &&
				(getIncomplete || (
				images[0]->image.data != NULL && images[1]->image.data != NULL &&
				images[2]->image.data != NULL && images[3]->image.data != NULL)))
				break;
			baseQueue->waitForCompleteFrame();
		}
		
		typename DoubleStereoFrame<T>::Ptr framePtr;
		if((size_t(images[0]->image.data) & 0xF) == 0 && (size_t(images[1]->image.data) & 0xF) == 0 &&
			(size_t(images[2]->image.data) & 0xF) == 0 && (size_t(images[3]->image.data) & 0xF) == 0)
			framePtr = DependentSharedPtr<typename DoubleStereoFrame<T>::Type>::create4(
				images[0], images[1], images[2], images[3],
				new typename DoubleStereoFrame<T>::Type(
					typename StereoFrame<T>::Type(images[0]->image, images[1]->image),
					typename StereoFrame<T>::Type(images[2]->image, images[3]->image)
				));
		else {
			// Image is not 16-byte aligned. Might happen when a bag is replayed.
			// We have to clone the image to get proper alignment
			if(!warned) {
				ROS_WARN("Image data not 16-byte aligned!");
				warned = true;
			}
			framePtr.reset(new typename DoubleStereoFrame<T>::Type(
				typename StereoFrame<T>::Type(images[0]->image.clone(), images[1]->image.clone()),
				typename StereoFrame<T>::Type(images[2]->image.clone(),	images[3]->image.clone())));
		}
        this->push(framePtr);
	}

	
	// Explicit template instantiations
	template class MonoRosQueue<unsigned char>;
	template class MonoRosQueue<unsigned short>;
	template class StereoRosQueue<unsigned char>;
	template class StereoRosQueue<unsigned short>;
	template class DoubleStereoRosQueue<unsigned char>;
	template class DoubleStereoRosQueue<unsigned short>;
}
