#include "libks/imageio/imagequeue.h"
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <exception>

namespace ks {
	using namespace boost;
	using namespace cv;
	using namespace std;
	
	template <class T>
	ImageQueue<T>::ImageQueue(bool useRos, bool allowFrameSkip, int queueSize, bool multiThreaded)
		:useRos(useRos), allowFrameSkip(allowFrameSkip), queueSize(queueSize), queue(new shared_ptr<const T>[queueSize]),
			readIndex(0), writeIndex(0), done(false), multiThreaded(multiThreaded) {
	}
	
	template <class T>
	ImageQueue<T>::~ImageQueue() {
		done = true;
		// Notify queue thread if blocked
		pushCond.notify_all();
		
		if(imageThread.get_id() != thread::id())
			imageThread.join();
	}
	
	template <class T>
	shared_ptr<const T> ImageQueue<T>::pop() {
		if(!multiThreaded)
			// Synchroneously queue frame
			while((!useRos || ros::ok()) && !done && queue[readIndex] == NULL)
				queueFrame();
		// Start the thread if not yet running
		else if(!done && imageThread.get_id() == thread::id())
			imageThread = thread(bind(&ImageQueue::threadMain, this));
			
		{
			unique_lock<mutex> lock(queueMutex);
			
			while(queue[readIndex] == NULL) {
				if((useRos && !ros::ok()) || done) {
					// No more images available. Return an empty object
					return shared_ptr<const T>();
				}
				else {
					// Queue underrun! Wait for more images
					popCond.wait(lock);
				}
			}
			
			// Get image from queue
			shared_ptr<const T> ret = queue[readIndex];
			// Clear queue slot
			queue[readIndex].reset();
			// Notify queue thread if blocked
			pushCond.notify_all();
			// Advance index
			readIndex = (readIndex + 1)%queueSize;
			
			return ret;
		} 
	}
	
	template <class T>
	void ImageQueue<T>::threadMain() {
		try {
			while(!done)
				queueFrame();
		} catch(const std::exception &e) {
			ROS_ERROR("Exception in image queueing thread: %s", e.what());
		}
	}
	
	template <class T>
	void ImageQueue<T>::waitForFreeSlot() {
		unique_lock<mutex> lock(queueMutex);
		if(queue[writeIndex] != NULL)
			pushCond.wait(lock);
	}
	
	template <class T>
	void ImageQueue<T>::push(const shared_ptr<const T>& frame) {
		unique_lock<mutex> lock(queueMutex);
		if(done)
			return; // everything has already finished
		
		if(queue[writeIndex] != NULL) {
			// Queue is full
			if(!allowFrameSkip)
				// We have to wait
				pushCond.wait(lock);
			else {
				// We can discard a frame
				queue[writeIndex].reset();
			}
		}
		
		queue[writeIndex] = frame;
		
		// Advance Index
		writeIndex = (writeIndex + 1)%queueSize;
		// Notify waiting pop()
		popCond.notify_one();
	}
	
	template <class T>
	void ImageQueue<T>::closeQueue() {
		done = true;
		// Notify waiting pop()
		popCond.notify_all();
	}
	
	// Explicitly instantiate templates
	template class ImageQueue<MonoFrame<unsigned char>::Type>;
	template class ImageQueue<StereoFrame<unsigned char>::Type>;
	template class ImageQueue<DoubleStereoFrame<unsigned char>::Type>;
	template class ImageQueue<MonoFrame<unsigned short>::Type>;
	template class ImageQueue<StereoFrame<unsigned short>::Type>;
	template class ImageQueue<DoubleStereoFrame<unsigned short>::Type>;
	template class ImageQueue<MonoFrame<float>::Type>;
	template class ImageQueue<StereoFrame<float>::Type>;
	template class ImageQueue<DoubleStereoFrame<float>::Type>;
}
