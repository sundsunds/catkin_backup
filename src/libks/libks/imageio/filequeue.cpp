#include "libks/imageio/filequeue.h"
#include "libks/imageio/rawimage.h"
#include <ros/ros.h>
#include <algorithm>
#include <cstring>
#include <pthread.h>

namespace ks {
	using namespace boost;
	using namespace boost::filesystem;
	using namespace cv;
	using namespace std;
	
	template <class T, typename U>
	BaseFileQueue<T, U>::BaseFileQueue(const path& directory, bool multiThreaded, int cvFlags)
		:ImageQueue<T>(false, false, 1, multiThreaded), threadInitialized(false), cvFlags(cvFlags), fileIndex(0) {
		
		// Crate file list
		directory_iterator end_itr; // default construction yields past-the-end
		for (directory_iterator itr(directory); itr!=end_itr; itr++) {
#if BOOST_FILESYSTEM_VERSION >= 3
			imageFiles.push_back(itr->path().native());
#else
			imageFiles.push_back(itr->path().native_file_string());
#endif
		}
		// Sort files
		sort(imageFiles.begin(), imageFiles.end());
	}
	
	template <class T, typename U>
	const char* BaseFileQueue<T, U>::getNextFileName() {
		if(fileIndex >= imageFiles.size())
			return NULL;
		else return imageFiles[fileIndex++].c_str();
	}
	
	template <class T, typename U>
	Mat_<U> BaseFileQueue<T, U>::readNextFile() {
		/*if(!threadInitialized) {
			// Increase priority of the thread
			threadInitialized = true;
			sched_param param;
			memset(&param, 0, sizeof(param));
			param.sched_priority = sched_get_priority_max(SCHED_RR);
			pthread_setschedparam(pthread_self(), SCHED_RR, &param);
		}*/
	
		Mat_<U> frame;
		const char* fileName;
		while((fileName = getNextFileName()) != NULL) {
			frame = imread(fileName, cvFlags);
			if(frame.data == NULL)
				ROS_WARN("Skipping unreadable file: %s", fileName);
			else return frame;
		}
		return frame; // Empty object
	}
	
	template <typename T>
	void MonoFileQueue<T>::queueFrame() {
		typename MonoFrame<T>::Ptr monoFrame(new Mat_<T>(this->readNextFile()));
		
		if(monoFrame->data != NULL)
            this->push(monoFrame);
		else this->closeQueue();
	}
	
	template <typename T>
	void StereoFileQueue<T>::queueFrame() {
		typename StereoFrame<T>::Ptr stereoFrame(new typename StereoFrame<T>::Type);
		stereoFrame->first = this->readNextFile();
		stereoFrame->second = this->readNextFile();
		
		if(stereoFrame->first.data != NULL && stereoFrame->second.data != NULL)
            this->push(stereoFrame);
		else this->closeQueue();
	}
	
	template <typename T>
	void DoubleStereoFileQueue<T>::queueFrame() {
		typename DoubleStereoFrame<T>::Ptr doubleFrame(new typename DoubleStereoFrame<T>::Type);
		doubleFrame->first.first = this->readNextFile();
		doubleFrame->first.second = this->readNextFile();
		doubleFrame->second.first = this->readNextFile();
		doubleFrame->second.second = this->readNextFile();
		
		if(doubleFrame->first.first.data != NULL && doubleFrame->first.second.data != NULL &&
			doubleFrame->second.first.data != NULL && doubleFrame->second.second.data != NULL)
            this->push(doubleFrame);
		else this->closeQueue();
	}
	
	Mat_<float> MonoRawFileQueue::readNextFile() {
		const char* fileName;
		while((fileName = getNextFileName()) != NULL) {
			try {
				RawImage raw(fileName);
				return raw.toMat().clone();
			} catch (const RawImage::DecodingException) {
				cerr << "Skipping unreadable file: " << fileName << endl;
			}
		}
		return Mat_<float>(); // Empty object
	}
	
	// Explicit template instantiation
	template class MonoFileQueue<unsigned char>;
	template class MonoFileQueue<unsigned short>;
	template class MonoFileQueue<float>;
	template class StereoFileQueue<unsigned char>;
	template class StereoFileQueue<unsigned short>;
	template class DoubleStereoFileQueue<unsigned char>;
	template class DoubleStereoFileQueue<unsigned short>;
}
