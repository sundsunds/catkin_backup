#ifndef KS_FILEQUEUE_H
#define KS_FILEQUEUE_H

#include "libks/imageio/imagequeue.h"
#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

namespace ks {
	// Template structs for automatically
	// selecting the correct opencv flags
	template <typename T>
	struct DefaultFlags {
		// Default flag is grayscale
		enum {flags = CV_LOAD_IMAGE_GRAYSCALE};
	};
	
	template <>
	struct DefaultFlags<unsigned short> {
		// Load 16-bit images unchanged
		enum {flags = CV_LOAD_IMAGE_UNCHANGED};
	};

	// Base class for all queues reading from image files
	template <class T, typename U>
	class BaseFileQueue: public ImageQueue<T> {
	public:
		BaseFileQueue(const boost::filesystem::path& directory,
			bool multiThreaded, int cvFlags);
		virtual ~BaseFileQueue() {}
			
	protected:
		virtual cv::Mat_<U> readNextFile();
		const char* getNextFileName();
		
	private:
		bool threadInitialized;
		int cvFlags; // OpenCV flags for loading images
		unsigned fileIndex;
		std::vector<std::string> imageFiles;
	};
	
	// Template for monocular image queues
	template <class T>
	class MonoFileQueue: public BaseFileQueue<typename MonoFrame<T>::Type, T > {
	public:
		MonoFileQueue(const boost::filesystem::path& directory, bool multiThreaded = true,
			int cvFlags = DefaultFlags<T>::flags)
			: BaseFileQueue<typename MonoFrame<T>::Type, T >(directory, multiThreaded, cvFlags) {}
		virtual ~MonoFileQueue() {}
	
	protected: 
		virtual void queueFrame();
	};
	
	// Template for stereo image queues
	template <class T>
	class StereoFileQueue: public BaseFileQueue<typename StereoFrame<T>::Type, T > {
	public:
		StereoFileQueue(const boost::filesystem::path& directory, bool multiThreaded = true,
			int cvFlags = DefaultFlags<T>::flags)
			: BaseFileQueue<typename StereoFrame<T>::Type, T >(directory, multiThreaded, cvFlags)
		{}
		
		virtual ~StereoFileQueue() {}
	
	protected: 
		virtual void queueFrame();
	};
	
	// Template for stereo image queues
	template <class T>
	class DoubleStereoFileQueue: public BaseFileQueue<typename DoubleStereoFrame<T>::Type, T > {
	public:
		DoubleStereoFileQueue(const boost::filesystem::path& directory, bool multiThreaded = true,
			int cvFlags = DefaultFlags<T>::flags)
			: BaseFileQueue<typename DoubleStereoFrame<T>::Type, T >(directory, multiThreaded, cvFlags)
		{}
		
		virtual ~DoubleStereoFileQueue() {}
	
	protected: 
		virtual void queueFrame();
	};
	
	
	// Class for reading raw files
	class MonoRawFileQueue: public MonoFileQueue<float> {
	public:
		MonoRawFileQueue(const boost::filesystem::path& directory, bool multiThreaded = true)
			:MonoFileQueue<float>(directory, multiThreaded) {}
			
	private:
		virtual cv::Mat_<float> readNextFile();
	};
	
	typedef MonoFileQueue<unsigned char> MonoFileQueue8U;
	typedef MonoFileQueue<unsigned short> MonoFileQueue16U;
	typedef StereoFileQueue<unsigned char> StereoFileQueue8U;
	typedef StereoFileQueue<unsigned short> StereoFileQueue16U;
	typedef DoubleStereoFileQueue<unsigned char> DoubleStereoFileQueue8U;
	typedef DoubleStereoFileQueue<unsigned short> DoubleStereoFileQueue16U;
}
#endif
