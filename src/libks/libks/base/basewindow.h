#ifndef KS_BASEWINDOW_H
#define KS_BASEWINDOW_H

#include <opencv2/opencv.hpp>

namespace ks {
	// Creates an SDL window that displays an image
	class BaseWindow {
	public:
		// Displays a new image
		virtual void displayImage(const cv::Mat& image) = 0; 
		// Displays a stereo pair
		virtual void displayStereoPair(const std::pair<cv::Mat, cv::Mat>& pair) = 0;
		// Displays a double stereo pair
		virtual void displayDoubleStereoPair(const std::pair<std::pair<cv::Mat, cv::Mat>, std::pair<cv::Mat, cv::Mat> >& doublePair) = 0;
		
		// Resizes the window
		void resize(int width, int height) {
			resize(cv::Size2i(width, height));
		}
		
		virtual void resize(cv::Size2i size) = 0;
		virtual cv::Size2i getSize() const = 0;
		
		// Processes user interface events. Should be called periodically
		// from the main thread
		virtual void processEvents(bool wait) = 0;
		
		// Waits for a keypress
		virtual void waitForKey() = 0;
		// Returns the pressed key or 0
		virtual int getPressedKey() = 0;
	};
}

#endif
