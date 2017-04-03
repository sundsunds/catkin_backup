#include "libks/base/cvwindow.h"
#include <string>
#include <cstdlib>
#include "libks/base/exception.h"
#include "libks/imageproc/imageconversion.h"

namespace ks {
	using namespace std;
	using namespace cv;

	CVWindow::CVWindow(int width, int height, const char* title) {
		initialize(Size2i(width, height), title);
	}
	
	CVWindow::CVWindow(Size2i size, const char* title) {
		initialize(size, title);
	}

	CVWindow::~CVWindow() {
		destroyWindow(windowTitle);
	}
	
	void CVWindow::resize(Size2i size) {
		windowSize = size;
		resizeWindow(windowTitle, size.width, size.height);
	}
	
	void CVWindow::initialize(Size2i size, const char* title) {
		windowTitle = title;
		pressedKey = 0;
		windowSize = size;
		
		namedWindow(windowTitle, 0 /*no autosize*/);
		processEvents(false);
		resizeWindow(windowTitle, size.width, size.height);
		processEvents(false);
	}
	
	void CVWindow::displayImage(const Mat& image) {
		imshow(windowTitle, image);
	}
	
	void CVWindow::displayStereoPair(const std::pair<cv::Mat, cv::Mat>& pair) {
		Mat_<Vec3b> screen(pair.first.rows, pair.first.cols + pair.second.cols);
		Mat_<Vec3b> dest;
		
		ImageConversion::convertToColor(pair.first, &dest);
		dest.copyTo(screen(Rect(0, 0, pair.first.cols, pair.first.rows)));
		
		ImageConversion::convertToColor(pair.second, &dest);
		dest.copyTo(screen(Rect(pair.first.cols, 0, pair.second.cols, pair.first.rows)));
		
		imshow(windowTitle, screen);
	}
	
	void CVWindow::displayDoubleStereoPair(const std::pair<std::pair<cv::Mat, cv::Mat>,
		std::pair<cv::Mat, cv::Mat> >& doublePair) {
		
		int width = doublePair.first.first.size().width;
		int height = doublePair.first.first.size().height;
			
		Mat_<Vec3b> screen(2*width, 2*height);
		Mat_<Vec3b> dest;
		
		ImageConversion::convertToColor(doublePair.first.first, &dest);
		dest.copyTo(screen(Rect(0, 0, width, height)));
		
		ImageConversion::convertToColor(doublePair.first.second, &dest);
		dest.copyTo(screen(Rect(width, 0, width, height)));
		
		ImageConversion::convertToColor(doublePair.second.first, &dest);
		dest.copyTo(screen(Rect(0, height, width, height)));
		
		ImageConversion::convertToColor(doublePair.second.second, &dest);
		dest.copyTo(screen(Rect(width, height, width, height)));
		
		imshow(windowTitle, screen);
	}
	
	void CVWindow::processEvents(bool wait) {
		if(wait) {
			pressedKey = waitKey(0);
		}
		else {
			int result = waitKey(1);
			if(result != -1)
				pressedKey = result;
		}
	}
	
	void CVWindow::waitForKey() {
		processEvents(true);
	}
	
	int CVWindow::getPressedKey() {
		if(pressedKey == -1)
			return 0; // Be consistent to SDL
	
		int r = pressedKey;
		pressedKey = 0;
		return r;
	}
}
