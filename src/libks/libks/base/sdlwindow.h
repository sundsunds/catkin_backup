#ifndef KS_SDLWINDOW_H
#define KS_SDLWINDOW_H

#include <SDL.h>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <string>
#include <SDL_main.h>
#include "libks/base/basewindow.h"

namespace ks {
	// Creates an SDL window that displays an image
	class SDLWindow: public BaseWindow {
	public:
		SDLWindow(int width, int height, const char* title = NULL);
		SDLWindow(cv::Size2i size, const char* title = NULL);
		virtual ~SDLWindow();
		
		// Displays a new image
		virtual void displayImage(const cv::Mat& image); 
		// Displays a stereo pair
		virtual void displayStereoPair(const std::pair<cv::Mat, cv::Mat>& pair);
		// Displays a double stereo pair
		virtual void displayDoubleStereoPair(const std::pair<std::pair<cv::Mat, cv::Mat>, std::pair<cv::Mat, cv::Mat> >& doublePair);
		
		// Resizes the window
		using BaseWindow::resize;
		virtual void resize(cv::Size2i size);
		virtual cv::Size2i getSize() const {return windowSize;}
		
		// Processes user interface events. Should be called periodically
		// from the main thread
		virtual void processEvents(bool wait);
		
		// Waits for a keypress
		virtual void waitForKey();
		// Returns the pressed key or 0
		virtual int getPressedKey();
		
	private:
		SDL_Surface* screen;
		cv::Size2i windowSize;
		int pressedKey;
		
		void setVideoMode();
		void initialize(cv::Size2i size, const char* title);
		
		//Converts an OpenCV image to SDL image
		SDL_Surface* convertCvImage(const cv::Mat& image);
	};
}

#endif
