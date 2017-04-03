#include "libks/base/sdlwindow.h"

#include <string>
#include <cstdlib>
#include "libks/base/exception.h"

namespace ks {
	using namespace std;
	using namespace cv;
	using namespace boost;

	SDLWindow::SDLWindow(int width, int height, const char* title) {
		initialize(Size2i(width, height), title);
	}
	
	SDLWindow::SDLWindow(Size2i size, const char* title) {
		initialize(size, title);
	}

	SDLWindow::~SDLWindow() {
		SDL_FreeSurface(screen);
		SDL_Quit();
	}
	
	void SDLWindow::resize(Size2i size) {
		windowSize = size;
		setVideoMode();
	}
	
	void SDLWindow::setVideoMode() {
		// Initialize a screen
		screen = SDL_SetVideoMode(windowSize.width, windowSize.height, 24,
			SDL_ANYFORMAT | SDL_HWSURFACE | SDL_DOUBLEBUF /*SDL_SWSURFACE*/);
		if(screen == NULL)
			throw Exception(string("Couldn't set video mode: ") + SDL_GetError());
	}
	
	void SDLWindow::initialize(Size2i size, const char* title) {
		pressedKey = 0;
		screen = NULL;
		windowSize = size;
		
		// Initialize SDL
		if(SDL_Init(SDL_INIT_VIDEO /*| SDL_INIT_EVENTTHREAD*/) < 0)
			throw Exception(string("Could not initialize SDL: ") + SDL_GetError());
		
		setVideoMode();
		SDL_EnableUNICODE(true);
		
		if(title != NULL)		
			SDL_WM_SetCaption(title, title);
		
		processEvents(false);
	}
	
	void SDLWindow::displayImage(const Mat& image) {
		if(screen == NULL)
			return;
	
		SDL_Surface* surface = convertCvImage(image);
		// Blit image
		if(SDL_BlitSurface(surface, NULL, screen, NULL) < 0)
			throw Exception(string("BlitSurface error: ") + SDL_GetError());
		
		SDL_Flip(screen);
		//SDL_UpdateRect(screen, 0, 0, 0, 0);
		SDL_FreeSurface(surface);
	}
	
	void SDLWindow::displayStereoPair(const std::pair<cv::Mat, cv::Mat>& pair) {
		SDL_Surface* left = convertCvImage(pair.first),
			*right = convertCvImage(pair.second);
		
		SDL_Rect leftRect = {0, 0, pair.first.size().width, pair.first.size().height},
			rightRect = {pair.first.size().width, 0, pair.second.size().width, pair.second.size().height};
			
		if(SDL_BlitSurface(left, NULL, screen, &leftRect) < 0
			|| SDL_BlitSurface(right, NULL, screen, &rightRect) < 0)
			throw Exception(string("BlitSurface error: ") + SDL_GetError());
			
		SDL_Flip(screen);
		SDL_FreeSurface(left);
		SDL_FreeSurface(right);
	}
	
	void SDLWindow::displayDoubleStereoPair(const std::pair<std::pair<cv::Mat, cv::Mat>,
		std::pair<cv::Mat, cv::Mat> >& doublePair) {
		
		SDL_Surface* left1 = convertCvImage(doublePair.first.first),
			*right1 = convertCvImage(doublePair.first.second),
			*left2 = convertCvImage(doublePair.second.first),
			*right2 = convertCvImage(doublePair.second.second);
		
		int width = doublePair.first.first.size().width;
		int height = doublePair.first.first.size().height;
		
		SDL_Rect left1Rect = {0, 0, width, height},
			right1Rect = {width, 0, width, height},
			left2Rect = {0, height, width, height},
			right2Rect = {width, height, width, height};
			
		if(SDL_BlitSurface(left1, NULL, screen, &left1Rect) < 0
			|| SDL_BlitSurface(right1, NULL, screen, &right1Rect) < 0
			|| SDL_BlitSurface(left2, NULL, screen, &left2Rect) < 0
			|| SDL_BlitSurface(right2, NULL, screen, &right2Rect) < 0)
			throw Exception(string("BlitSurface error: ") + SDL_GetError());
			
		SDL_Flip(screen);
		SDL_FreeSurface(left1);
		SDL_FreeSurface(right1);
		SDL_FreeSurface(left2);
		SDL_FreeSurface(right2);
	}
	
	SDL_Surface* SDLWindow::convertCvImage(const cv::Mat& image) {
		// Identify the pixel format
		int rmask, gmask, bmask;
		if(image.channels() == 1) 
			rmask = gmask = bmask = 0xff;
		else if(image.channels() == 3) {
			if(image.elemSize1() != 1)
				throw Exception("Unsupported bit depth");
				
			rmask = 0xff0000;
			gmask = 0x00ff00;
			bmask = 0x0000ff;
		}
		else throw Exception("Unsupported channel number");
		
		// Perform conversion
		return SDL_CreateRGBSurfaceFrom((void*)image.data,
			image.size().width, image.size().height,
			image.elemSize() * 8, image.step,
			rmask, gmask, bmask, 0);
	}
	
	void SDLWindow::processEvents(bool wait) {
		SDL_Event event;
		bool waited = false;
		
		do {		
			// Get next event
			bool polled = SDL_PollEvent(&event);
			if(!polled) {
				if(wait) {
					SDL_WaitEvent(&event);
					waited = true;
				}
				else return; 
			}
				
			// Process events
			switch(event.type) {
				case SDL_QUIT:
					exit(0);
					break;
				case SDL_KEYDOWN:
					pressedKey = (int)event.key.keysym.unicode;
					break;
			}
		} while(!waited);
	}
	
	void SDLWindow::waitForKey() {
		pressedKey = 0;
		while(pressedKey == 0) {
			processEvents(true);
		}
	}
	
	int SDLWindow::getPressedKey() {
		int r = pressedKey;
		pressedKey = 0;
		return r;
	}
}