#ifndef KS_LOCALCONTRASTFILTER_H
#define KSLCALCONTRASTFILTER_H

#include <opencv2/opencv.hpp>
#include <limits>
#include "libks/base/typesequal.h"
#include "libks/base/exception.h"

namespace ks {
	// Class for filtering an image to enance the local contrast
	class LocalContrastFilter {
	public:
		template <int WINDOW_SIZE, typename T>
		static void process(const cv::Mat_<T>& image, cv::Mat_<T>& dest, float factor) {
			// Find maximum intensity
			T imax;
			if(TypesEqual<T, unsigned char>::result)
				imax = (T)255;
			else if(TypesEqual<T, unsigned short>::result)
				imax = (T)65535;
			else if(TypesEqual<T, float>::result)
				imax = (T)1;
			else throw Exception("Invalid pixel type");
			
			if(dest.size() != image.size())
				dest = cv::Mat_<T>(image.rows, image.cols, (T)0);
						
			// Perform filtering
			for(int y=WINDOW_SIZE/2; y<image.rows-WINDOW_SIZE/2; y++)
				for(int x=WINDOW_SIZE/2; x<image.cols-WINDOW_SIZE/2; x++) {
				
					// Find minimum and maximum
					T min = image(y,x), max = image(y,x);
					for(int dy=-WINDOW_SIZE/2; dy<=WINDOW_SIZE/2; dy++)
						for(int dx=-WINDOW_SIZE/2; dx<=WINDOW_SIZE/2; dx++) {
							if(image(y+dy, x+dx) > max)
								max = image(y+dy, x+dx);
							else if(image(y+dy, x+dx) < min)
								min = image(y+dy, x+dx);
						}
					
					// Scale contrast
					T mean = (max + min) / 2;
					
					T range, dist;
					if(imax - max < min) {
						range = max - mean;
						dist = imax - max;
					}
					else {
						range = mean - min;
						dist = min;
					}
					
					if(range == 0)
						dest(y, x) = image(y, x);
					else {
						float scaling = factor * ((range + dist) / range) + (1.0 - factor)*1.0;
						dest(y, x) = (float)(image(y,x) - mean) * scaling + mean;
					}
				}
		}
	};
}

#endif