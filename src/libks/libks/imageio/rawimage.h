#ifndef KS_RAWIMAGE_H
#define KS_RAWIMAGE_H

#include <opencv2/opencv.hpp>
#include "libks/base/exception.h"

namespace ks {
	// Stores and saves a set of float values representing
	// an image. Width and height is constant
	class RawImage {
	public:
		// Converts an open cv image
		RawImage(const IplImage* img);
		// Converts an open cv image
		RawImage(const cv::Mat_<float>& img);
		// Creates a new image storing  a set of values.
		RawImage(unsigned int width, unsigned int height, float* value = NULL);
		// Creates a new image by loading values from a file
		RawImage(const char* file, bool convertNonRaw = true);
		// Copy constructor
		RawImage(const RawImage& img);
		~RawImage();
		
		// Saves the image to a file
		void save(const char* file);
		
		// Returns the float values for an image
		float* getValues() {return values;}
		const float* getValues() const {return values;}
		// Returns the value for a pixel
		float& at(int x, int y) {return values[width*y+x];}
		
		// Converts the image to an IplImage
		IplImage toIplImage() {return (IplImage)(*matrix);}
		// Converts the image to a Mat image
		cv::Mat_<float>& toMat() {return *matrix;}
		
		// Returns the height of the image
		unsigned int getWidth() const {return width;}
		// Returns the width of the image
		unsigned int getHeight() const {return height;}

		// Finds the minimum and maximum value in the image
		void findMinMax(float& min, float& max);
		// Scales the image by the given factor and adss the offset
		void addAndScale(float factor, float offset);
		
		// Exception class thrown when a decoding error occures
		class DecodingException: public Exception {
		public:
			DecodingException(): Exception(
				 "Error decoding raw image file") {}
		};
		
	private:
		float* values;
		cv::Mat_<float>* matrix;
		unsigned int width, height;
		static const unsigned int magicNumber;
		
		void initFromMat(const cv::Mat_<float>& img);
	};
}
#endif
