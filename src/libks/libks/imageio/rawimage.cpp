#include "libks/imageio/rawimage.h"
#include <fstream>

namespace ks {
	using namespace std;
	using namespace cv;

	const unsigned int RawImage::magicNumber=0xA31F9317; // Some random number

	RawImage::RawImage(const IplImage* img) {
		initFromMat(cvarrToMat(img));
	}

	RawImage::RawImage(const Mat_<float>& img) {
		initFromMat(img);
	}

	RawImage::RawImage(unsigned int width, unsigned int height, float* values) {
		this->width = width;
		this->height = height;
		this->values = new float[width*height];
		if(values != NULL)
			memcpy(this->values, values, sizeof(float) * width*height);
		else memset(this->values, 0, sizeof(float)*width*height);
		
		matrix = new Mat_<float>(height, width, this->values);
	}

	RawImage::RawImage(const char* file, bool convertNonRaw) {
		fstream strm(file, ios::in | ios::binary);
		unsigned int magic = 0;
		strm.read((char*)&magic, sizeof(magic));
		if(magic != magicNumber) {
			if(!convertNonRaw)
				throw DecodingException();
			Mat_<float> image = imread(file, CV_LOAD_IMAGE_GRAYSCALE);
			if(image.data == NULL)
				throw DecodingException();
			else initFromMat(image);
		}
		else {
			strm.read((char*)&width, sizeof(width));
			strm.read((char*)&height, sizeof(height));
			values = new float[width*height];
			strm.read((char*)values, sizeof(float) * width*height);
			matrix = new Mat_<float>(height, width, values);
		}
	}

	void RawImage::initFromMat(const cv::Mat_<float>& img) {
		width = img.cols;
		height = img.rows;
		values = new float[width*height];
		memcpy(values, img.data, sizeof(float) * width*height);
		matrix = new Mat_<float>(height, width, values);
	}

	RawImage::RawImage(const RawImage& img) {
		width = img.width;
		height = img.height;
		values = new float[width*height];
		memcpy(values, img.values, sizeof(float) * width*height);
		matrix = new Mat_<float>(height, width, values);
	}	


	RawImage::~RawImage() {
		delete []values;
		delete matrix;
	}

	void RawImage::save(const char* file) {
		fstream strm(file, ios::out | ios::binary);
		strm.write((char*)&magicNumber, sizeof(magicNumber));
		strm.write((char*)&width, sizeof(width));
		strm.write((char*)&height, sizeof(height));
		strm.write((char*)values, sizeof(float) * width*height);
	}

	void RawImage::findMinMax(float &min, float &max) {
		min = max = *values;
		
		for(unsigned int i = 1; i < width * height; i++) {
			if(values[i] < min)
				min = values[i];
			else if(values[i] > max)
				max = values[i];
		}
	}

	void RawImage::addAndScale(float offset, float factor) {
		for(unsigned int i=0; i < width*height; i++)
			values[i] = (values[i] + offset) * factor;
	}
}
