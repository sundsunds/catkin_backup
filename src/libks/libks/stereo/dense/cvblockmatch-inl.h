#ifndef KS_CVBLOCKMATCH_INL_H
#define KS_CVBLOCKMATCH_INL_H

#include "libks/stereo/dense/cvblockmatch.h"
#include <iostream>
#include "libks/base/typesequal.h"
#include "libks/base/exception.h"

namespace ks {
	using namespace cv;
	using namespace std;
	
	template<typename TDisp>
	CvBlockMatch<TDisp>::CvBlockMatch(int disparities, int winSize):
		dispBuffer(NULL), stereoBM(StereoBM::BASIC_PRESET, disparities, winSize)
	{
		// Conversion is required if the resulting disparity map is not
		// of type short or float
		conversionRequired = !TypesEqual<TDisp, short>::result &&
			!TypesEqual<TDisp, float>::result;
		
		subPixel = false;
		
		// Find the correct OpenCV type constant for the disparity type
		if(TypesEqual<TDisp, char>::result)
			cvDispType = CV_8S;
		else if(TypesEqual<TDisp, unsigned char>::result)
			cvDispType = CV_8U;
		else if(TypesEqual<TDisp, short>::result)
			cvDispType = CV_16S;
		else if(TypesEqual<TDisp, unsigned short>::result)
			cvDispType = CV_16U;
		else if(TypesEqual<TDisp, float>::result) {
			cvDispType = CV_32F;
			subPixel = true;
		}
		else throw Exception("Unsupported disparity datatype");
	}
	
	template<typename TDisp>
	CvBlockMatch<TDisp>::~CvBlockMatch() {
		if(conversionRequired && dispBuffer != NULL)
			delete dispBuffer;
	}
	
	template<typename TDisp>	
	void CvBlockMatch<TDisp>::match(const Mat_<unsigned char>& left,
		const Mat_<unsigned char>& right, cv::Mat_<TDisp>* out) {
		
		if(!conversionRequired)
			// If no conversion is performed we don't need to allocate a buffer
			dispBuffer = out;
		else if(dispBuffer == NULL)
			// Buffer required and not yet allocated
			dispBuffer = new Mat(left.size(), cvDispType);
		else if(dispBuffer->size() != left.size()) {
			// Input image size has changed so we need a new buffer
			delete dispBuffer;
			dispBuffer = new Mat(left.size(), cvDispType);
		}
		
		stereoBM(left, right, *dispBuffer, subPixel ? CV_32F : CV_16S);
		
		if(dispBuffer != out)
			dispBuffer->convertTo(*out, cvDispType);
	}
}

#endif
