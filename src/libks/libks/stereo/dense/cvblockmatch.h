#ifndef KS_CVBLOCKMATCH_H
#define KS_CVBLOCKMATCH_H

#include "libks/stereo/dense/densestereoalgorithm.h"

namespace ks {
	// Wrapper for the OpenCV block matching stereo algorithm
	template<typename TDisp>
	class CvBlockMatch: public DenseStereoAlgorithm<TDisp, unsigned char> {
	public:
		CvBlockMatch(int disparities, int winSize=21);
		virtual ~CvBlockMatch();
		
		// Performs the stereo matching
		virtual void match(const cv::Mat_<unsigned char>& left,
			const cv::Mat_<unsigned char>& right, cv::Mat_<TDisp>* out);
		
	private:
		// True if a converson of the disparity map is required
		bool conversionRequired;
		// Buffer required if disparity map conversion is neccessary
		cv::Mat* dispBuffer;
		// OpenCV type constant for the disparity map type		
		int cvDispType;
		// OpenCV stereo matching object
		cv::StereoBM stereoBM;
		// True if subpixel matching is performed
		bool subPixel;
	};
}

#endif
