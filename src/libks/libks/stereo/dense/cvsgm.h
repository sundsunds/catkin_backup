#ifndef KS_CVSGM_H
#define KS_CVSGM_H

#include "libks/stereo/dense/densestereoalgorithm.h"

namespace ks {
	// Wrapper for the OpenCV SGM stereo algorithm
	template<typename TDisp, typename TPix>
	class CvSGM: public DenseStereoAlgorithm<TDisp, TPix> {
	public:
		CvSGM(int dispMin, int dispMax, int winSize = 1);
		virtual ~CvSGM() {}
		
		// Performs the stereo matching
		virtual void match(const cv::Mat_<TPix>& left,
			const cv::Mat_<TPix>& right, cv::Mat_<TDisp>* out);
		
	private:
		// OpenCV stereo matching object
		cv::StereoSGBM sgm;
		//ks::StereoSGBM sgm;
	};
}

#endif
