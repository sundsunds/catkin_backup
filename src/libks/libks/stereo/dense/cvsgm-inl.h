#ifndef KS_CVSGM_INL_H
#define KS_CVBSGM_INL_H

#include "libks/stereo/dense/cvsgm.h"

namespace ks {
	using namespace cv;
	using namespace std;
	
	template<typename TDisp, typename TPix>
	CvSGM<TDisp, TPix>::CvSGM(int dispMin, int dispMax, int winSize)
		:sgm(dispMin, dispMax - dispMin, winSize)
	{}
	
	template<typename TDisp, typename TPix>	
	void CvSGM<TDisp, TPix>::match(const Mat_<TPix>& left,
		const Mat_<TPix>& right, cv::Mat_<TDisp>* out) {
		sgm(left, right, *out);
	}
}

#endif
