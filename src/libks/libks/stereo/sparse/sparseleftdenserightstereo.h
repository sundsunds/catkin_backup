#ifndef KS_SPARSELEFTDENSERIGHTSTEREO_H
#define KS_SPARSELEFTDENSERIGHTSTEREO_H

#include <vector>
#include <opencv2/opencv.hpp>

#include "libks/stereo/sparse/sparsematch.h"
#include "libks/stereo/sparse/sparserectification.h"

namespace ks {
	
	// Stereo algirthm that performs a sparse left and dense right matching
	template <class CORRELATION, typename COST_TYPE>
	class SparseLeftDenseRightStereo {
	public:	
		SparseLeftDenseRightStereo(int maxDisparity, float uniqueness=1);
		~SparseLeftDenseRightStereo();
		
		void match(const cv::Mat& left, const cv::Mat& right,
			const std::vector<cv::KeyPoint>& leftFeatures, std::vector<SparseMatch>* matches);
			
	private:
		int maxDisparity;
		float uniqueness;
		unsigned int matchingOps;
	};
}

#endif