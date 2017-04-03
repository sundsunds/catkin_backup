#ifndef KS_DENSESTEREOALGORITHM_H
#define KS_DENSESTEREOALGORITHM_H

#include <opencv2/opencv.hpp>

namespace ks {

	// Abstract base class for all stereo algorithms
	template<typename TDisp, typename TPix>
	class DenseStereoAlgorithm {
	public:	
		// Constant for invalid disparities, which is
		// either -1 or the maximum value for TDisp
		enum {
			INVALID_DISP = int((TDisp)-1)
		};
				
		virtual ~DenseStereoAlgorithm() {}		
		// Performs the stereo matching
		virtual void match(const cv::Mat_<TPix>& left,
			const cv::Mat_<TPix>& right, cv::Mat_<TDisp>* out ) = 0;
	};

	// Some predefined algorithm configurations
	typedef DenseStereoAlgorithm<unsigned char, unsigned char> DenseStereoAlg8U;
	typedef DenseStereoAlgorithm<float, unsigned char>         denseStereoAlgSubpix8U;
}

#endif
