#ifndef KS_WINNERTAKESALL_H
#define KS_WINNERTAKESALL_H

#include "libks/stereo/dense/densestereoalgorithm.h"

namespace ks {
	template<class CORRELATION, typename PIXEL_TYPE>
	class WinnerTakesAll: public DenseStereoAlgorithm<unsigned char, PIXEL_TYPE> {
	public:
		WinnerTakesAll(int maxDisp, bool consistencyCheck, float uniqueness = 1.0, CORRELATION corr = CORRELATION());
		virtual ~WinnerTakesAll();
		
		// Performs the stereo matching
		virtual void match(const cv::Mat_<PIXEL_TYPE>& left,
			const cv::Mat_<PIXEL_TYPE>& right, cv::Mat_<unsigned char>* out);
	
	private:
		int maxDisp;
		int border;
		bool consistencyCheck;
		float uniqueness;
		CORRELATION correlation;
		unsigned int matchOps;
		
		void matchWithoutConsistencyCheck(cv::Mat_<unsigned char>* out, int rows, int cols);
		void matchWithConsistencyCheck(cv::Mat_<unsigned char>* out, int rows, int cols);
	};
};

#endif
