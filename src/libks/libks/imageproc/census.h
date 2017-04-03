#ifndef KS_CENSUS_H
#define KS_CENSUS_H

#include <opencv2/opencv.hpp>
#include "libks/base/simd.h"

namespace ks {
	// Computes various variants of the census transform
	class Census {
	public:
		// Census transform using 9x3 window
		template <typename T>
		static void transform9x3(const cv::Mat_<T>& input, cv::Mat_<unsigned int>* output);
		
		// Census transform using 5x5 window
		template <typename T>
		static void transform5x5(const cv::Mat_<T>& input, cv::Mat_<unsigned int>* output);
		
		// Census transform using 8x8 window
		template <typename T>
		static void transform8x8(const cv::Mat_<T>& input, cv::Mat_<unsigned long long>* output);

		// Census transform using 9x7 window
		template <typename T>
		static void transform9x7(const cv::Mat_<T>& input, cv::Mat_<unsigned long long>* output);
		
	private:
		// SSE optimized implementations
		static void transform9x3SSE(const cv::Mat_<char>& input, cv::Mat_<unsigned int>* output);
		static void transform5x5SSE(const cv::Mat_<char>& input, cv::Mat_<unsigned int>* output);
		
		// Efficiently stores 4 byte blocks by interleaving four 1-byte vectors
		static __always_inline void storeSSEVec(const v16qi& byte4, const v16qi& byte3, const v16qi& byte2,
			const v16qi& byte1, char* dst);
	};
}

#endif
