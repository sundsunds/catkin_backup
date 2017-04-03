#ifndef KS_NOISEROBUSTCENSUS_H
#define KS_NOISEROBUSTCENSUS_H

#include "libks/imageproc/census.h"

namespace ks {
	// Census transform using the average or median
	// of the census window for comparisons. This
	// increases the robustness towards noise.
	class NoiseRobustCensus {
	public:
		// Census transform using 9x3 window and average intensity
		static void average9x3(cv::Mat_<unsigned char>* input, cv::Mat_<unsigned int>* output, bool preserveInput = false);
		// Census transform using 9x3 window and median intensity
		static void median9x3(cv::Mat_<unsigned char>* input, cv::Mat_<unsigned int>* output, bool preserveInput = false);
		
		// Census transform using 5x5 window and average intensity
		static void average5x5(cv::Mat_<unsigned char>* input, cv::Mat_<unsigned int>* output, bool preserveInput = false);
		// Census transform using 5x5 window and average intensity
		static void median5x5(cv::Mat_<unsigned char>* input, cv::Mat_<unsigned int>* output, bool preserveInput = false);
	
	private:
		// Calculates the average image used for the average census transform
		template <int W, int H>
		static void calcAvgImage(const cv::Mat_<unsigned char>& input, cv::Mat_<unsigned char>* output);
		
		// Calculates the mediane image used for the median census transform
		template <int W, int H>
		static void calcMedianImage(const cv::Mat_<unsigned char>& input, cv::Mat_<unsigned char>* output);
	};
}

#endif
