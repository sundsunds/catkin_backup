#ifndef KS_FAKEFAST_H
#define KS_FAKEFAST_H

#include <opencv2/opencv.hpp>

namespace ks {
	// Class providing a slow but verbose implementing the FAST-9 edge detector
	// This class is intended as a playground rather than for productive use
	class FakeFast {
	public:
		void detect(const cv::Mat_<unsigned char>& input, cv::Mat_<unsigned char>* edgeMap, int threshold);
		
	private:
		void makeOffsets(int pixel1[], int pixel2[], int row_stride);
		inline unsigned char evalPixel(const cv::Mat_<unsigned char>& input,
			int offsets1[], int offsets2[], int x, int y, int thresold);
	};
}

#endif
