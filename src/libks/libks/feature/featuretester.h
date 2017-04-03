#ifndef KS_FEATURETESTER_H
#define KS_FEATURETESTER_H

#include <opencv2/opencv.hpp>

namespace ks {
	// Interface for performing feature tests
	class FeatureTester {
	public:
		virtual bool testFeature(const cv::Mat& image, cv::Point2i pt) = 0;
	};
}

#endif
