#ifndef KS_HARRIS_H
#define KS_HARRIS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "libks/feature/fast9.h"

namespace ks {
	class Harris: public cv::FeatureDetector {
	public:
		Harris(double threshold, bool nonMaxSuppression = true, int border = 0, int blockSize = 3,
			int apertureSize = 3, double k=0.04 /* or 0.15*/);
			
	protected:
		virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<Harris*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
		
	private:
		double threshold;
		bool nonMaxSuppression;
		int blockSize;
		int apertureSize;
		double k;
		int border;
		
		FAST9<unsigned char> fast9;
		cv::Mat_<float> buffer;
		std::vector<cv::Point2i> corners;
		std::vector<float> scores;
	};
}

#endif
