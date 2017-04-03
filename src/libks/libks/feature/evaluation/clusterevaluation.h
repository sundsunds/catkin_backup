#ifndef KS_CLUSTEREVALUATION_H
#define KS_CLUSTEREVALUATION_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace ks {
	class ClusterEvaluation {
	public:
		// This class provides a measure on how clustered a given set of feature points are
		double evaluate(const std::vector<cv::KeyPoint>& points, cv::Rect_<float> roi);
		double evaluate(const std::vector<cv::Point2f>& points, cv::Rect_<float> roi);
	
	private:
		//double findNearestNeighborDistances(const std::vector<cv::Point2f>& points, unsigned int ref);
		//unsigned int countCloseNeighbors(const std::vector<cv::Point2f>& points, unsigned int ref);
		unsigned int countPointsInRect(const std::vector<cv::Point2f>& points, cv::Rect_<float> roi);
	};
}

#endif
