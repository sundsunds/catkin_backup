#ifndef KS_REPEATABILITYEVALUATION_H
#define KS_REPEATABILITYEVALUATION_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace ks {
	// Evaluates the repeatability usint the measure used by Gauglitz
	class RepeatabilityEvaluation {
	public:
		RepeatabilityEvaluation(double epsilon = 2.0)
			:epsilon(epsilon) {}
			
		double evaluate(const std::vector<cv::Point2f>& projectedPoints1, const std::vector<cv::Point2f>& projectedPoints2);
					
	private:
		double epsilon;
		
		bool findCorrespondingPoint(cv::Point2f point, const std::vector<cv::Point2f>& points);
	};
}

#endif
