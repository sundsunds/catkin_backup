#include "libks/feature/evaluation/repeatabilityevaluation.h"

namespace ks {
	using namespace std;
	using namespace cv;

	double RepeatabilityEvaluation::evaluate(const vector<Point2f>& projectedPoints1, const vector<Point2f>& projectedPoints2) {
		unsigned int matched = 0;
		for(unsigned int i=0; i<projectedPoints1.size(); i++)
			if(findCorrespondingPoint(projectedPoints1[i], projectedPoints2))
				matched++;
		
		return matched / double(projectedPoints1.size());
	}
	
	bool RepeatabilityEvaluation::findCorrespondingPoint(Point2f point, const std::vector<cv::Point2f>& points) {
		for(unsigned int i=0; i<points.size(); i++) {
			double dSquare = (points[i].x - point.x) * (points[i].x - point.x) + (points[i].y - point.y) * (points[i].y - point.y); 
			if(dSquare <= epsilon * epsilon)
				return true;
		}
		
		return false;
	}
}
