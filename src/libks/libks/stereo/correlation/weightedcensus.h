#ifndef KS_WEIGHTEDCENSUS_H
#define KS_WEIGHTEDCENSUS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "libks/base/hammingdistance.h"


namespace ks {
	// Matches over a set of census windows
	template <int SIZE>
	class WeightedCensus {
	public:
		WeightedCensus(): factor(1) {
		}
		
		WeightedCensus(double factor, const cv::Mat_<unsigned char>& weightRef, const cv::Mat_<unsigned char>& weightComp)
			:factor(factor), weightRef(weightRef), weightComp(weightComp) {
		}
		
		void setReferenceImage(const cv::Mat_<unsigned int>& image) {
			refImage = image;
		}
		
		void setComparisonImage(const cv::Mat_<unsigned int>& image) {
			compImage = image;
		}
		
		const cv::Mat_<unsigned int>& getReferenceImage() const {return refImage;}
		const cv::Mat_<unsigned int>& getComparisonImage() const {return compImage;}
		const int getWindowSize() const {return SIZE;}
		
		// Sets the position of the reference window
		void setReferencePoint(const cv::Point2i& point) {
			refPoint = point;
		}
		
		// Performs a window matching using census transformed images
		__always_inline short match(cv::Point2i point) const {
			double costs = 0.0;
			
			for(int y=-SIZE/2; y<=SIZE/2; y++)
				for(int x=-SIZE/2; x<=SIZE/2; x++)
					costs += (1.0 + factor * (weightRef(refPoint.y + y, refPoint.x + x) + weightComp(point.y + y, point.x +x))) *
						hammingDist.calculate(refImage(refPoint.y + y, refPoint.x + x),	compImage(point.y + y, point.x + x));
			
			return costs;
		}
		
	private:
		HammingDistance hammingDist;
		cv::Mat_<unsigned int> refImage;
		cv::Mat_<unsigned int> compImage;
		cv::Point2i refPoint;
		double factor;
		cv::Mat_<unsigned char> weightRef, weightComp;
	};
}

#endif
