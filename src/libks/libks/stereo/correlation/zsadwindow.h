#ifndef KS_ZSADWINDOW_H
#define KS_ZSADWINDOW_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>

namespace ks {
	// Matches over a set of Zero Mean Sum of Absolute Differences Windows
	template <int SIZE>
	class ZSADWindow{
	public:
		ZSADWindow() {
		}
		
		void setReferenceImage(const cv::Mat_<unsigned char>& image) {
			refImage = image;
		}
		
		void setComparisonImage(const cv::Mat_<unsigned char>& image) {
			compImage = image;
		}
		
		const cv::Mat_<unsigned char>& getReferenceImage() const {return refImage;}
		const cv::Mat_<unsigned char>& getComparisonImage() const {return compImage;}
		const int getWindowSize() const {return SIZE;}
		
		// Sets the position of the reference window
		void setReferencePoint(const cv::Point2i& point);
		
		// Performs a window matching using census transformed images
		short match(cv::Point2i point) const;
		
	private:
		cv::Mat_<unsigned char> refImage;
		cv::Mat_<unsigned char> compImage;
		cv::Point2i refPoint;
	};
	
	template <int SIZE>
	void ZSADWindow<SIZE>::setReferencePoint(const cv::Point2i& point) {
		refPoint = point;
	}
	
	template <int SIZE>
	short ZSADWindow<SIZE>::match(cv::Point2i point) const {
		// Sum over window
		int meanRef=0, meanComp=0;
		for(int y=-SIZE/2; y<=SIZE/2; y++)
			for(int x=-SIZE/2; x<=SIZE/2; x++) {
				meanRef += refImage(refPoint.y + y, refPoint.x + x);
				meanComp += compImage(point.y + y, point.x + x);
			}
		
		static const int realSize = (SIZE/2)*2+1; // Avoid rounding errors
		meanRef /= realSize*realSize;
		meanComp /= realSize*realSize;
		int cost = 0;
		
		for(int y=-SIZE/2; y<=SIZE/2; y++)
			for(int x=-SIZE/2; x<=SIZE/2; x++)
				cost += std::abs(refImage(refPoint.y + y, refPoint.x + x) - meanRef
					- compImage(point.y + y, point.x + x) + meanComp);
		
		return (short)cost;
	}
}

#endif
