#ifndef KS_EXTENDEDFAST_H
#define KS_EXTENDEDFAST_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "libks/feature/featuretester.h"
#include "libks/feature/fast9.h"

namespace ks {
	// Extended version of the fast feature detector
	class ExtendedFAST: public cv::FeatureDetector, public FeatureTester {
	public:
		ExtendedFAST(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder = 0);
		virtual ~ExtendedFAST();
			
		// Tests whether a given point is a feature
		virtual bool testFeature(const cv::Mat& image, cv::Point2i pt);
		
		// Performs subpixel refinement for the given image coordinate
		void subpixelRefine(const cv::Mat_<unsigned char>& image, cv::KeyPoint* keyPt, bool horizontal = true, bool vertical = true);
		
	protected:
		// Implements the feature detector interface
		virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<ExtendedFAST*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
	
	private:
		bool nonmaxSuppression;
		// Adaptive threshold parameteres
		unsigned char minThreshold;
		unsigned char maxThreshold;
		float adaptivity;
		bool subpixelPrecision;
		int border;
		
		FAST9<unsigned char> fast9;
		
		// Feature point data
		std::vector<cv::Point2i> corners;
		std::vector<unsigned char> scores;
		
		// Displays a debug window
		/*void showDebugWindow(const cv::Mat_<unsigned char>& image, cv::Point2i center, int arcStart, int arcEnd,
			cv::Point2f subpixArcStart, cv::Point2f subpixArcEnd, cv::Point2f featurePos);*/
			
		// Gets the adapting thresold by local contrast
		__always_inline unsigned char getAdaptiveThreshold(const unsigned char* p);
		// Performs subpixel refinement for one point
		cv::Point2f subpixelRefine(const cv::Mat_<unsigned char>& image, int x, int y, int score, bool horizontal, bool vertical);
		
		// Starts the feature detection
		void fast9Detect(const cv::Mat_<unsigned char>& img, bool calculateScore);
		// Removes features that are closer than the minimum distance
		void removeCloseFeatures(const std::vector<int>& features, std::vector<int>* result);
	};
}

#endif
