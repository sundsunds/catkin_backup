#ifndef KS_PYRAMIDRANGEDETECTOR_H
#define KS_PYRAMIDRANGEDETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include "libks/feature/featuretester.h"

namespace ks {
	// Performs an ordinary feature detection and then determines for each
	// feature, up to which pyramid level it's visible. It is required that
	// the detected features are sorted by image coordinates!
	class PyramidRangeDetector:  public cv::FeatureDetector {
	public:
		// Creates an object based on a feature detecter
		PyramidRangeDetector(boost::shared_ptr<cv::FeatureDetector> detector, int maxLevel, bool storePyramid = false)
			: detector(detector), maxLevel(maxLevel), unsignedImg(NULL), storePyramid(storePyramid) {}
			
		// Creates an object based on a feature tester. This approach is faster and
		// more reliable!
		PyramidRangeDetector(boost::shared_ptr<cv::FeatureDetector> detector, boost::shared_ptr<FeatureTester> tester, int maxLevel, bool storePyramid = false)
			: detector(detector), tester(tester), maxLevel(maxLevel), storePyramid(storePyramid) {}
			
		// If signed char images are used, this function should be called to set a
		// unsigned char alternative version.
		void setUnsignedImage(const cv::Mat_<unsigned char>* uImg) {unsignedImg = uImg;}
		
		// Returns the stored pyramid level. Level 0 is not stored!
		cv::Mat getPyramidLevel(int level) {return pyramid[level-1];}
			
	protected:
		// Implements the feature detector interface
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<PyramidRangeDetector*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
	
	private:
		boost::shared_ptr<cv::FeatureDetector> detector;
		boost::shared_ptr<FeatureTester> tester;
		int maxLevel;
		const cv::Mat_<unsigned char>* unsignedImg;
		bool storePyramid;
		std::vector<cv::Mat> pyramid;
		
		void implDetector(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
		void implTester(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
			
		// Searches points that appear in both levels and updates the point octaves
		void findLevels(std::vector<cv::KeyPoint>& prevPoints, const std::vector<cv::KeyPoint>& newPoints) const;
		// Optimized feature detection that only tests for features at the feature locations from the
		// previous pyramid level
		void fastDetection(const cv::Mat& image, const std::vector<cv::KeyPoint>& prevPoints,
			std::vector<cv::KeyPoint>& newPoints) const;
	};
}

#endif
