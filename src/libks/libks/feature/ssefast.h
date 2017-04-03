#ifndef KS_SSEFAST_H
#define KS_SSEFAST_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "libks/base/simd.h"
#include "libks/feature/featuretester.h"
#include "libks/feature/fast9-inl.h"

namespace ks {
	// FAST-Like feature detector not using a search tree, but SSE instructions
	class SSEFast: public cv::FeatureDetector, public FeatureTester {
	public:
		SSEFast(unsigned char threshold = 20, unsigned char minLength = 9, bool nonmaxSuppression = true, int minBorder = 0);
		virtual ~SSEFast();
		
		// Performs feature detection with explicitely specified parameters.
		// Alternative to cv::FeatureDetector::detect()
		void detect(const cv::Mat_<char>& input, unsigned char threshold,
			std::vector<cv::KeyPoint>* results, bool nonmaxSuppression);
			
		// Tests whether a given point is a feature.
		virtual bool testFeature(const cv::Mat& image, cv::Point2i pt){
			return fast9.cornerTest(&image.at<char>(pt.y, pt.x), image.at<char>(pt.y, pt.x), threshold);
		}
					
	protected:
		// Provides non-maximum suppression functionality
		FAST9<char> fast9;
	
		// Implements the feature detector interface
		virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<SSEFast*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
	
		// Initializes offsets for corner detection
		void initOffsets(int step);
		// Loads a single circle to an SSE register
		template <typename T>
		__always_inline v16qi loadCircleSSE(const cv::Mat_<T>& input, int x, int y);
		// Calculates the score for a single found feature
		__always_inline unsigned char calcSingleScoreSSE(const v16qi& circle, v16qi center, const v16qi& bstartVec, unsigned char minLength);
		// Performs corner detection and stores result as a list of cv::Point2i
		void detectCorners(const cv::Mat_<char>& input, unsigned char threshold, std::vector<cv::Point2i>* results);
		// Detects a single corner with SSE
		__always_inline bool detectSingleCornerSSE(const v16qi& circle, const v16qi& center, const v16qi& threshold, unsigned char minLength);
		// Calculates the scores for a set of detected corners
		void calcScores(const cv::Mat_<char>& input, const std::vector<cv::Point2i>& corners, int bstart, unsigned char minLength,
			std::vector<unsigned char>* scores);
	
	private:
		// Lookup table for longest arc lengths
		static unsigned char lookupTable[1U<<16];
		static bool lookupInitialized;
		
		// Constants used for SSE processing
		static const v16qi const0, const128, const255, const0x01, const0x02,
			const0x04, const0x08, const0x10, const0x20, const0x40, const0x80;
		
		// Default algorithm parameters
		unsigned char threshold;
		unsigned char minLength;
		bool nonmaxSuppression;
		int border;
		
		std::vector<cv::Point2i> corners;
		std::vector<unsigned char> scores;
		
		// Offsets for the circle pixel
		int offsets[16];
		
		inline unsigned char findLongestArc(unsigned short stripe);
	};
}

#endif
