#ifndef KS_EXTENDEDFAST2_H
#define KS_EXTENDEDFAST2_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "libks/feature/featuretester.h"
#include "libks/feature/ssefast-inl.h"
#include "libks/base/simd.h"

namespace ks {

//#define KS_EXFAST_CENTRAL_VALUE(img, y, x) img(y, x) 
#define KS_EXFAST_CENTRAL_VALUE(img, y, x) ((5*128+ (int)img(y, x) + (int)img(y-1, x) + (int)img(y+1, x) + (int)img(y, x-1) + (int)img(y, x+1))/5 - 128)

	// Extended version of the fast feature detector
	class ExtendedFAST2: public SSEFast {
	public:
		ExtendedFAST2(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder = 0);
		virtual ~ExtendedFAST2();
			
		// Performs subpixel refinement for the given image coordinate
		//void subpixelRefine(const cv::Mat_<unsigned char>& image, cv::KeyPoint* keyPt, bool horizontal = true, bool vertical = true);
		
		virtual __always_inline bool testFeature(const cv::Mat& image, cv::Point2i pt) {
			if(pt.y<3 || pt.y>= image.rows - 3 || pt.x<3 || pt.x>= image.cols-3)
				return false;
		
			initOffsets((int)image.step);
			return testFeatureAndScore(image, pt, false);
		}
		
	protected:
		// Implements the feature detector interface
		virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const {
			// OpenCV tries to force this method to be const, but we don't like that!
			const_cast<ExtendedFAST2*>(this)->detectImpl(image, keypoints, mask);
		}
		void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());
	
	private:
		// Constants for SSE processing
		static const v16qi const0, const128;
	
		bool nonmaxSuppression;
		// Adaptive threshold parameteres
		unsigned char minThreshold;
		unsigned char maxThreshold;
		float adaptivity;
		bool subpixelPrecision;
		int border;
		
		// Feature point data
		std::vector<cv::Point2i> cornersMin, cornersAdapt;
		std::vector<unsigned char> scores;
		
		// Performs subpixel refinement for one point
		//cv::Point2f subpixelRefine(const cv::Mat_<unsigned char>& image, int x, int y, int score, bool horizontal, bool vertical);
		
		// Removes features that are closer than the minimum distance
		void removeCloseFeatures(const std::vector<int>& features, std::vector<int>* result);
		// Applies the adaptive threshold to cornersMin, and calculates the score
		void adaptiveThresholdAndScore(const cv::Mat_<char>& input);
		
		// Tests for a feature and optionally stores the score
		__always_inline bool testFeatureAndScore(const cv::Mat_<char>& input, const cv::Point2i pt, bool storeScore) {
			// Calculate the adaptive threshold by computing an RMS contrast
			// like measure based on absolute values
			v16qi circle = loadCircleSSE(input, pt.x, pt.y);
			v16qi uCircle = circle + const128;
			v2di partialSum = __builtin_ia32_psadbw128(uCircle, const0);
			int sum = SIMD::element2(partialSum, 0) + SIMD::element2(partialSum, 1);
			v16qi avg = SIMD::scalar16(sum/16);
			
			v2di partialSAD = __builtin_ia32_psadbw128(uCircle, avg);
			int sad = SIMD::element2(partialSAD, 0) + SIMD::element2(partialSAD, 1);
			unsigned char adaptiveThreshold = cv::saturate_cast<char>(sad * adaptivity / 16);
			v16qi adaptiveThresholdVec = SIMD::scalar16(adaptiveThreshold);
			
			// Perform corner test
			v16qi center = SIMD::scalar16(KS_EXFAST_CENTRAL_VALUE(input, pt.y, pt.x));
			if(detectSingleCornerSSE(circle, center, adaptiveThresholdVec, 9)) {
				if(storeScore) // We have to calculate a corner score
					scores.push_back(calcSingleScoreSSE(circle, center, adaptiveThresholdVec, 9) - adaptiveThreshold);
				return true;
			}
			else return false;
		}
		
		// Non-SSE alternative
		/*__always_inline bool testFeatureAndScore(const cv::Mat_<char>& input, const cv::Point2i pt, bool storeScore) {
			if(pt.y<3 || pt.y>= input.rows - 3 || pt.x<3 || pt.x>= input.cols-3)
				return false;
			fast9.setStep((int)input.step);
		
			int sum = 0;
			const char* p = &input(pt.y, pt.x);
			for(int i=0; i<16; i++)
				sum += p[fast9.pixel[i]] +  128;
			int avg = sum/16;
			
			int sad = 0;
			for(int i=0; i<16; i++)
				sad += std::abs(p[fast9.pixel[i]] + 128 - avg);
			
			unsigned char adaptiveThreshold = cv::saturate_cast<char>(sad * adaptivity / 16);
			
			if(fast9.cornerTest(p, KS_EXFAST_CENTRAL_VALUE(input, pt.y, pt.x), adaptiveThreshold)) {
				if(storeScore) // We have to calculate a corner score
					BROKEN!!!
					scores.push_back(fast9.cornerScore(p, *p, adaptiveThreshold) - adaptiveThreshold);
				return true;
			}
			else return false;
		}*/
	};
}

#endif
