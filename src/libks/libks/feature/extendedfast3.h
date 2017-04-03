#ifndef KS_EXTENDEDFAST3_H
#define KS_EXTENDEDFAST3_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "libks/feature/featuretester.h"
#include "libks/feature/ssefast-inl.h"
#include "libks/base/simd.h"

namespace ks {

//#define KS_EXFAST3_CENTRAL_VALUE(img, y, x) img(y, x) 
#define KS_EXFAST3_CENTRAL_VALUE(img, y, x) (((int)img(y, x) + (int)img(y-1, x) + (int)img(y+1, x) + (int)img(y, x-1) + (int)img(y, x+1))/5)

	// Extended version of the fast feature detector
	class ExtendedFAST3: public SSEFast {
	public:
		ExtendedFAST3(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder = 0);
		virtual ~ExtendedFAST3();
			
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
			const_cast<ExtendedFAST3*>(this)->detectImpl(image, keypoints, mask);
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
		std::vector<cv::Point2i> cornersAdapt;
		std::vector<cv::Point2i> cornersMin;
		std::vector<unsigned char> scores;
		
		// Performs subpixel refinement for one point
		//cv::Point2f subpixelRefine(const cv::Mat_<unsigned char>& image, int x, int y, int score, bool horizontal, bool vertical);
		
		// Removes features that are closer than the minimum distance
		void removeCloseFeatures(const std::vector<int>& features, std::vector<int>* result);
		// Applies the adaptive threshold to cornersMin, and calculates the score
		void adaptiveThresholdAndScore(const cv::Mat_<unsigned char>& input);
		
		// Tests for a feature and optionally stores the score
		template <typename T>
		__always_inline bool testFeatureAndScore(const cv::Mat_<unsigned char>& input, const cv::Point_<T> pt, bool storeScore) {
			// Calculate the adaptive threshold by computing an RMS contrast
			// like measure based on absolute values
			v16qi uCircle = loadCircleSSE(input, (int)pt.x, (int)pt.y);
			v16qi circle = uCircle - const128;
			v2di partialSum = __builtin_ia32_psadbw128(uCircle, const0);
			int sum = SIMD::element2(partialSum, 0) + SIMD::element2(partialSum, 1);
			v16qi avg = SIMD::scalar16(sum/16);
			
			v2di partialSAD = __builtin_ia32_psadbw128(uCircle, avg);
			int sad = SIMD::element2(partialSAD, 0) + SIMD::element2(partialSAD, 1);
			unsigned char adaptiveThreshold = cv::saturate_cast<char>(sad * adaptivity / 16);
			v16qi adaptiveThresholdVec = SIMD::scalar16(adaptiveThreshold);
			
			// Perform corner test
			v16qi center = SIMD::scalar16(KS_EXFAST3_CENTRAL_VALUE(input, (int)pt.y, (int)pt.x) - 128);
			if(detectSingleCornerSSE(circle, center, adaptiveThresholdVec, 9)) {
				if(storeScore) // We have to calculate a corner score
					scores.push_back(calcSingleScoreSSE(circle, center, adaptiveThresholdVec, 9) - adaptiveThreshold);
				return true;
			}
			else return false;
		}
	};
}

#endif
