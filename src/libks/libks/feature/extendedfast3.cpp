#include <iostream>
#include <iomanip>
#include "libks/feature/extendedfast3.h"
#include "libks/feature/fast9-inl.h"
#include "libks/base/exception.h"

namespace ks {
	using namespace cv;
	using namespace std;
	
	const v16qi ExtendedFAST3::const0(SIMD::scalar16NonLookup(0));
	const v16qi ExtendedFAST3::const128(SIMD::scalar16NonLookup(128));
	
	ExtendedFAST3::ExtendedFAST3(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder)
		: SSEFast(20, 9, true, minBorder /*values are just dummy*/), nonmaxSuppression(nonmaxSuppression),
		minThreshold(minThreshold), adaptivity(adaptivity), subpixelPrecision(subpixelPrecision) {
		
		const int reserve = 512;
		cornersMin.reserve(reserve); cornersAdapt.reserve(reserve); scores.reserve(reserve);
	}
	
	ExtendedFAST3::~ExtendedFAST3() {
	}

	void ExtendedFAST3::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		
		if(mask.data != NULL)
			throw Exception("Feature detection masks not supported!");
		else if(image.type() != CV_8U)
			throw Exception("Image data has to be of type unsigned char!");
		
		// Create offsets for circle pixel
		initOffsets(image.step);
		
		//FAST(image, cornersMin, minThreshold, false);
		detectCorners(image, minThreshold, &cornersMin);
		adaptiveThresholdAndScore(image);	
	 
		if(nonmaxSuppression)
		{
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(cornersAdapt, scores, nonmaxPoints);
			
			// Copy and optionally refine result
			keypoints.reserve(nonmaxPoints.size());
			for(unsigned int i=0; i<nonmaxPoints.size(); i++) {
				int index = nonmaxPoints[i];
				Point2f pt = /*subpixelPrecision ? subpixelRefine(image, corners[index].x, corners[index].y, scores[index], true, true) :*/
					Point2f(cornersAdapt[index].x, cornersAdapt[index].y);
				keypoints.push_back(KeyPoint(pt, 6.f, -1.f, scores[index]));
			}
		}
		else
		{   
			// Copy everything
			size_t i, n = cornersAdapt.size();
			keypoints.resize(n);
			for( i = 0; i < n; i++ )
				keypoints[i] = KeyPoint(cornersAdapt[i], 6.f, -1.f, -1000);
		}
		
		// Clear buffers
		cornersMin.clear(); cornersAdapt.clear(); scores.clear();
	}
	
	void ExtendedFAST3::adaptiveThresholdAndScore(const Mat_<unsigned char>& input) {
		initOffsets((int)input.step);
		for(unsigned int i=0; i<cornersMin.size(); i++) {
			if(testFeatureAndScore(input, cornersMin[i], nonmaxSuppression))
				cornersAdapt.push_back(Point2i(cornersMin[i].x, cornersMin[i].y));
		}
	}
}
