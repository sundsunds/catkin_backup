#include <iostream>
#include <iomanip>
#include "libks/feature/extendedfast2.h"
#include "libks/feature/fast9-inl.h"
#include "libks/base/exception.h"

//#define ENFORCE_MIN_DISTANCE


namespace ks {
	using namespace cv;
	using namespace std;
	
	const v16qi ExtendedFAST2::const0(SIMD::scalar16NonLookup(0));
	const v16qi ExtendedFAST2::const128(SIMD::scalar16NonLookup(128));
	
	ExtendedFAST2::ExtendedFAST2(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder)
		: SSEFast(20, 9, true, minBorder /*values are just dummy*/), nonmaxSuppression(nonmaxSuppression),
		minThreshold(minThreshold), adaptivity(adaptivity), subpixelPrecision(subpixelPrecision) {
		
		const int reserve = 512;
		cornersMin.reserve(reserve); cornersAdapt.reserve(reserve); scores.reserve(reserve);
	}
	
	ExtendedFAST2::~ExtendedFAST2() {
	}

	void ExtendedFAST2::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		
		if(mask.data != NULL)
			throw Exception("Feature detection masks not supported!");
		else if(image.type() != CV_8S)
			throw Exception("Image data has to be of type char!");
		
		// Create offsets for circle pixel
		initOffsets(image.step);
		
		detectCorners(image, minThreshold, &cornersMin);
		adaptiveThresholdAndScore(image);
	 
		if(nonmaxSuppression)
		{
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(cornersAdapt, scores, nonmaxPoints);
			
#ifdef ENFORCE_MIN_DISTANCE
			vector<int> nonClosePoints;
			removeCloseFeatures(nonmaxPoints, &nonClosePoints);
			vector<int>& resultPoints = nonClosePoints;
#else
			vector<int>& resultPoints = nonmaxPoints;
#endif
			
			// Copy and optionally refine result
			keypoints.reserve(resultPoints.size());
			for(unsigned int i=0; i<resultPoints.size(); i++) {
				int index = resultPoints[i];
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
	
	void ExtendedFAST2::adaptiveThresholdAndScore(const Mat_<char>& input) {
		initOffsets((int)input.step);
		for(unsigned int i=0; i<cornersMin.size(); i++) {
			if(testFeatureAndScore(input, cornersMin[i], nonmaxSuppression))
				cornersAdapt.push_back(cornersMin[i]);
		}
	}
	
	/*void ExtendedFAST2::subpixelRefine(const Mat_<unsigned char>& image, KeyPoint* keyPt, bool horizontal, bool vertical) {
		fast9.setStep((int)image.step);
		
		int x = int(keyPt->pt.x+0.5), y = int(keyPt->pt.y+0.5);
		if(y<4 || y>= image.rows - 4 || x<4 || x>= image.cols-4)
			return; // Too close to the border for refinement
		else keyPt->pt = subpixelRefine(image, x, y, (int)keyPt->response, horizontal, vertical);
	}
	
	Point2f ExtendedFAST2::subpixelRefine(const Mat_<unsigned char>& image, int x, int y, int score, bool horizontal, bool vertical) {
		int c2 = score;
		int centralInt = CENTRAL_VALUE(image, y, x);
		int threshold = getAdaptiveThreshold(&image(y, x), centralInt);
		
		if(c2 < -900)
			c2 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
	
		// Horizontal refinement
		float xOffset = 0;
		if(horizontal) {
			int centralInt = CENTRAL_VALUE(image, y, x-1);
			int c1 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			centralInt = CENTRAL_VALUE(image, y, x+1);
			int c3 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			xOffset = SubpixelInterpolation::maxOffset(c1, c2, c3, SubpixelInterpolation::QUADRATIC);
		}
		
		// Vertical refinement
		float yOffset = 0;
		if(vertical) {
			int centralInt = CENTRAL_VALUE(image, y-1, x);
			int c1 = fast9.cornerScore(&image(y-1, x), centralInt,  0) - threshold;
			centralInt = CENTRAL_VALUE(image, y, x);
			int c3 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			yOffset = SubpixelInterpolation::maxOffset(c1, c2, c3, SubpixelInterpolation::QUADRATIC);
		}
		
		Point2f ret(x + xOffset, y + yOffset);
		
		return ret;
	}*/

	void ExtendedFAST2::removeCloseFeatures(const std::vector<int>& features, std::vector<int>* result) {
		static const int minDist = adaptivity;
		result->clear();
		
		// TODO optimize if this method shows to be of any use
		for(unsigned int i=0; i<features.size(); i++) {
			bool skipPoint = false;
			for(unsigned int j=0; j<features.size(); j++) {
				if(i==j)
					continue;
					
				Point2i& p1 = cornersAdapt[features[i]];
				Point2i& p2 = cornersAdapt[features[j]];
				int dx = p1.x - p2.x;
				int dy = p1.y - p2.y;
				
				if(dx*dx + dy*dy < minDist*minDist && scores[features[i]] < scores[features[j]]) {
					skipPoint = true;
					break;
				}
			}
			
			if(!skipPoint)
				result->push_back(features[i]);
		}
	}
}
