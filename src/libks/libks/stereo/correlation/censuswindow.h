#ifndef KS_CENSUSWINDOW_H
#define KS_CENSUSWINDOW_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "libks/base/hammingdistance.h"
#include "libks/base/simd.h"

//#define KS_NO_POPCNT //Defined by CMake

#if defined(KS_NO_POPCNT)
#define KS_SSE_OPTIMIZE
#endif

namespace ks {
	// Matches over a set of census windows
	template <int SIZE>
	class CensusWindow {
	public:
		CensusWindow() {
#ifdef KS_SSE_OPTIMIZE
			lookupTablePtr = hammingDist.getLookupTable();
			lookupTableVec = SIMD::scalar4((int)hammingDist.getLookupTable());
			zero = SIMD::scalar16(0);
#endif
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
		void setReferencePoint(const cv::Point2i& point);
		
		// Performs a window matching using census transformed images
		__always_inline short match(cv::Point2i point) const;
		
	private:
		v4si lookupTableVec;
		//v4si refWindow[7];
		v4si refWindow[6];
		v16qi zero;
	
		const unsigned char* lookupTablePtr;
		HammingDistance hammingDist;
		cv::Mat_<unsigned int> refImage;
		cv::Mat_<unsigned int> compImage;
		cv::Point2i refPoint;
		
		// Stores a window in two SSE vectors
		void loadWindow(const cv::Mat_<unsigned int>& image, const cv::Point2i& point, v4si* dstWindow) const;
	} __attribute__ ((aligned (16)));
}

#endif
