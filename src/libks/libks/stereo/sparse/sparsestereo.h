#ifndef KS_SPARSESTEREO_H
#define KS_SPARSESTEREO_H

#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "libks/stereo/sparse/sparsematch.h"
#include "libks/stereo/sparse/sparserectification.h"
#include "libks/stereo/sparse/sparsecostcube.h"

namespace ks {
	class StereoRectification;
	
	// Class bundling sparse stereo algorithms	
	template <class CORRELATION, typename COST_TYPE>
	class SparseStereo {
	public:	
		SparseStereo(int maxDisparity, float yTolerance = 1, float uniqueness = 0.6, StereoRectification* rect = NULL,
			bool subpixelFeatures = false, bool storeUnmatched = false, int leftRightStep = 1);
		~SparseStereo();
		
		// Matches using a census window
		void match(const cv::Mat& left, const cv::Mat& right, const std::vector<cv::KeyPoint>& leftFeat,
			const std::vector<cv::KeyPoint>& rightFeat, std::vector<SparseMatch>* matches);

        // Don't actually match but prepare incomplete matches vector from left features only.
        void pseudoMatch(const cv::Mat& left, const std::vector<cv::KeyPoint>& leftFeat, std::vector<SparseMatch>* matches);
		
		// Performs subpixel refinement of matched features
		void subpixelRefine(const cv::Mat& left, const cv::Mat& right, std::vector<SparseMatch>* matches);
			
	private:
		std::vector<SparseRectification::RectifiedFeature> leftFeatures, rightFeatures;	
		int maxDisparity;
		float yTolerance;
		float uniqueness;
		StereoRectification* rect;
		bool storeUnmatched;
		int leftRightStep;
		boost::scoped_ptr<SparseCostCube<COST_TYPE> > costCube;
		std::vector<std::pair<int, COST_TYPE> > minimumMatches;
		cv::Mat_<short int> precompEpilinesStart; // Preocomputed epilines start positions
		unsigned int matchingOps;
		cv::Size frameSize;
		SparseRectification sparseRect;
		
		// Gets the starting offsets of each row and returns the maximum row length
		int getRowOffsets(const std::vector<SparseRectification::RectifiedFeature>& features, unsigned int* offsets,
			int maxRows);
			
		// Calculates the matching costs using census windows
		void calcCosts(const cv::Mat& left, const cv::Mat& right, unsigned int* rowOffsets);
			
		// Performs a left/right consistency check that is dense in the left image
		void denseConsistencyCheck(const cv::Mat& left, const cv::Mat& right);
			
		// Invalidates the given match in the cost cube
		inline void invalidateMatch(int leftFeature);
	};
}

#endif
