#ifndef KS_SPARSELEFTDENSERIGHTSTEREO_INL_H
#define KS_SPARSELEFTDENSERIGHTSTERE_INLO_H

#include "libks/stereo/sparse/sparseleftdenserightstereo.h"
#include "libks/stereo/sparse/sparseconstraints.h"

//#define COUNT_MATCHING_OPS

namespace ks {
	template <class CORRELATION, typename COST_TYPE>
	SparseLeftDenseRightStereo<CORRELATION, COST_TYPE>::SparseLeftDenseRightStereo(int maxDisparity, float uniqueness)
		:maxDisparity(maxDisparity), uniqueness(uniqueness), matchingOps(0) {
	}
	
	template <class CORRELATION, typename COST_TYPE>
	SparseLeftDenseRightStereo<CORRELATION, COST_TYPE>::~SparseLeftDenseRightStereo() {
#ifdef COUNT_MATCHING_OPS
		std::cout << "Matching operations: " << matchingOps << std::endl;
#endif

	}
	
	template <class CORRELATION, typename COST_TYPE>
	void SparseLeftDenseRightStereo<CORRELATION, COST_TYPE>::match(const cv::Mat& left, const cv::Mat& right,
		const std::vector<cv::KeyPoint>& leftFeatures, std::vector<SparseMatch>* matches) {
		
		SparseCostCube<COST_TYPE> costCube(left.rows, left.cols, leftFeatures.size());
		CORRELATION correlation;
		correlation.setReferenceImage(left);
		correlation.setComparisonImage(right);		
		
		// Old code. Something is wrong with it!
		int lastRow = -1;
		for(unsigned int feature = 0; feature < leftFeatures.size(); feature++) {
			if(int(leftFeatures[feature].pt.y) != lastRow) {		
				// Skip top and bottom
				if(int(leftFeatures[feature].pt.y) < correlation.getWindowSize()/2)
					continue;
				else if(int(leftFeatures[feature].pt.y) >= left.rows - correlation.getWindowSize()/2)
					break;
					
				lastRow = int(leftFeatures[feature].pt.y);
				// Initialize a new row in the cost cube
				costCube.advanceRow(lastRow);
			}
			else costCube.advanceLeftFeature();
			
			correlation.setReferencePoint(cv::Point2i((int)leftFeatures[feature].pt.x, (int)leftFeatures[feature].pt.y));
			
			int xStart = max(correlation.getWindowSize()/2, int(leftFeatures[feature].pt.x) - maxDisparity);
			costCube.advanceRightFeature(xStart);
			for(int x = xStart; x<=int(leftFeatures[feature].pt.x); x++) {
#ifdef COUNT_MATCHING_OPS
				matchingOps++;
#endif						
				COST_TYPE currentCost = correlation.match(cv::Point2i(x, lastRow));
				costCube.pushCost(currentCost);
			}
		}
		costCube.finalizeCube();
		
		//SparseConstraints::leftRightConsistency(leftFeatures, &costCube);
		//SparseConstraints::uniquenessConstraint(&costCube, 0.5);
		SparseConstraints::combindedConsistencyUniqueness(&costCube, uniqueness);
		
		// Compose sparse disparity list
		for(unsigned int i=0; i<leftFeatures.size(); i++)
			if(costCube.getRightMinimumCostIndex(i) != -1) {
				int disparity = leftFeatures[i].pt.x - costCube.getRightMinimumCostIndex(i);
				matches->push_back(SparseMatch(&leftFeatures[i], NULL, leftFeatures[i].pt,
					cv::Point2f(leftFeatures[i].pt.x-disparity, leftFeatures[i].pt.y)));
			}
	}
}

#endif
