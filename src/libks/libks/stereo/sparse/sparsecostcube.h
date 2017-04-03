#ifndef KS_SPARSECOSTCUBE_H
#define KS_SPARSECOSTCUBE_H

#include <limits>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cassert>
#include "libks/base/exception.h"

namespace ks {
	template <typename T>	
	// Stores a sparsce cost cube for sparse stereo matching
	class SparseCostCube {
	public:
		SparseCostCube(int rows, int maxRightFeaturesInRow, int leftFeaturesCount)
			: costs(leftFeaturesCount, maxRightFeaturesInRow, std::numeric_limits<T>::max()),
				featureRows(leftFeaturesCount + 1, -1), costRowOffsets(rows + 1, -1),
				minimumMatches(leftFeaturesCount, -1), rightFeaturesTable(),
				rightFeaturesTableRowOffsets(rows+1, -1), rightFeatureIndex(-1),
				leftFeatureIndex(-1), currentRow(-1), rightMinimumCost(0) {
		}
		
		// Adds a new cost for the current left feature and image row.
		// The right feature index is automatically incremented
		void pushCost(T cost) {
			assert(rightFeatureIndex < (int)costs.cols);
			assert(leftFeatureIndex < (int)costs.rows);
			
			costs(leftFeatureIndex, rightFeatureIndex) = cost;
			if(cost < rightMinimumCost) {
				rightMinimumCost = cost;
				minimumMatches[leftFeatureIndex] = rightFeatureIndex;
			}
			
			rightFeatureIndex++;
		}
		
		// Advances the right feature index to the given value
		void advanceRightFeature(int nextRightFeature) {
			rightFeatureIndex = nextRightFeature;
		}
		
		// Advances the left feature index by one
		void advanceLeftFeature() {
			leftFeatureIndex++;
			rightFeatureIndex = 0;
			rightMinimumCost = std::numeric_limits<T>::max();
			featureRows[leftFeatureIndex] = currentRow;
		}
		
		// Advances the image row index to the given value
		void advanceRow(int nextRow) {
			assert(nextRow < (int)costRowOffsets.size());
			int prevRow = currentRow;
			currentRow = nextRow;
			advanceLeftFeature();
		
			for(int i=prevRow+1; i<=currentRow; i++) {
				costRowOffsets[i] = leftFeatureIndex;
				rightFeaturesTableRowOffsets[i] = rightFeaturesTable.size();
			}
		}
		
		// Adds a new right feature to the internal feature table for the current
		// image row
		void addRightFeature(int feature) {
			assert((int)rightFeaturesTable.size() - rightFeaturesTableRowOffsets[currentRow] < costs.cols -1 );
			rightFeaturesTable.push_back(feature);
		}
		
		// This method has to be called after cost values have been added to the
		// cost cube
		void finalizeCube() {
			advanceRow(costRowOffsets.size()-1);
		}
		
		// Gets a matrix containing all cost values for the given image row
		cv::Mat_<T> getCosts(int row) {
			assert(row < (int)costRowOffsets.size());
			return costs(cv::Rect(0, costRowOffsets[row], costs.cols,
				costRowOffsets[row+1] - costRowOffsets[row]));
		}
		//const cv::Mat_<T> getCosts(int row, int col) const {
		//	return costs(Rect(costRowOffsets[row], 0, costRowOffsets[row+1], costs.cols));
		//}
		
		// Returns the cost for matching the given left feature to the right feature
		// with the given index
		T getCost(int leftFeature, int rightIndex) const {
			assert(leftFeature < costs.rows);
			assert(rightIndex < costs.cols);
			return costs(leftFeature, rightIndex);
		}
		
		// Gets the number of right features stored in the feature table for
		// the given image row
		int getRightFeaturesCount(int row) const {
			assert(row < (int)rightFeaturesTableRowOffsets.size() -1);
			return rightFeaturesTableRowOffsets[row+1] - rightFeaturesTableRowOffsets[row];
		}
		
		// Returns the total number of left features
		int getLeftFeaturesCount() const {
			return costs.rows;
		}
		
		// Returns the row for the left feature
		int getLeftFeatureRow(int leftFeature) {
			assert(leftFeature < (int)featureRows.size());
			return featureRows[leftFeature];
		}
		
		// Gets the relative index of the left feature in its row
		int getLeftFeatureIndexInRow(int leftFeature) {
			return leftFeature - costRowOffsets[getLeftFeatureRow(leftFeature)];
		}
		
		// Gets a right feature from the internal feature table
		int getRightFeature(int row, int index) const {
			assert(index < getRightFeaturesCount(row));
			return rightFeaturesTable[rightFeaturesTableRowOffsets[row] + index];
		}
		
		// Gets the right feature index with the minimal matching cost for the
		// given left feature
		int getRightMinimumCostIndex(int leftFeature) const {
			assert(leftFeature < (int)minimumMatches.size());
			return minimumMatches[leftFeature];
		}
				
		// Sets the minimum match for the given left feature to an invalid value
		void invalidateMatch(int leftFeature) {
			assert(leftFeature < (int)minimumMatches.size());
			minimumMatches[leftFeature] = -1;
		}
		void invalidateMatch(int row, int rowLeftFeature) {
			assert(row < (int)costRowOffsets.size());
			assert(costRowOffsets[row] + rowLeftFeature < (int)minimumMatches.size());
			minimumMatches[costRowOffsets[row] + rowLeftFeature] = -1;
		}
		
		// Returns the number of image rows in the stereo pair
		int getRows() const {return costRowOffsets.size() -1;}
		
	private:
		cv::Mat_<T> costs;
		std::vector<int> featureRows;
		std::vector<int> costRowOffsets;
		std::vector<int> minimumMatches;
		std::vector<int> rightFeaturesTable;
		std::vector<int> rightFeaturesTableRowOffsets;
		int rightFeatureIndex;
		int leftFeatureIndex;
		int currentRow;
		T rightMinimumCost;
	};
}

#endif
