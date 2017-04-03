#include "libks/stereo/sparse/sparseconstraints.h"
#include <climits>

namespace ks {
	using namespace cv;
	using namespace std;

	/*void SparseConstraints::leftRightConsistency(const vector<Point2f>& leftFeatures, SparseCostCube<short>* costCube) {
		Mat_<short> costs;
		int currentRow = -1;
		int leftIndex = 0;
		
		for(int l = 0; l < (int)leftFeatures.size(); l++) {
			if(int(leftFeatures[l].y) != currentRow) {
				currentRow = (int)leftFeatures[l].y;
				costs = costCube->getCosts(currentRow);
				leftIndex = 0;
			}
		
			short minCost = SHRT_MAX;
			int leftMin = 0;
			
			int r = costCube->getRightMinimumCostIndex(l);
			for(int l2=0; l2<costs.rows; l2++)
				if(costs(l2,r) < minCost) {
					minCost = costs(l2, r);
					leftMin = l2;
				}
			
			
			if(leftMin != leftIndex)
				costCube->invalidateMatch(l);
			
			leftIndex++;
		}
	}
	
	void SparseConstraints::uniquenessConstraint(SparseCostCube<short>* costCube, float uniqueness) {
		for(int row=0; row<costCube->getRows(); row++) {
			Mat_<short> costs = costCube->getCosts(row);
			
			for(int l=0; l<costs.rows; l++) {
				int lowestRight = -1;
				short lowestCost = SHRT_MAX, secondLowestCost = SHRT_MAX;
				
				for(int r=0; r<costs.cols; r++) {
					if(costs(l, r) < lowestCost) {
						//if(r != lowestRight+1) // We ignore direct neighbors
						//	secondLowestCost = lowestCost;
						lowestCost = costs(l,r);
						lowestRight = r;
					}
				}
				
				if((float)lowestCost / secondLowestCost > uniqueness)
					costCube->invalidateMatch(row, l);
			}
		}
	}*/
	
	void SparseConstraints::combindedConsistencyUniqueness(SparseCostCube<short>* costCube, float uniqueness) {
		Mat_<short> costs;
		int currentRow = -1;
		
		for(int l = 0; l < costCube->getLeftFeaturesCount(); l++) {
			if(costCube->getLeftFeatureRow(l) != currentRow) {
				currentRow = costCube->getLeftFeatureRow(l);
				costs = costCube->getCosts(currentRow);
			}
		
			int r = costCube->getRightMinimumCostIndex(l);
			short minCost = costCube->getCost(l, r);
			int l1 = costCube->getLeftFeatureIndexInRow(l);
			
			for(int l2=0; l2<costs.rows; l2++) {
				if(l2 == l1)
					continue;
					
				if(costs(l2,r) < minCost/uniqueness) {
					costCube->invalidateMatch(l);
					break;
				}
			}
		}
	}
}