#ifndef KS_SPARSECONSTRAINTS_H
#define KS_SPARSECONSTRAINTS_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "libks/stereo/sparse/sparsecostcube.h"

namespace ks {
	// Constraints employed by sparse stereo algorithms
	class SparseConstraints {
	public:
		//static void leftRightConsistency(const std::vector<cv::Point2f>& leftFeatures, SparseCostCube<short>* costCube);
		//static void uniquenessConstraint(SparseCostCube<short>* costCube, float uniqueness);
		static void combindedConsistencyUniqueness(SparseCostCube<short>* costCube, float uniqueness);
	};
}

#endif
