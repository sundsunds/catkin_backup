#include "libks/tests/sparsecostcube.h"

#include <iostream>
#include <iomanip>
#include <climits>
using namespace std;

namespace ks {
	using namespace cv;

	TestSparseCostCube::TestSparseCostCube()
		: cube(4, 4, 7)
	{	
		// Row 0
		cube.advanceRow(0);
		cube.addRightFeature(1);
		cube.addRightFeature(2);
		cube.pushCost(10);
		cube.pushCost(20);
		cube.advanceLeftFeature();
		cube.pushCost(-10);
		cube.pushCost(-20);
		
		// Row 1
		
		// Row 2
		cube.advanceRow(2);
		cube.addRightFeature(5);
		cube.addRightFeature(3);
		cube.addRightFeature(4);
		cube.pushCost(50);
		cube.pushCost(30);
		cube.pushCost(40);
		
		// Row 3
		cube.advanceRow(3);
		cube.addRightFeature(6);
		cube.pushCost(60);
		cube.advanceLeftFeature();
		cube.pushCost(61);
		cube.advanceLeftFeature();
		cube.pushCost(62);
		
		cube.finalizeCube();
	}
	
	void TestSparseCostCube::testRightFeaturesCount() {
		CPPUNIT_ASSERT(cube.getRightFeaturesCount(0) == 2);
		CPPUNIT_ASSERT(cube.getRightFeaturesCount(1) == 0);
		CPPUNIT_ASSERT(cube.getRightFeaturesCount(2) == 3);
		CPPUNIT_ASSERT(cube.getRightFeaturesCount(3) == 1);
	}
	
	void TestSparseCostCube::testGetRightFeature() {
		CPPUNIT_ASSERT(cube.getRightFeature(0, 0) == 1);
		CPPUNIT_ASSERT(cube.getRightFeature(0, 1) == 2);
		CPPUNIT_ASSERT(cube.getRightFeature(2, 0) == 5);
		CPPUNIT_ASSERT(cube.getRightFeature(2, 1) == 3);
		CPPUNIT_ASSERT(cube.getRightFeature(2, 2) == 4);
		CPPUNIT_ASSERT(cube.getRightFeature(3, 0) == 6);
	}
	
	void TestSparseCostCube::testGetRightMinimumCostIndex() {
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(0) == 0);
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(1) == 1);
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(2) == 1);
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(3) == 0);
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(4) == 0);
		CPPUNIT_ASSERT(cube.getRightMinimumCostIndex(5) == 0);
	}
	
	void TestSparseCostCube::testGetLeftFeatureRow() {
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(0) == 0);
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(1) == 0);
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(2) == 2);
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(3) == 3);
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(4) == 3);
		CPPUNIT_ASSERT(cube.getLeftFeatureRow(5) == 3);
	}
	
	void TestSparseCostCube::testGetLeftFeatureIndexInRow() {
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(0) == 0);
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(1) == 1);
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(2) == 0);
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(3) == 0);
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(4) == 1);
		CPPUNIT_ASSERT(cube.getLeftFeatureIndexInRow(5) == 2);
	}
	
	void TestSparseCostCube::testGetCosts() {
		Mat_<short> costs = cube.getCosts(0);
		CPPUNIT_ASSERT(costs.rows == 2);
		CPPUNIT_ASSERT(costs(0, 0) == 10);
		CPPUNIT_ASSERT(costs(0, 1) == 20);
		CPPUNIT_ASSERT(costs(1, 0) == -10);
		CPPUNIT_ASSERT(costs(1, 1) == -20);
		CPPUNIT_ASSERT(costs(1, 2) == SHRT_MAX);
		
		costs = cube.getCosts(2);
		CPPUNIT_ASSERT(costs.rows == 1);
		
		costs = cube.getCosts(3);
		CPPUNIT_ASSERT(costs.rows == 3);
	}
}