#ifndef KS_TEST_SPARSECOSTCUBE_H
#define KS_TEST_SPARSECOSTCUBE_H

#include "libks/stereo/sparse/sparsecostcube.h"
#include <cppunit/TestFixture.h>
#include <cppunit/TestAssert.h>
#include <cppunit/TestSuite.h>
#include <cppunit/TestCaller.h>

namespace ks {
	class TestSparseCostCube: public CppUnit::TestFixture {
	public:
		TestSparseCostCube();
		
		void setUp() {};
		void tearDown() {}
		
		void testRightFeaturesCount();
		void testGetRightFeature();
		void testGetRightMinimumCostIndex();
		void testGetCosts();
		void testGetLeftFeatureRow();
		void testGetLeftFeatureIndexInRow();
		
		static CppUnit::Test* suite() {
			CppUnit::TestSuite *suiteOfTests = new CppUnit::TestSuite("SparseCostCube");
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testRightFeaturesCount", &TestSparseCostCube::testRightFeaturesCount));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testGetRightFeature", &TestSparseCostCube::testGetRightFeature));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testGetRightMinimumCostIndex", &TestSparseCostCube::testGetRightMinimumCostIndex));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testGetCosts", &TestSparseCostCube::testGetCosts));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testGetLeftFeatureRow", &TestSparseCostCube::testGetLeftFeatureRow));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestSparseCostCube>(
				"testGetLeftFeaturIndexInRoweRow", &TestSparseCostCube::testGetLeftFeatureIndexInRow));
			return suiteOfTests;
		}
		
	private:
		SparseCostCube<short> cube;
	};
}

#endif
