#ifndef KS_TEST_HAMMINGDISTANCE_H
#define KS_TEST_HAMMINGDISTANCE_H

#include "libks/base/hammingdistance.h"
#include <cppunit/TestFixture.h>
#include <cppunit/TestAssert.h>
#include <cppunit/TestSuite.h>
#include <cppunit/TestCaller.h>

namespace ks {
	class TestHammingDistance: public CppUnit::TestFixture {
	public:
		void setUp() {}
		void tearDown() {}
		
		void testBitCount() {
			CPPUNIT_ASSERT(hammingDist.countBits(0U) == 0);
			CPPUNIT_ASSERT(hammingDist.countBits(0xFFFFFFFFU) == 32);
			CPPUNIT_ASSERT(hammingDist.countBits(0xAAAAAAAAU) == 16);
			CPPUNIT_ASSERT(hammingDist.countBits(0x55555555U) == 16);
			CPPUNIT_ASSERT(hammingDist.countBits(2628617491U) == 15);
			CPPUNIT_ASSERT(hammingDist.countBits(3686103184U) == 17);
		}
		
		void testHammingDistance() {
			CPPUNIT_ASSERT(hammingDist.calculate(0, 0xFFFFFFFFU) == 32);
			CPPUNIT_ASSERT(hammingDist.calculate(0, 0xAAAAAAAAU) == 16);
			CPPUNIT_ASSERT(hammingDist.calculate(0xAAAAAAAAU, 0x55555555U) == 32);
			CPPUNIT_ASSERT(hammingDist.calculate(2628617491U, 3686103184U) == 16);
		}
		
		static CppUnit::Test* suite() {
			CppUnit::TestSuite *suiteOfTests = new CppUnit::TestSuite("HammingDistance");
			suiteOfTests->addTest(new CppUnit::TestCaller<TestHammingDistance>(
				"testBitCount", &TestHammingDistance::testBitCount));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestHammingDistance>(
				"testHammingDistance", &TestHammingDistance::testHammingDistance));
			return suiteOfTests;                                                                                                                                                        
		}
		
	private:
		HammingDistance hammingDist;
	};
}

#endif
