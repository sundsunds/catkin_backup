#ifndef KS_TEST_CENSUS_H
#define KS_TEST_CENSUS_H

#include "libks/imageproc/census.h"
#include <opencv2/opencv.hpp>
#include <cppunit/TestFixture.h>
#include <cppunit/TestAssert.h>
#include <cppunit/TestSuite.h>
#include <cppunit/TestCaller.h>

namespace ks {
	class TestCensus: public CppUnit::TestFixture {
	public:
		TestCensus();
		
		void setUp();
		void tearDown() {}
		
		void test9x3Int();
		void test9x3Float();
		void test5x5Int();
		void test5x5Float();
		
		static CppUnit::Test* suite() {
			CppUnit::TestSuite *suiteOfTests = new CppUnit::TestSuite("Census");
			suiteOfTests->addTest(new CppUnit::TestCaller<TestCensus>(
				"test9x3Int", &TestCensus::test9x3Int));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestCensus>(
				"test9x3Float", &TestCensus::test9x3Float));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestCensus>(
				"test5x5Int", &TestCensus::test5x5Int));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestCensus>(
				"test5x5Float", &TestCensus::test5x5Float));
			return suiteOfTests;
		}
		
	private:
		static char pixel[16*8];
	
		cv::Mat_<char> testImageInt;
		cv::Mat_<char> testImageFloat;
		cv::Mat_<unsigned int> censusImage;
		Census census;
	};
}

#endif
