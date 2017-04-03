#ifndef KS_TEST_PYRAMIDRANGEDETECTOR_H
#define KS_TEST_PYRAMIDRANGEDETECTOR_H

#include "libks/feature/pyramidrangedetector.h"
#include "libks/feature/featuretester.h"
#include <opencv2/opencv.hpp>
#include <cppunit/TestFixture.h>
#include <cppunit/TestAssert.h>
#include <cppunit/TestSuite.h>
#include <cppunit/TestCaller.h>

namespace ks {
	class TestPyramidRangeDetector: public CppUnit::TestFixture {
	public:
		void setUp() {};
		void tearDown() {}
		
		void testDetector(int step);
		void testDetector1() {testDetector(1);}
		void testDetector2() {testDetector(2);}
		
		void testTester(int step);
		void testTester1() {testTester(1);}
		void testTester2() {testTester(2);}
		
		static CppUnit::Test* suite() {
			CppUnit::TestSuite *suiteOfTests = new CppUnit::TestSuite("PyramidRangeDetector");
			suiteOfTests->addTest(new CppUnit::TestCaller<TestPyramidRangeDetector>(
				"testDetector1", &TestPyramidRangeDetector::testDetector1));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestPyramidRangeDetector>(
				"testDetector2", &TestPyramidRangeDetector::testDetector2));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestPyramidRangeDetector>(
				"testTester1", &TestPyramidRangeDetector::testTester1));
			suiteOfTests->addTest(new CppUnit::TestCaller<TestPyramidRangeDetector>(
				"testTester2", &TestPyramidRangeDetector::testTester2));
			return suiteOfTests;
		}
		
	private:
		class DummyDetector:  public cv::FeatureDetector, public FeatureTester {
		public:
			DummyDetector(int step): step(step) {};
			
			virtual bool testFeature(const cv::Mat& image, cv::Point2i pt);
		
		protected:
			virtual void detectImpl(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const;	
		
		private:
			int step;
		};
		
		void performTest(PyramidRangeDetector* detector, int step);
	};
}

#endif
