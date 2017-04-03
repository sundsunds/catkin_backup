#include "libks/tests/pyramidrangedetector.h"
#include <cstring>

#define MAX_LEVEL 4

namespace ks {
	using namespace cv;
	using namespace std;
	using namespace boost;
	
	void TestPyramidRangeDetector::DummyDetector::detectImpl(const Mat& image,
		vector<KeyPoint>& keypoints, const Mat& mask) const {
		for(int y = 0; y<image.rows; y+=step)
			for(int x = 0; x<image.cols; x+=step)
				keypoints.push_back(KeyPoint(x, y, 0));
	}
	
	bool TestPyramidRangeDetector::DummyDetector::testFeature(const cv::Mat& image, cv::Point2i pt) {
		return pt.x%step == 0 && pt.y%step == 0;
	}
	
	void TestPyramidRangeDetector::testDetector(int step) {
		PyramidRangeDetector detector(shared_ptr<FeatureDetector>(new DummyDetector(step)), MAX_LEVEL);
		performTest(&detector, step);
	}
	
	void TestPyramidRangeDetector::testTester(int step) {
		shared_ptr<DummyDetector> baseDetector(new DummyDetector(step));
		PyramidRangeDetector detector(baseDetector, baseDetector, MAX_LEVEL);
		performTest(&detector, step);
	}
	
	void TestPyramidRangeDetector::performTest(PyramidRangeDetector* detector, int step) {
		Mat_<unsigned char> dummyImg(480, 640);
		vector<KeyPoint> points;
		detector->detect(dummyImg, points);
		
		int counts[MAX_LEVEL+1];
		memset(counts, 0, sizeof(counts));
		
		for(unsigned int i=0; i<points.size(); i++) {
			for(int j=0; j<=points[i].octave; j++)
				counts[j]++;
		}
		
		CPPUNIT_ASSERT(counts[0] == dummyImg.rows * dummyImg.cols / (step*step));
		for(int i=1; i<=MAX_LEVEL; i++) {
			CPPUNIT_ASSERT(counts[i] == counts[i-1]/4);
		}
	}
}
