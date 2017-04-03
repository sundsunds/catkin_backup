#include "libks/feature/pyramidrangedetector.h"
#include "libks/base/exception.h"
#include "libks/imageproc/imageconversion.h"

namespace ks {
	using namespace std;
	using namespace cv;
	using namespace boost;
	
	void PyramidRangeDetector::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		if(image.type() == CV_8S && unsignedImg == NULL)
			throw Exception("For signed char images, an alternative unsigned version is required");
		if(storePyramid)
			pyramid.clear();
	
		// Create result vectors for all pyramids
		vector<vector<KeyPoint>*> pyramidLevels(maxLevel+1);
		vector<shared_ptr<vector<KeyPoint> > > pyramidStorage(maxLevel);
		pyramidLevels[0] = &keypoints; // Level 0 is the final result
		for(int i=0; i<maxLevel; i++) {
			pyramidStorage[i].reset(new vector<KeyPoint>);
			pyramidLevels[i+1] = pyramidStorage[i].get();
		}
		
		// Detect points for 0-level
		keypoints.clear();
		detector->detect(image, *(pyramidLevels[0]), mask);
		
		Mat src;
		if(image.type() != CV_8S)
			src = image;
		else src = *unsignedImg;
		
		// Detect remaining levels
		for(int i=1; i<=maxLevel; i++) {
			// Downsample
			Mat dst;
			pyrDown(src, dst);
			src = dst;
			if(storePyramid)
				pyramid.push_back(dst);
			
			Mat detectorImg;
			if(image.type() != CV_8S)
				detectorImg = src;
			else {
				Mat_<char> tmp(src.rows, src.cols);
				ImageConversion::unsignedToSigned(src, &tmp);
				detectorImg = tmp;
			}
			
			// Detect new points
			if(tester.get() != NULL)
				fastDetection(detectorImg, *(pyramidLevels[i-1]), *(pyramidLevels[i]));
			else detector->detect(detectorImg, *(pyramidLevels[i]), mask);
		}
		
		// Find pyramid range
		for(int i=maxLevel; i>0; i--)
			findLevels(*(pyramidLevels[i-1]), *(pyramidLevels[i]));
	}
	
	void PyramidRangeDetector::fastDetection(const Mat& image, const vector<KeyPoint>& prevPoints,
		vector<KeyPoint>& newPoints) const {
		
		unsigned int evenRowIndex = 0;
		int prevRow = -1;
		
		for(unsigned int i=0; i<prevPoints.size(); i++) {
			// We store the starting point of every even row in the new points list
			if(prevRow != prevPoints[i].pt.y) {
				if(int(prevPoints[i].pt.y) %2 == 0)
					evenRowIndex = newPoints.size();
				prevRow = prevPoints[i].pt.y;
			}
		
			Point2f newPoint = Point2f(int(prevPoints[i].pt.x)/2, int(prevPoints[i].pt.y)/2);
			if(tester->testFeature(image, newPoint)) {
				// Check if we added the same point in the previous column
				if(newPoints.size() > 0 && newPoints.back().pt == newPoint)
					continue; // We already added a point at this location
				// Every second row we check if we added the same point in the previous row
				if(int(prevPoints[i].pt.y) % 2 == 1) {
					while(evenRowIndex < newPoints.size() && newPoints[evenRowIndex].pt.y == newPoint.y &&
						newPoints[evenRowIndex].pt.x < newPoint.x)
						evenRowIndex++;
						
					if(evenRowIndex < newPoints.size() && newPoints[evenRowIndex].pt == newPoint)
						continue; // We already added a point at this location
				}
					
				newPoints.push_back(KeyPoint(newPoint, 0));
			}
		}
	}
	
	void PyramidRangeDetector::findLevels(vector<KeyPoint>& prevPoints, const vector<KeyPoint>& newPoints) const {
		int prevRow = -1;
		unsigned int prevIndex1 = 0, prevIndex2 = 0; //Indices for even and uneven rows in prev. level
		
		for(unsigned int newP = 0; newP < newPoints.size(); newP++) {
			int newY = newPoints[newP].pt.y;
			int newX = newPoints[newP].pt.x;
			
			if(newY != prevRow) {
				// Find new row starting indices for the previous pyramid level
				prevIndex1 = prevIndex2;
				while(prevIndex1 < prevPoints.size() && prevPoints[prevIndex1].pt.y < newY*2)
					prevIndex1++;
				prevIndex2 = prevIndex1;
				while(prevIndex2 < prevPoints.size() && prevPoints[prevIndex2].pt.y < newY*2+1)
					prevIndex2++;
				prevRow = newY;
			}
			
			// First search the even rows
			while(prevIndex1 < prevPoints.size() && int(prevPoints[prevIndex1].pt.y)/2 == newY
				&& int(prevPoints[prevIndex1].pt.x)/2 < newX)
				prevIndex1++;
				
			if(prevIndex1 < prevPoints.size() && int(prevPoints[prevIndex1].pt.y)/2 == newY && int(prevPoints[prevIndex1].pt.x)/2 == newX)
				prevPoints[prevIndex1].octave = newPoints[newP].octave + 1;
			else {
				// Then search the uneven rows
				while(prevIndex2 < prevPoints.size() && int(prevPoints[prevIndex2].pt.y)/2 == newY
					&& int(prevPoints[prevIndex2].pt.x)/2 < newX)
					prevIndex2++;
					
				if(prevIndex2 < prevPoints.size() && int(prevPoints[prevIndex2].pt.y)/2 == newY && int(prevPoints[prevIndex2].pt.x)/2 == newX)
					prevPoints[prevIndex2].octave = newPoints[newP].octave + 1;
			}
		}
	}
}
