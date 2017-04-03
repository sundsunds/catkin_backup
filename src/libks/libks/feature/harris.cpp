#include "libks/feature/harris.h"
#include "libks/feature/fast9-inl.h"

using namespace cv;
using namespace std;

namespace ks {
	Harris::Harris(double threshold, bool nonMaxSuppression, int border, int blockSize,
		int apertureSize, double k)
		: threshold(threshold), nonMaxSuppression(nonMaxSuppression), blockSize(blockSize),
		apertureSize(apertureSize), k(k), border(border) {
		corners.reserve(512);
		scores.reserve(512);
	}
	
	void Harris::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		// Allocate buffer for processing
		if(buffer.size() != image.size())
			buffer = Mat_<float>(image.rows, image.cols);
		
		// Run OpenCV Harris detector	
		cornerHarris(image, buffer, blockSize, apertureSize, k);
		
		corners.clear();
		scores.clear();
		
		// Perform thresholding
		for(int y=border; y<buffer.rows-border; y++)
			for(int x=border; x<buffer.cols-border; x++)
				if(buffer(y,x) >= threshold) {
					corners.push_back(Point2i(x,y));
					scores.push_back(buffer(y,x));
				}
					
		if(nonMaxSuppression) {
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(corners, scores, nonmaxPoints);
			
			// Copy results
			keypoints.reserve(nonmaxPoints.size());
			for(unsigned int i=0; i<nonmaxPoints.size(); i++) {
				int index = nonmaxPoints[i];
				keypoints.push_back(KeyPoint(Point2f(corners[index].x, corners[index].y), 6.f, -1.f, scores[index]));
			}
		} else {
			// Copy everything
			keypoints.reserve(corners.size());
			for(unsigned int i = 0; i < corners.size(); i++ )
				keypoints.push_back(KeyPoint(corners[i], 6.f, -1.f, scores[i]));
		}
	}
}
