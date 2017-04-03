#include "libks/feature/evaluation/clusterevaluation.h"
#include <climits>
#include <cmath>

//#define K_NEAREST	1
//#define NEIGHBORHOOD_SIZE	10
#define RECTANGLES	10

namespace ks {
	using namespace cv;
	using namespace std;

	double ClusterEvaluation::evaluate(const vector<KeyPoint>& points, Rect_<float> roi) {
		vector<Point2f> cvPoints(points.size());
		for(unsigned int i=0; i<points.size(); i++)
			cvPoints[i] = points[i].pt;
		return evaluate(cvPoints, roi);
	}
	
	double ClusterEvaluation::evaluate(const std::vector<cv::Point2f>& points, Rect_<float> roi) {
		//if(points.size() <= K_NEAREST)
		if(points.size() <= 1)
			return -1; // No evaluation possible
		
		/*vector<unsigned int> neighbors(points.size());
			
		double sum = 0;
		for(unsigned int i=0; i<points.size(); i++) {
			neighbors[i] = countCloseNeighbors(points, i);
			sum += neighbors[i];
		}
		
		double avg = sum / points.size();
		double deviation = 0.0;
		for(unsigned int i=0; i<points.size(); i++)
			deviation += fabs(neighbors[i] - avg);
		
		return (deviation / points.size()) / avg;*/
		
		vector<double> counts;
		unsigned int totalSum = 0;
		
		double width = roi.width / double(RECTANGLES);
		double height = roi.height / double(RECTANGLES);
		
		for(int y=0; y<RECTANGLES; y++)
			for(int x=0; x<RECTANGLES; x++) {
				Rect_<float> rect(roi.x + x*width, roi.y + y*height, width, height);
				counts.push_back(countPointsInRect(points, rect));
				totalSum += counts.back();
			}
			
		for(unsigned int i=0; i<counts.size(); i++)
			counts[i] = counts[i] / totalSum;
		
		double avg = 1.0 / double(counts.size());
		double deviation = 0;			
		for(unsigned int i=0;i<counts.size();i++)
			deviation += (counts[i] - avg)*(counts[i] - avg);
			
		return sqrt(deviation / (counts.size()-1));
	}
	
	unsigned int ClusterEvaluation::countPointsInRect(const std::vector<cv::Point2f>& points, Rect_<float> roi) {
		int count = 0;
		for(unsigned int i = 0; i<points.size(); i++)
			if(roi.contains(points[i]))
				count++;
		return count;
	}
	
	/*unsigned int ClusterEvaluation::countCloseNeighbors(const vector<Point2f>& points, unsigned int ref) {
		unsigned int count = 0;
	
		for(unsigned int i = 0; i<points.size(); i++) {
			if(i == ref)
				continue;
			
			float dx = points[i].x - points[ref].x;
			float dy = points[i].y - points[ref].y;
			double dist = sqrt(dx*dx + dy*dy);
			
			if(dist <= NEIGHBORHOOD_SIZE)
				count++;
		}
		
		return count;
	}*/
	
	/*double ClusterEvaluation::findNearestNeighborDistances(const vector<Point2f>& points, unsigned int ref) {
		double minDist[K_NEAREST];
		for(int i=0; i<K_NEAREST; i++)
			minDist[i] = DBL_MAX;
		
		// Find minimum distances
		for(unsigned int i=0; i<points.size(); i++) {
			if(i == ref)
				continue;
				
			float dx = points[i].x - points[ref].x;
			float dy = points[i].y - points[ref].y;
			double dist = sqrt(dx*dx + dy*dy);
			
			for(int j=0; j<K_NEAREST; j++)
				if(dist < minDist[j]) {
					// move other mins
					for(int k=K_NEAREST-1; k>j; k--)
						minDist[k] = minDist[k-1];
					// insert new value
					minDist[j] = dist;
					break;
				}
		}
		
		// Calc average
		double sum =0;
		for(int i=0; i<K_NEAREST; i++)
			sum += minDist[i];
		return sum / K_NEAREST;
	}*/
}
