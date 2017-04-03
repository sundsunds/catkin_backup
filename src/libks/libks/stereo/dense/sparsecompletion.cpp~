#include "sparsecompletion.h"
#include "libks/stereo/correlation/censuswindow-inl.h"
#include "triangle.h" //elas
//#include "/home/ait_jellal/projects/ws_mav/src/libelas/src/triangle.h"
#include <limits>

namespace ks {
	using namespace cv;
	using namespace std;

	SparseCompletion::SparseCompletion(int maxDisp, int subsampling, StereoRectification* rect)
		: maxDisp(maxDisp), subsampling(subsampling), rect(rect) {
	}

	void SparseCompletion::match(const std::vector<SparseMatch>& sparseMatches,
		const Mat_<unsigned int>& left, const Mat_<unsigned int>& right,
		Mat_<float>& dispMap) {
		
		cv::Size subSize(left.cols / 4, left.rows / 4);
		if(minDispMap.size() != subSize) {
			// Allocate buffers
			minDispMap = Mat_<float>(subSize, (unsigned char)0);
			maxDispMap = Mat_<float>(subSize, (unsigned char)0);
		} else {
			// Reset buffers
			minDispMap = Mat_<float>::zeros(subSize.height, subSize.width);
			maxDispMap = Mat_<float>::zeros(subSize.height, subSize.width);
		}
		
		interpolateMatches(sparseMatches, dispMap);
		//refineDispMap(dispMap, left, right);
	}
	
	void SparseCompletion::filterTriangles(const std::vector<SparseMatch>& sparseMatches, triangulateio& tri) {
		for (int triangle=0; triangle < tri.numberoftriangles; triangle++) {
			float minDisp = maxDisp;
			float maxDisp = 0;
		
			for(int neighbor = 0; neighbor < 3; neighbor++) {
				int neighborTriangle = tri.neighborlist[3*triangle + neighbor];
				if(neighborTriangle < 0)
					continue; // No more neighbors
				
				for(int neighborPoint = 0; neighborPoint < 3; neighborPoint++) {
					int point = tri.trianglelist[3*neighborTriangle + neighborPoint] & 0x0FFFFFFF;
					int disparity = sparseMatches[point].disparity();
					if(disparity < minDisp)
						minDisp = disparity;
					if(disparity > maxDisp)
						maxDisp = disparity;
				}
			}
		
			if(maxDisp - minDisp > 15)
				tri.trianglelist[3*triangle] |= 0xF0000000; // Mark as invalid
		}
	}
	
	void SparseCompletion::interpolateMatches(const std::vector<SparseMatch>& sparseMatches,
		Mat_<float>& dispMap) {
		
		// TODO: Replace with http://code.google.com/p/poly2tri/
		// input/output structure for triangulation
		struct triangulateio in, out;

		// inputs
		in.numberofpoints = sparseMatches.size();
		in.pointlist = (float*)malloc(in.numberofpoints*2*sizeof(float));
		int k=0;
		for (unsigned int i=0; i<sparseMatches.size(); i++) {
			in.pointlist[k++] = sparseMatches[i].rectLeft.x;
			in.pointlist[k++] = sparseMatches[i].rectLeft.y;
		}

		in.numberofpointattributes = 0;
		in.pointattributelist      = NULL;
		in.pointmarkerlist         = NULL;
		in.numberofsegments        = 0;
		in.numberofholes           = 0;
		in.numberofregions         = 0;
		in.regionlist              = NULL;
		
		// outputs
		out.pointlist              = NULL;
		out.pointattributelist     = NULL;
		out.pointmarkerlist        = NULL;
		out.trianglelist           = NULL;
		out.triangleattributelist  = NULL;
		out.neighborlist           = NULL;
		out.segmentlist            = NULL;
		out.segmentmarkerlist      = NULL;
		out.edgelist               = NULL;
		out.edgemarkerlist         = NULL;

		// do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
		char parameters[] = "znQB";
		triangulate(parameters, &in, &out, NULL);
		
		filterTriangles(sparseMatches, out);
		
		k=0;
		// Very slow!! Needs replacing (OpenGL)
		for (int i=0; i<out.numberoftriangles; i++) {
			if(out.trianglelist[k] > 0) { // Not filtered
				interpolateTriangle(sparseMatches[out.trianglelist[k]],
					sparseMatches[out.trianglelist[k+1]],
					sparseMatches[out.trianglelist[k+2]], dispMap);
			}
			k+=3;
		}
		
		// free memory used for triangulation
		free(in.pointlist);
		free(out.pointlist);
		free(out.trianglelist);
		free(out.neighborlist);
	}
	
	void SparseCompletion::interpolateTriangle(const SparseMatch& a, const SparseMatch& b, 
		const SparseMatch& c, Mat_<float>& dst) {
		
		Point2f pA(a.rectLeft.x/subsampling, a.rectLeft.y/subsampling),
			pB(b.rectLeft.x/subsampling, b.rectLeft.y/subsampling),
			pC(c.rectLeft.x/subsampling, c.rectLeft.y/subsampling);
		
		float minX = max(0.0F, min(pA.x, min(pB.x, pC.x)));
		float minY = max(0.0F, min(pA.y, min(pB.y, pC.y)));
		float maxX = min(float(dst.cols), max(pA.x, max(pB.x, pC.x)));
		float maxY = min(float(dst.rows), max(pA.y, max(pB.y, pC.y)));
		
		unsigned char minDisp = min(a.disparity(), min(b.disparity(), c.disparity()));
		unsigned char maxDisp = max(a.disparity(), max(b.disparity(), c.disparity()));
		
		for(Point2i p(0,(int)minY); p.y <=maxY; p.y++)
			for(p.x = (int)minX; p.x <= maxX; p.x++) {
				if(pointInTriangle(p, pA, pB, pC)) {
					Point3f bary = baryCoords(p, pA, pB, pC);
					
					dst(p.y,p.x) = 1.0F / (
						bary.x * 1.0F/a.disparity()
						+ bary.y * 1.0F/b.disparity()
						+ bary.z * 1.0F/c.disparity());
						
					minDispMap(p) = minDisp;
					maxDispMap(p) = maxDisp;
				}
			}	
	}
	
	void SparseCompletion::refineDispMap(cv::Mat_<float>& dispMap, const cv::Mat_<unsigned int>& left,
		const cv::Mat_<unsigned char>& right) {
		
		cv::Mat_<short> costs(left.cols, maxDisp+1); // Buffer for keeping current costs
		
		CensusWindow<5> correlation;
		correlation.setReferenceImage(left);
		correlation.setComparisonImage(right);
		
		// Compute image boundaries (TODO: min/max from triangulation)
		int startY = correlation.getWindowSize() / 2;
		int endY = left.rows - correlation.getWindowSize() / 2;
		int startX = correlation.getWindowSize() / 2 + 1;
		int endX = left.cols - correlation.getWindowSize() / 2 - 1;
		
		for(int y = startY; y <= endY; y++) {
			// Reset costs to -1
			memset(costs.data, 0xFF, costs.rows * costs.step[0]);
			
			// Perform left / right matching and store costs
			for(int xL = startX; xL <= endX; xL++) {
				if(maxDispMap(y, xL) == 0)
					continue; // Outside of triangulated region
			
				correlation.setReferencePoint(Point2i(xL,y));
				
				int refX = xL - int(dispMap(y, xL) + 0.5);
				//int startXR = max(startX, xL - maxDispMap(y, xL));
				//int endXR = min(endX, xL - minDispMap(y, xL));
				//int startXR = max(startX, xL - min((int)maxDispMap(y, xL), dispMap(y, xL) + 5));
				//int endXR = min(endX, xL - max((int)minDispMap(y, xL), dispMap(y, xL) - 5));
				int startXR = max(startX, xL - (int(dispMap(y, xL) + 0.5) + 2));
				int endXR = min(endX, xL - (int(dispMap(y, xL) + 0.5) - 2));
				
				int minCost = numeric_limits<int>::max();
				int bestXR = endXR;
				const double lambda = 0;//15.0;
				const double maxSmooth = 100.0;
				
				for(int xR = startXR; xR <= endXR; xR++) {
					int matchCost = correlation.match(Point2i(xR, y));
					int totalCost = matchCost + max(maxSmooth, lambda * abs(xR - refX)*abs(xR - refX));
					if(totalCost < minCost) {
						minCost = totalCost;
						bestXR = xR;
					}
					costs(xL, xL - xR) = totalCost;
				}
				
				dispMap(y, xL) = xL - bestXR;
			}
			
			// Perform right / left evaluation of collected costs and invalidate
			// inconsistent matches
			for(int x = startX; x <= endX; x++) {
				int xR = x - int(dispMap(y, x) + 0.5); // Best match
				int minCost = costs(x, dispMap(y, x));
				
				int maxXL = min(endX, xR + maxDisp);
				for(int xL = xR; xL <= maxXL; xL++) {
					int cost = costs(xL, xL - xR);
				
					if(xL == x || cost < 0)
						continue;
									
					const float uniqueness = 0.6;
					if(cost < minCost/uniqueness && abs(xL - x) > 2) {
						// Found a better match. Mark as inconsistent
						dispMap(y, x) = -1.0F;
						break;
					}
				}
			}
		}
	}
}
