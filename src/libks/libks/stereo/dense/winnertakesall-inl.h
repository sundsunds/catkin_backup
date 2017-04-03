#ifndef KS_WINNERTAKESALL_INL_H
#define KS_WINNERTAKESALL_INL_H

#include "libks/stereo/dense/winnertakesall.h"
#include "libks/base/hammingdistance.h"
#include <vector>
#include <cstdio>
#include <iostream>

//#define COUNT_MATCHING_OPS

namespace ks {
	template<class CORRELATION, typename PIXEL_TYPE>
	WinnerTakesAll<CORRELATION, PIXEL_TYPE>::WinnerTakesAll(int maxDisp, bool consistencyCheck, float uniqueness,
		CORRELATION corr)
		: maxDisp(maxDisp), consistencyCheck(consistencyCheck), uniqueness(uniqueness), correlation(corr), matchOps(0) {
		border = correlation.getWindowSize() / 2;
	}
	
	template<class CORRELATION, typename PIXEL_TYPE>
	WinnerTakesAll<CORRELATION, PIXEL_TYPE>::~WinnerTakesAll() {
#ifdef COUNT_MATCHING_OPS
		std::cout << "Matching operations: " << matchOps << std::endl;
#endif
	}


	template<class CORRELATION, typename PIXEL_TYPE>
	void WinnerTakesAll<CORRELATION, PIXEL_TYPE>::match(const cv::Mat_<PIXEL_TYPE>& left,
		const cv::Mat_<PIXEL_TYPE>& right, cv::Mat_<unsigned char>* out) {
		
		(*out) = cv::Mat_<unsigned char>(left.rows, left.cols, (unsigned char) 255);
		
		correlation.setReferenceImage(left);
		correlation.setComparisonImage(right);
		
		if(!consistencyCheck)
			matchWithoutConsistencyCheck(out, left.rows, left.cols);
		else matchWithConsistencyCheck(out, left.rows, left.cols);
	}
		
	template<class CORRELATION, typename PIXEL_TYPE>
	void WinnerTakesAll<CORRELATION, PIXEL_TYPE>::matchWithoutConsistencyCheck(cv::Mat_<unsigned char>* out, int rows, int cols) {
		
		// This code can be merged with the L/R consistency check version
		for(int y = border; y < rows-border; y++)
			for(int x1 = border; x1 < cols-border; x1++) {
				correlation.setReferencePoint(cv::Point2i(x1, y));
				
				int minX = -1;
				int minCost = 0xFFFF;
				for(int x2 = std::max(border, x1 - maxDisp); x2 <= x1; x2++) {
#ifdef COUNT_MATCHING_OPS
					matchOps++;
#endif
					int cost = correlation.match(cv::Point(x2, y));
					
					if(cost < minCost) {
						minCost = cost;
						minX = x2;
					}
				}
				
				(*out)(y,x1) = x1 - minX;
			}	
	}
	
	template<class CORRELATION, typename PIXEL_TYPE>
	void WinnerTakesAll<CORRELATION, PIXEL_TYPE>::matchWithConsistencyCheck(cv::Mat_<unsigned char>* out, int rows, int cols) {
		
		cv::Mat_<unsigned short> costs(cols, maxDisp+1);
		
		for(int y = border; y < rows-border; y++) {
		
			// Fist perform left / right matching and store costs
			for(int x1 = border; x1 < cols-border; x1++) {
				correlation.setReferencePoint(cv::Point2i(x1, y));
				
				int minX = -1;
				int minCost = 0xFFFF;
				
				for(int x2 = std::max(border, x1 - maxDisp); x2 <= x1; x2++) {
#ifdef COUNT_MATCHING_OPS
					matchOps++;
#endif				
					int cost = correlation.match(cv::Point(x2, y));
					
					if(cost < minCost) {
						minCost = cost;
						minX = x2;
					}
					costs(x1, x1 - x2) = cost;
				}
				
				(*out)(y,x1) = x1 - minX;
			}
			
			// Perform right / left evaluation of collected costs and invalidate
			// inconsistent matches
			for(int x = border; x < cols-border; x++) {
				int rightX = x - (*out)(y, x); // Best match
				int minCost = costs(x, (*out)(y,x));
				
				int maxLeftX = std::min(cols-border-1, rightX + maxDisp);
				for(int leftX = rightX; leftX <= maxLeftX; leftX++) {
					if(leftX == x)
						continue;
									
					if(costs(leftX, leftX - rightX) < minCost/uniqueness && abs(leftX - x) > 1) {
						// Found a better match. Mark as inconsistent
						(*out)(y,x) = 255;
						break;
					}
				}
			}
		}
	}
}

#endif
