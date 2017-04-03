#ifndef KS_CENSUS_INL_H
#define KS_CENSUS_INL_H

#include "libks/base/exception.h"
#include "libks/imageproc/census.h"

namespace ks {
	template <typename T>
	void Census::transform9x3(const cv::Mat_<T>& input, cv::Mat_<unsigned int>* output) {
		int maxX = input.cols-4, maxY = input.rows-1;
		
		//#pragma omp parallel for default(none) shared(input, output, maxX, maxY)
		for(int y=1; y<maxY; y++)
			for(int x=4; x<maxX; x++) {
				T centoid = input(y,x);
				(*output)(y,x) =
					// Row 1
					((centoid > input(y-1, x-4)) << 26) |
					((centoid > input(y-1, x-3)) << 25) |
					((centoid > input(y-1, x-2)) << 24) |
					((centoid > input(y-1, x-1)) << 23) |
					((centoid > input(y-1, x)) << 22) |
					((centoid > input(y-1, x+1)) << 21) |
					((centoid > input(y-1, x+2)) << 20) |
					((centoid > input(y-1, x+3)) << 19) |
					((centoid > input(y-1, x+4)) << 18) |
					// Row 2
					((centoid > input(y, x-4)) << 17) |
					((centoid > input(y, x-3)) << 16) |
					((centoid > input(y, x-2)) << 15) |
					((centoid > input(y, x-1)) << 14) |
					// Center
					((centoid > input(y, x+1)) << 12) |
					((centoid > input(y, x+2)) << 11) |
					((centoid > input(y, x+3)) << 10) |
					((centoid > input(y, x+4)) << 9) |
					// Row 3
					((centoid > input(y+1, x-4)) << 8) |
					((centoid > input(y+1, x-3)) << 7) |
					((centoid > input(y+1, x-2)) << 6) |
					((centoid > input(y+1, x-1)) << 5) |
					((centoid > input(y+1, x)) << 4) |
					((centoid > input(y+1, x+1)) << 3) |
					((centoid > input(y+1, x+2)) << 2) |
					((centoid > input(y+1, x+3)) << 1) |
					(centoid > input(y+1, x+4));		
			}
	}
	
	template <>
	void Census::transform9x3<char>(const cv::Mat_<char>& input, cv::Mat_<unsigned int>* output);
	
	template<typename T>
	void Census::transform5x5(const cv::Mat_<T>& input, cv::Mat_<unsigned int>* output) {
		int maxX=input.cols-2, maxY=input.rows-2;

		//#pragma omp parallel for default(none) shared(input, output, maxX, maxY)
		for(int y=2; y<maxY; y++)
			for(int x=2; x<maxX; x++) {
				T centoid = input(y,x);
				/*int avg = 0;
				for(int y2=-2; y2<=2; y2++)
					for(int x2=-2; x2<=2; x2++)
						avg += input(y+y2, x+x2);
				avg /= (5*5);
				T centoid = avg;*/
				
				(*output)(y,x) =
					// Row 1
					((centoid > input(y-2, x-2)) << 24) |
					((centoid > input(y-2, x-1)) << 23) |
					((centoid > input(y-2, x)) << 22) |
					((centoid > input(y-2, x+1)) << 21) |
					((centoid > input(y-2, x+2)) << 20) |
					// Row 2
					((centoid > input(y-1, x-2)) << 19) |
					((centoid > input(y-1, x-1)) << 18) |
					((centoid > input(y-1, x)) << 17) |
					((centoid > input(y-1, x+1)) << 16) |
					((centoid > input(y-1, x+2)) <<15) |
					// Row 3
					((centoid > input(y, x-2)) << 14) |
					((centoid > input(y, x-1)) << 13) |
					((centoid > input(y, x)) << 12) |
					((centoid > input(y, x+1)) << 11) |
					((centoid > input(y, x+2)) << 10) |
					// Row 4
					((centoid > input(y+1, x-2)) << 9) |
					((centoid > input(y+1, x-1)) << 8) |
					((centoid > input(y+1, x)) << 7) |
					((centoid > input(y+1, x+1)) << 6) |
					((centoid > input(y+1, x+2)) << 5) |
					// Row 5
					((centoid > input(y+2, x-2)) << 4) |
					((centoid > input(y+2, x-1)) << 3) |
					((centoid > input(y+2, x)) << 2) |
					((centoid > input(y+2, x+1)) << 1) |
					(centoid > input(y+2, x+2));
			}
	}
	
	template <>
	void Census::transform5x5<char>(const cv::Mat_<char>& input, cv::Mat_<unsigned int>* output);
	
	template<typename T>
	void Census::transform8x8(const cv::Mat_<T>& input, cv::Mat_<unsigned long long>* output) {
		int maxX=input.cols-4, maxY=input.rows-4;

		//#pragma omp parallel for default(none) shared(input, output, maxX, maxY)
		for(int y=3; y<maxY; y++)
			for(int x=3; x<maxX; x++) {
				T centoid = input(y,x);
				/*int avg = 0;
				for(int y2=-3; y2<=4; y2++)
					for(int x2=-3; x2<=4; x2++)
						avg += input(y+y2, x+x2);
				avg /= (8*8);
				T centoid = avg;*/
					
				(*output)(y,x) =
					// Row 1
					(((unsigned long long)(centoid > input(y-3, x-3))) << 63) |
					(((unsigned long long)(centoid > input(y-3, x-2))) << 62) |
					(((unsigned long long)(centoid > input(y-3, x-1))) << 61) |
					(((unsigned long long)(centoid > input(y-3, x))) << 60) |
					(((unsigned long long)(centoid > input(y-3, x+1))) << 59) |
					(((unsigned long long)(centoid > input(y-3, x+2))) << 58) |
					(((unsigned long long)(centoid > input(y-3, x+3))) << 57) |
					(((unsigned long long)(centoid > input(y-3, x+4))) << 56) |
					
					// Row 2
					(((unsigned long long)(centoid > input(y-2, x-3))) << 55) |
					(((unsigned long long)(centoid > input(y-2, x-2))) << 54) |
					(((unsigned long long)(centoid > input(y-2, x-1))) << 53) |
					(((unsigned long long)(centoid > input(y-2, x))) << 52) |
					(((unsigned long long)(centoid > input(y-2, x+1))) << 51) |
					(((unsigned long long)(centoid > input(y-2, x+2))) << 50) |
					(((unsigned long long)(centoid > input(y-2, x+3))) << 49) |
					(((unsigned long long)(centoid > input(y-2, x+4))) << 48) |
					
					// Row 3
					(((unsigned long long)(centoid > input(y-1, x-3))) << 47) |
					(((unsigned long long)(centoid > input(y-1, x-2))) << 46) |
					(((unsigned long long)(centoid > input(y-1, x-1))) << 45) |
					(((unsigned long long)(centoid > input(y-1, x))) << 44) |
					(((unsigned long long)(centoid > input(y-1, x+1))) << 43) |
					(((unsigned long long)(centoid > input(y-1, x+2))) << 42) |
					(((unsigned long long)(centoid > input(y-1, x+3))) << 41) |
					(((unsigned long long)(centoid > input(y-1, x+4))) << 40) |
					
					// Row 4
					(((unsigned long long)(centoid > input(y, x-3))) << 39) |
					(((unsigned long long)(centoid > input(y, x-2))) << 38) |
					(((unsigned long long)(centoid > input(y, x-1))) << 37) |
					(((unsigned long long)(centoid > input(y, x))) << 36) |
					(((unsigned long long)(centoid > input(y, x+1))) << 35) |
					(((unsigned long long)(centoid > input(y, x+2))) << 34) |
					(((unsigned long long)(centoid > input(y, x+3))) << 33) |
					(((unsigned long long)(centoid > input(y, x+4))) << 32) |
					
					// Row 5
					(((unsigned long long)(centoid > input(y+1, x-3))) << 31) |
					(((unsigned long long)(centoid > input(y+1, x-2))) << 30) |
					(((unsigned long long)(centoid > input(y+1, x-1))) << 29) |
					(((unsigned long long)(centoid > input(y+1, x))) << 28) |
					(((unsigned long long)(centoid > input(y+1, x+1))) << 27) |
					(((unsigned long long)(centoid > input(y+1, x+2))) << 26) |
					(((unsigned long long)(centoid > input(y+1, x+3))) << 25) |
					(((unsigned long long)(centoid > input(y+1, x+4))) << 24) |
					
					// Row 6
					(((unsigned long long)(centoid > input(y+2, x-3))) << 23) |
					(((unsigned long long)(centoid > input(y+2, x-2))) << 22) |
					(((unsigned long long)(centoid > input(y+2, x-1))) << 21) |
					(((unsigned long long)(centoid > input(y+2, x))) << 20) |
					(((unsigned long long)(centoid > input(y+2, x+1))) << 19) |
					(((unsigned long long)(centoid > input(y+2, x+2))) << 18) |
					(((unsigned long long)(centoid > input(y+2, x+3))) << 17) |
					(((unsigned long long)(centoid > input(y+2, x+4))) << 16) |
					
					// Row 7
					(((unsigned long long)(centoid > input(y+3, x-3))) << 15) |
					(((unsigned long long)(centoid > input(y+3, x-2))) << 14) |
					(((unsigned long long)(centoid > input(y+3, x-1))) << 13) |
					(((unsigned long long)(centoid > input(y+3, x))) << 12) |
					(((unsigned long long)(centoid > input(y+3, x+1))) << 11) |
					(((unsigned long long)(centoid > input(y+3, x+2))) << 10) |
					(((unsigned long long)(centoid > input(y+3, x+3))) << 9) |
					(((unsigned long long)(centoid > input(y+3, x+4))) << 8) |
					
					// Row 8
					(((unsigned long long)(centoid > input(y+4, x-3))) << 7) |
					(((unsigned long long)(centoid > input(y+4, x-2))) << 6) |
					(((unsigned long long)(centoid > input(y+4, x-1))) << 5) |
					(((unsigned long long)(centoid > input(y+4, x))) << 4) |
					(((unsigned long long)(centoid > input(y+4, x+1))) << 3) |
					(((unsigned long long)(centoid > input(y+4, x+2))) << 2) |
					(((unsigned long long)(centoid > input(y+4, x+3))) << 1) |
					((unsigned long long)(centoid > input(y+4, x+4)));
			}
	}
	
	template<typename T>
	void Census::transform9x7(const cv::Mat_<T>& input, cv::Mat_<unsigned long long>* output) {
		int maxX=input.cols-4, maxY=input.rows-3;

		//#pragma omp parallel for default(none) shared(input, output, maxX, maxY)
		for(int y=4; y<maxY; y++)
			for(int x=3; x<maxX; x++) {
				//T centoid = input(y,x);
				int avg = 0;
				for(int y2=-4; y2<=4; y2++)
					for(int x2=-3; x2<=3; x2++)
						avg += input(y+y2, x+x2);
				avg /= (9*7);
				T centoid = avg;
					
				(*output)(y,x) =
					// Row 1
					(((unsigned long long)(centoid > input(y-3, x-4))) << 62) |
					(((unsigned long long)(centoid > input(y-3, x-3))) << 61) |
					(((unsigned long long)(centoid > input(y-3, x-2))) << 60) |
					(((unsigned long long)(centoid > input(y-3, x-1))) << 59) |
					(((unsigned long long)(centoid > input(y-3, x))) << 58) |
					(((unsigned long long)(centoid > input(y-3, x+1))) << 57) |
					(((unsigned long long)(centoid > input(y-3, x+2))) << 56) |
					(((unsigned long long)(centoid > input(y-3, x+3))) << 55) |
					(((unsigned long long)(centoid > input(y-3, x+4))) << 54) |
					
					// Row 2
					(((unsigned long long)(centoid > input(y-2, x-4))) << 53) |
					(((unsigned long long)(centoid > input(y-2, x-3))) << 52) |
					(((unsigned long long)(centoid > input(y-2, x-2))) << 51) |
					(((unsigned long long)(centoid > input(y-2, x-1))) << 50) |
					(((unsigned long long)(centoid > input(y-2, x))) << 49) |
					(((unsigned long long)(centoid > input(y-2, x+1))) << 48) |
					(((unsigned long long)(centoid > input(y-2, x+2))) << 47) |
					(((unsigned long long)(centoid > input(y-2, x+3))) << 46) |
					(((unsigned long long)(centoid > input(y-2, x+4))) << 45) |
					
					// Row 3
					(((unsigned long long)(centoid > input(y-1, x-4))) << 44) |
					(((unsigned long long)(centoid > input(y-1, x-3))) << 43) |
					(((unsigned long long)(centoid > input(y-1, x-2))) << 42) |
					(((unsigned long long)(centoid > input(y-1, x-1))) << 41) |
					(((unsigned long long)(centoid > input(y-1, x))) << 40) |
					(((unsigned long long)(centoid > input(y-1, x+1))) << 39) |
					(((unsigned long long)(centoid > input(y-1, x+2))) << 38) |
					(((unsigned long long)(centoid > input(y-1, x+3))) << 37) |
					(((unsigned long long)(centoid > input(y-1, x+4))) << 36) |
					
					// Row 4
					(((unsigned long long)(centoid > input(y, x-4))) << 35) |
					(((unsigned long long)(centoid > input(y, x-3))) << 34) |
					(((unsigned long long)(centoid > input(y, x-2))) << 33) |
					(((unsigned long long)(centoid > input(y, x-1))) << 32) |
					(((unsigned long long)(centoid > input(y, x))) << 31) |
					(((unsigned long long)(centoid > input(y, x+1))) << 30) |
					(((unsigned long long)(centoid > input(y, x+2))) << 29) |
					(((unsigned long long)(centoid > input(y, x+3))) << 28) |
					(((unsigned long long)(centoid > input(y, x+4))) << 27) |
					
					// Row 5
					(((unsigned long long)(centoid > input(y+1, x-4))) << 26) |
					(((unsigned long long)(centoid > input(y+1, x-3))) << 25) |
					(((unsigned long long)(centoid > input(y+1, x-2))) << 24) |
					(((unsigned long long)(centoid > input(y+1, x-1))) << 23) |
					(((unsigned long long)(centoid > input(y+1, x))) << 22) |
					(((unsigned long long)(centoid > input(y+1, x+1))) << 21) |
					(((unsigned long long)(centoid > input(y+1, x+2))) << 20) |
					(((unsigned long long)(centoid > input(y+1, x+3))) << 19) |
					(((unsigned long long)(centoid > input(y+1, x+4))) << 18) |
					
					// Row 6
					(((unsigned long long)(centoid > input(y+2, x-4))) << 17) |
					(((unsigned long long)(centoid > input(y+2, x-3))) << 16) |
					(((unsigned long long)(centoid > input(y+2, x-2))) << 15) |
					(((unsigned long long)(centoid > input(y+2, x-1))) << 14) |
					(((unsigned long long)(centoid > input(y+2, x))) << 13) |
					(((unsigned long long)(centoid > input(y+2, x+1))) << 12) |
					(((unsigned long long)(centoid > input(y+2, x+2))) << 11) |
					(((unsigned long long)(centoid > input(y+2, x+3))) << 10) |
					(((unsigned long long)(centoid > input(y+2, x+4))) << 9) |
					
					// Row 7
					(((unsigned long long)(centoid > input(y+3, x-4))) << 8) |
					(((unsigned long long)(centoid > input(y+3, x-3))) << 7) |
					(((unsigned long long)(centoid > input(y+3, x-2))) << 6) |
					(((unsigned long long)(centoid > input(y+3, x-1))) << 5) |
					(((unsigned long long)(centoid > input(y+3, x))) << 4) |
					(((unsigned long long)(centoid > input(y+3, x+1))) << 3) |
					(((unsigned long long)(centoid > input(y+3, x+2))) << 2) |
					(((unsigned long long)(centoid > input(y+3, x+3))) << 1) |
					(((unsigned long long)(centoid > input(y+3, x+4))));
			}
	}
}

#endif
