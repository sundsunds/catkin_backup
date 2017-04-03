#include "libks/feature/fakefast.h"
#include <cmath>

namespace ks {
	using namespace cv;

	void FakeFast::detect(const Mat_<unsigned char> &input, Mat_<unsigned char>* edgeMap,
			int threshold) {
		int offsets1[16], offsets2[16];
		makeOffsets(offsets1, offsets2, (int)input.step);
		
		for(int y=3; y<input.rows-3; y++)
			for(int x=3; x<input.cols-3; x++) {
			(*edgeMap)(y,x) = evalPixel(input, offsets1, offsets2, x, y, threshold);
			}
	}
	
	void FakeFast::makeOffsets(int pixel1[], int pixel2[], int row_stride)
	{
		// This method was copied from the OpenCV implemetation
		pixel1[0] = 0 + row_stride * 3;
		pixel1[1] = 1 + row_stride * 3;
		pixel1[2] = 2 + row_stride * 2;
		pixel1[3] = 3 + row_stride * 1;
		pixel1[4] = 3 + row_stride * 0;
		pixel1[5] = 3 + row_stride * -1;
		pixel1[6] = 2 + row_stride * -2;
		pixel1[7] = 1 + row_stride * -3;
		pixel1[8] = 0 + row_stride * -3;
		pixel1[9] = -1 + row_stride * -3;
		pixel1[10] = -2 + row_stride * -2;
		pixel1[11] = -3 + row_stride * -1; 
		pixel1[12] = -3 + row_stride * 0;
		pixel1[13] = -3 + row_stride * 1;
		pixel1[14] = -2 + row_stride * 2;
		pixel1[15] = -1 + row_stride * 3;
		
		/*pixel2[0] = 0 + row_stride * -1;
		pixel2[1] = 0 + row_stride * -1;
		pixel2[2] = -1 + row_stride * 0;
		pixel2[3] = -1 + row_stride * 0;
		pixel2[4] = -1 + row_stride * 0;
		pixel2[5] = -1 + row_stride * 0;
		pixel2[6] = 0 + row_stride * 1;
		pixel2[7] = 0 + row_stride * 1;
		pixel2[8] = 0 + row_stride * 1;
		pixel2[9] = 0 + row_stride * 1;
		pixel2[10] = 1 + row_stride * 0;
		pixel2[11] = 1 + row_stride * 0;
		pixel2[12] = 1 + row_stride * 0;
		pixel2[13] = 1 + row_stride * 0;
		pixel2[14] = 0 + row_stride * -1;
		pixel2[15] = 0 + row_stride * -1;*/
		
		for(int i=0; i<16;i++)
			pixel2[i] = 0; //pixel1[(i+8)%16];
	}
	
	inline unsigned char FakeFast::evalPixel(const Mat_<unsigned char>& input, int offsets1[], int offsets2[],
			int x, int y, int threshold) {	
			
		int bestLength = 0;
		int bestStart = 0;
		int startLength = -1;
		bool startGreater = false;
		bool startSmaller = false;
		int currentLength = 0;
		int currentStart = 0;
		
		bool greater = false, smaller = false;
		
		const unsigned char* pixel = &input(y,x);
		greater = (pixel[offsets1[0]] > pixel[offsets2[0]] + threshold);
		smaller = (pixel[offsets1[0]] < pixel[offsets2[0]] - threshold);
		
		for(int i=0; i<16; i++) {
			//if((greater && pixel[offsets[i]] > *pixel + threshold) ||
			//	(smaller && pixel[offsets[i]] < *pixel - threshold))	
			if((greater && pixel[offsets1[i]] > pixel[offsets2[i]] + threshold) ||
				(smaller && pixel[offsets1[i]] < pixel[offsets2[i]] - threshold))
				currentLength++;
			else {
				
				
				if(currentLength > bestLength) {
					bestLength = currentLength;
					bestStart = currentStart;
				}
				if(startLength == -1) {
					startLength = currentLength;
					startGreater = greater;
					startSmaller = smaller;
				}
				currentLength = 1;
				currentStart = i;
				greater = (pixel[offsets1[i]] > pixel[offsets2[i]] + threshold);
				smaller = (pixel[offsets1[i]] < pixel[offsets2[i]] - threshold);
			}
		}
		
		// wrap-around case
		if(greater == startGreater && startSmaller == smaller && startLength != -1 && (greater || smaller))
			currentLength += startLength;
		
		if(currentLength > bestLength) {
			bestLength = currentLength;
			bestStart = currentStart;
		}
		
		return bestLength;
		//return bestStart;
	}
}
