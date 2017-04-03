#include <cmath>
#include "libks/feature/ssefast.h"
#include "libks/feature/ssefast-inl.h"
#include "libks/base/simd.h"
#include "libks/base/exception.h"

namespace ks {
	using namespace cv;
	using namespace std;
	
	// Longest arc length lookup table
	unsigned char SSEFast::lookupTable[1U<<16];
	bool SSEFast::lookupInitialized = false;
	
	const v16qi SSEFast::const0(SIMD::scalar16NonLookup(0));
	const v16qi SSEFast::const128(SIMD::scalar16NonLookup(128));
	const v16qi SSEFast::const255(SIMD::scalar16NonLookup(255));
	const v16qi SSEFast::const0x01(SIMD::scalar16NonLookup(0x01));
	const v16qi SSEFast::const0x02(SIMD::scalar16NonLookup(0x02));
	const v16qi SSEFast::const0x04(SIMD::scalar16NonLookup(0x04));
	const v16qi SSEFast::const0x08(SIMD::scalar16NonLookup(0x08));
	const v16qi SSEFast::const0x10(SIMD::scalar16NonLookup(0x10));
	const v16qi SSEFast::const0x20(SIMD::scalar16NonLookup(0x20));
	const v16qi SSEFast::const0x40(SIMD::scalar16NonLookup(0x40));
	const v16qi SSEFast::const0x80(SIMD::scalar16NonLookup(0x80));
	
	SSEFast::SSEFast(unsigned char threshold, unsigned char minLength, bool nonmaxSuppression, int minBorder)
		: threshold(threshold), minLength(minLength), nonmaxSuppression(nonmaxSuppression)
	{
		if(!lookupInitialized) {
			// The first instance initializes the lookup table.
			// Should be threadsafe as we always write the same values and set the
			// initialization flag last.
			for(int i=0; i<=0xFFFF; i++)
				lookupTable[i] = findLongestArc(i) >= minLength ? 0xFF : 0;
			lookupInitialized = true;
		}
		
		border = max(minBorder, 3);
		
		corners.reserve(512);
		scores.reserve(512);
	}
	
	SSEFast::~SSEFast() {
	}

	inline unsigned char SSEFast::findLongestArc(unsigned short stripe) {
		int bestLength = 0;
		int startLength = -1;
		int currentLength = 0;
		
		// We iterate over all possible 16-bit permutations
		for(int i=1; i<0xFFFF; i = i<<1) {
			if(stripe & i)
				// Inside an arc segment
				currentLength++;
			else {
				// Outside an arc segment
				if(currentLength > bestLength)
					bestLength = currentLength;
				if(startLength == -1)
					startLength = currentLength;
				currentLength = 0;
			}
		}
		
		// wrap-around case
		if(startLength != -1)
			currentLength += startLength;
		
		// Handle last arc segment
		if(currentLength > bestLength)
			bestLength = currentLength;
		
		return bestLength;
	}
	
	void SSEFast::detectImpl(const cv::Mat& input, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask) {
		if(mask.data != NULL)
			throw Exception("Feature detection masks not supported!");
		
		detect(input, threshold, &keypoints, nonmaxSuppression);
	}
	
	void SSEFast::detect(const Mat_<char>& input, unsigned char threshold, vector<KeyPoint>* results, bool nonmaxSuppression) {
		
		if(input.type() != CV_8S)
			throw Exception("Image data has to be of type char!");
		
		// Create offsets for circle pixel
		initOffsets(input.step);
		
		// Detect corners
		corners.clear();
		vector<Point2i> corners;
		corners.reserve(512);
		detectCorners(input, threshold, &corners);
		
		if(nonmaxSuppression) {
			// Calculate score
			scores.resize(corners.size());
			calcScores(input, corners, threshold, minLength, &scores);
			
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(corners, scores, nonmaxPoints);
			
			// Copy results
			results->reserve(nonmaxPoints.size());
			for(unsigned int i=0; i<nonmaxPoints.size(); i++) {
				int index = nonmaxPoints[i];
				results->push_back(KeyPoint(Point2f(corners[index].x, corners[index].y), 6.f, -1.f, scores[index]));
			}
		} else {
			// Copy everything
			results->reserve(corners.size());
			for(unsigned int i = 0; i < corners.size(); i++ )
				results->push_back(KeyPoint(corners[i], 6.f, -1.f, -1000));
		}
	}
	
	void SSEFast::initOffsets(int step) {
		offsets[0] = step*-3 -1;
		offsets[1] = step*-3;
		offsets[2] = step*-3 +1;
		offsets[3] = step*-2 +2;
		offsets[4] = step*-1 +3;
		offsets[5] = +3;
		offsets[6] = step +3;
		offsets[7] = step*+2 +2;
		offsets[8] = step*+3 +1;
		offsets[9] = step*+3;
		offsets[10] = step*+3 -1;
		offsets[11] = step*+2 -2;
		offsets[12] = step*+1 -3; //Aligned
		offsets[13] = -3; //Aligned
		offsets[14] = step*-1 -3; //Aligned
		offsets[15] = step*-2 -2;
	}
	
	// Non-SSE implementation
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		for(int y = 3; y < input.rows -3; y++)
			for(int x = 3; x < input.cols -3; x++) {
				const char* center = &(input(y, x));
				
				// Load one circle
				char circle[16] = {*(center + offsets[0]), *(center + offsets[1]), *(center + offsets[2]), *(center + offsets[3]), 
					*(center + offsets[4]), *(center + offsets[5]), *(center + offsets[6]), *(center + offsets[7]), 
					*(center + offsets[8]), *(center + offsets[9]), *(center + offsets[10]), *(center + offsets[11]), 
					*(center + offsets[12]), *(center + offsets[13]), *(center + offsets[14]), *(center + offsets[15]) 
				};
				
				unsigned short brighter = 0, darker = 0;
				
				// Create bit-strings
				for(int i=0; i<16; i++) {
					brighter |= (circle[i] > *center + threshold) << i;
					darker |= (circle[i] < *center - threshold) << i;
				}
				
				// Perform lookup
				if(lookupTable[brighter] >= minLength || lookupTable[darker] >= minLength)
					results->push_back(Point2i(x, y));
			}
	}*/
	
	// Non-Interleaved SSE implementation
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi circles[16], comparison[16];
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
		
		for(int y = 3; y<input.rows-4; y++) // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
			
			const char* center = &input(y, x);
			// Load the 16 pixel for each of the 16 circles
			circles[0] = __builtin_ia32_loaddqu(center + offsets[0]);
			circles[1] = __builtin_ia32_loaddqu(center + offsets[1]);
			circles[2] = __builtin_ia32_loaddqu(center + offsets[2]);
			circles[3] = __builtin_ia32_loaddqu(center + offsets[3]);
			circles[4] = __builtin_ia32_loaddqu(center + offsets[4]);
			circles[5] = __builtin_ia32_loaddqu(center + offsets[5]);
			circles[6] = __builtin_ia32_loaddqu(center + offsets[6]);
			circles[7] = __builtin_ia32_loaddqu(center + offsets[7]);
			circles[8] = __builtin_ia32_loaddqu(center + offsets[8]);
			circles[9] = __builtin_ia32_loaddqu(center + offsets[9]);
			circles[10] = __builtin_ia32_loaddqu(center + offsets[10]);
			circles[11] = __builtin_ia32_loaddqu(center + offsets[11]);
			circles[12] = __builtin_ia32_loaddqu(center + offsets[12]);
			circles[13] = __builtin_ia32_loaddqu(center + offsets[13]);
			circles[14] = __builtin_ia32_loaddqu(center + offsets[14]);
			circles[15] = __builtin_ia32_loaddqu(center + offsets[15]);
						
			// Copy the 16 center values for each circle and add or substracts the threshold
			v16qi centersPlus = __builtin_ia32_loaddqu(center);
			v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
			centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);
			
			// For all circle compare each of the 16 pixel if its BRIGHTER than the center
			comparison[0] = __builtin_ia32_pcmpgtb128(circles[0], centersPlus) & const0x01;
			comparison[1] = __builtin_ia32_pcmpgtb128(circles[1], centersPlus) & const0x02;
			comparison[2] = __builtin_ia32_pcmpgtb128(circles[2], centersPlus) & const0x04;
			comparison[3] = __builtin_ia32_pcmpgtb128(circles[3], centersPlus) & const0x08;
			comparison[4] = __builtin_ia32_pcmpgtb128(circles[4], centersPlus) & const0x10;
			comparison[5] = __builtin_ia32_pcmpgtb128(circles[5], centersPlus) & const0x20;
			comparison[6] = __builtin_ia32_pcmpgtb128(circles[6], centersPlus) & const0x40;
			comparison[7] = __builtin_ia32_pcmpgtb128(circles[7], centersPlus) & const0x80;
			comparison[8] = __builtin_ia32_pcmpgtb128(circles[8], centersPlus) & const0x01;
			comparison[9] = __builtin_ia32_pcmpgtb128(circles[9], centersPlus) & const0x02;
			comparison[10] = __builtin_ia32_pcmpgtb128(circles[10], centersPlus) & const0x04;
			comparison[11] = __builtin_ia32_pcmpgtb128(circles[11], centersPlus) & const0x08;
			comparison[12] = __builtin_ia32_pcmpgtb128(circles[12], centersPlus) & const0x10;
			comparison[13] = __builtin_ia32_pcmpgtb128(circles[13], centersPlus) & const0x20;
			comparison[14] = __builtin_ia32_pcmpgtb128(circles[14], centersPlus) & const0x40;
			comparison[15] = __builtin_ia32_pcmpgtb128(circles[15], centersPlus) & const0x80;
			
			// Create 16-Bit Bitstrings
			v16qi byte1 = comparison[0] | comparison[1] | comparison[2] | comparison[3] | comparison[4] | comparison[5] | comparison[6] | comparison[7];
			v16qi byte2 = comparison[8] | comparison[9] | comparison[10] | comparison[11] | comparison[12] | comparison[13] | comparison[14] | comparison[15];
			v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1, byte2); //low
			v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1, byte2); //high
			
			// Lookup longest arcs
			v16qi longestBrighter = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// For all circle compare each of the 16 pixel if its DARKER than the center
			comparison[0] = __builtin_ia32_pcmpgtb128(centersMinus, circles[0]) & const0x01;
			comparison[1] = __builtin_ia32_pcmpgtb128(centersMinus, circles[1]) & const0x02;
			comparison[2] = __builtin_ia32_pcmpgtb128(centersMinus, circles[2]) & const0x04;
			comparison[3] = __builtin_ia32_pcmpgtb128(centersMinus, circles[3]) & const0x08;
			comparison[4] = __builtin_ia32_pcmpgtb128(centersMinus, circles[4]) & const0x10;
			comparison[5] = __builtin_ia32_pcmpgtb128(centersMinus, circles[5]) & const0x20;
			comparison[6] = __builtin_ia32_pcmpgtb128(centersMinus, circles[6]) & const0x40;
			comparison[7] = __builtin_ia32_pcmpgtb128(centersMinus, circles[7]) & const0x80;
			comparison[8] = __builtin_ia32_pcmpgtb128(centersMinus, circles[8]) & const0x01;
			comparison[9] = __builtin_ia32_pcmpgtb128(centersMinus, circles[9]) & const0x02;
			comparison[10] = __builtin_ia32_pcmpgtb128(centersMinus, circles[10]) & const0x04;
			comparison[11] = __builtin_ia32_pcmpgtb128(centersMinus, circles[11]) & const0x08;
			comparison[12] = __builtin_ia32_pcmpgtb128(centersMinus, circles[12]) & const0x10;
			comparison[13] = __builtin_ia32_pcmpgtb128(centersMinus, circles[13]) & const0x20;
			comparison[14] = __builtin_ia32_pcmpgtb128(centersMinus, circles[14]) & const0x40;
			comparison[15] = __builtin_ia32_pcmpgtb128(centersMinus, circles[15]) & const0x80;
			
			// Create 16-Bit Bitstrings
			byte1 = comparison[0] | comparison[1] | comparison[2] | comparison[3] | comparison[4] | comparison[5] | comparison[6] | comparison[7];
			byte2 = comparison[8] | comparison[9] | comparison[10] | comparison[11] | comparison[12] | comparison[13] | comparison[14] | comparison[15];
			bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1, byte2); //low
			bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1, byte2); //high
			
			// Lookup longest arcs
			v16qi longestDarker = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// Decide for longest arc
			v16qi longestArcs = __builtin_ia32_pmaxub128(longestDarker, longestBrighter);
			
			// Find arcs that are above maxlength
			int features = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(longestArcs, minLengthVec));
			
			// Store the results if any of the comparisons were true
			if(features != 0) {
				if(features & 0x001F) {
					if(features & 0x1)
						results->push_back(Point2i(x, y));
					if(features & 0x2)				
						results->push_back(Point2i(x+1, y));	
					if(features & 0x4)
						results->push_back(Point2i(x+2, y));
					if(features & 0x8)
						results->push_back(Point2i(x+3, y));
					if(features & 0x10)
						results->push_back(Point2i(x+4, y));
				}
				if(features & 0x03E0) {
					if(features & 0x20)
						results->push_back(Point2i(x+5, y));
					if(features & 0x40)
						results->push_back(Point2i(x+6, y));
					if(features & 0x80)					
						results->push_back(Point2i(x+7, y));
					if(features & 0x100)
						results->push_back(Point2i(x+8, y));
					if(features & 0x200)
						results->push_back(Point2i(x+9, y));
				}
				if(features & 0xFC00 && x+13 < input.cols) {
					if(features & 0x400)
						results->push_back(Point2i(x+10, y));
					if(features & 0x800)
						results->push_back(Point2i(x+11, y));
					if(features & 0x1000)
						results->push_back(Point2i(x+12, y));
					if(features & 0x2000)
						results->push_back(Point2i(x+13, y));
					if(features & 0x4000)
						results->push_back(Point2i(x+14, y));
					if(features & 0x8000)
						results->push_back(Point2i(x+15, y));
				}
			}
		}
	}*/
	
	 
	// CVPR Version: Interleaved SSE implementation
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
		
		for(int y = border; y<input.rows-border-1; y++) // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
			
			const char* center = &input(y, x);
			
			// Copy the 16 center values for each circle and add or substracts the threshold
			v16qi centersPlus = __builtin_ia32_loaddqu(center);
			v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
			centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

			v16qi byte1Plus, byte2Plus;
			v16qi byte1Minus, byte2Minus;
			
			// Load the 16 pixel for each of the 16 circles
			// For all circle compare each of the 16 pixel if its brighter/darker than the center
			v16qi pixel = __builtin_ia32_loaddqu(center + offsets[0]);
			byte1Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x01;
			byte1Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x01;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[1]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x02;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x02;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[2]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x04;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x04;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[3]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x08;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x08;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[4]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x10;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x10;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[5]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x20;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x20;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[6]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x40;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x40;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[7]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x80;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x80;
			
			
			pixel = __builtin_ia32_loaddqu(center + offsets[8]);
			byte2Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x01;
			byte2Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x01;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[9]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x02;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x02;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[10]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x04;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x04;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[11]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x08;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x08;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[12])); //Aligned!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x10;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x10;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[13])); //Aligned!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x20;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x20;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[14])); //Aligned!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x40;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x40;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[15]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & const0x80;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & const0x80;
			
			// Create 16-Bit Bitstrings
			v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Plus, byte2Plus); //low
			v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Plus, byte2Plus); //high
			
			// Lookup longest arcs
			v16qi longestBrighter = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// Create 16-Bit Bitstrings
			bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Minus, byte2Minus); //low
			bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Minus, byte2Minus); //high
			
			// Lookup longest arcs
			v16qi longestDarker = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// Decide for longest arc
			v16qi longestArcs = __builtin_ia32_pmaxub128(longestDarker, longestBrighter);
			
			// Find arcs that are above maxlength
			int features = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(longestArcs, minLengthVec));
			
			// Store the results if any of the comparisons were true
			if(features != 0) {
				if(features & 0x001F) {
					if(features & 0x1)
						results->push_back(Point2i(x, y));
					if(features & 0x2)				
						results->push_back(Point2i(x+1, y));	
					if(features & 0x4)
						results->push_back(Point2i(x+2, y));
					if(features & 0x8)
						results->push_back(Point2i(x+3, y));
					if(features & 0x10)
						results->push_back(Point2i(x+4, y));
				}
				if(features & 0x03E0) {
					if(features & 0x20)
						results->push_back(Point2i(x+5, y));
					if(features & 0x40)
						results->push_back(Point2i(x+6, y));
					if(features & 0x80)					
						results->push_back(Point2i(x+7, y));
					if(features & 0x100)
						results->push_back(Point2i(x+8, y));
					if(features & 0x200)
						results->push_back(Point2i(x+9, y));
				}
				if(features & 0xFC00 && x+13 < input.cols) {
					if(features & 0x400)
						results->push_back(Point2i(x+10, y));
					if(features & 0x800)
						results->push_back(Point2i(x+11, y));
					if(features & 0x1000)
						results->push_back(Point2i(x+12, y));
					if(features & 0x2000)
						results->push_back(Point2i(x+13, y));
					if(features & 0x4000)
						results->push_back(Point2i(x+14, y));
					if(features & 0x8000)
						results->push_back(Point2i(x+15, y));
				}
			}
		}
	}*/
	
	// Optimized without lookups
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
				
		for(int y = border; y<input.rows-border-1; y++) // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
			
			const char* center = &input(y, x);
			
			// Copy the 16 center values for each circle and add or substracts the threshold
			v16qi centersPlus = __builtin_ia32_loaddqu(center);
			v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
			centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

			v16qi byte1Plus, byte2Plus;
			v16qi byte1Minus, byte2Minus;
			
			// Load the 16 pixel for each of the 16 circles
			// For all circle compare each of the 16 pixel if its brighter/darker than the center
			v16qi pixel = __builtin_ia32_loaddqu(center + offsets[0]);
			v16qi bitConst = const0x01; // 0x01
			byte1Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[8]);
			byte2Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[9]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x02
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[1]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
						
			pixel = __builtin_ia32_loaddqu(center + offsets[2]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x04
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[10]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[11]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x08
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[3]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[4]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x10
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[12])); //Aliged!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[13])); //Aligned!
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x20
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[5]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[6]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x40
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[14])); //Aligned!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[15]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x80
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[7]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			// Evaluate brighter arcs
			
			// Create 16-bit string
			v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Plus, byte2Plus); //low
			// Zero out pixels with less than 8 neighbours
			v8hi rotated = SIMD::prol16(bitStr1);
			bitStr1 &= rotated; // 8
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 7
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 6
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 5
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 4
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 3
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 2
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 1
			// Convert elements to 0x00 or 0xFF
			bitStr1 = __builtin_ia32_pcmpeqw128(bitStr1, bitStr1^bitStr1);
		
			// Create 16-bit string
			v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Plus, byte2Plus); //high
			// Zero out pixels with less than 8 neighbours
			rotated = SIMD::prol16(bitStr2);
			bitStr2 &= rotated; // 8
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 7
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 6
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 5
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 4
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 3
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 2
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 1
			// Convert elements to 0x00 or 0xFF
			bitStr2 = __builtin_ia32_pcmpeqw128(bitStr2, bitStr2^bitStr2);
			
			unsigned int features = __builtin_ia32_pmovmskb128(__builtin_ia32_packsswb128(bitStr1, bitStr2));
			
			
			// Evaluate darker arcs
			
			// Create 16-bit string
			bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Minus, byte2Minus); //low
			// Zero out pixels with less than 8 neighbours
			rotated = SIMD::prol16(bitStr1);
			bitStr1 &= rotated; // 8
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 7
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 6
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 5
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 4
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 3
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 2
			rotated = SIMD::prol16(rotated);
			bitStr1 &= rotated; // 1
			// Convert elements to 0x00 or 0xFF
			bitStr1 = __builtin_ia32_pcmpeqw128(bitStr1, bitStr1^bitStr1);
		
			// Create 16-bit string
			bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Minus, byte2Minus); //high
			// Zero out pixels with less than 8 neighbours
			rotated = SIMD::prol16(bitStr2);
			bitStr2 &= rotated; // 8
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 7
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 6
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 5
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 4
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 3
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 2
			rotated = SIMD::prol16(rotated);
			bitStr2 &= rotated; // 1
			// Convert elements to 0x00 or 0xFF
			bitStr2 = __builtin_ia32_pcmpeqw128(bitStr2, bitStr2^bitStr2);
			
			features &= __builtin_ia32_pmovmskb128(__builtin_ia32_packsswb128(bitStr1, bitStr2));
			
			// Store the results if any of the comparisons were true
			if(features != 0xFFFF) {			
				if((features & 0x001F) != 0x001F) {
					if(!(features & 0x1))
						results->push_back(Point2i(x, y));
					if(!(features & 0x2))				
						results->push_back(Point2i(x+1, y));	
					if(!(features & 0x4))
						results->push_back(Point2i(x+2, y));
					if(!(features & 0x8))
						results->push_back(Point2i(x+3, y));
					if(!(features & 0x10))
						results->push_back(Point2i(x+4, y));
				}
				if((features & 0x03E0) != 0x03E0) {
					if(!(features & 0x20))
						results->push_back(Point2i(x+5, y));
					if(!(features & 0x40))
						results->push_back(Point2i(x+6, y));
					if(!(features & 0x80))					
						results->push_back(Point2i(x+7, y));
					if(!(features & 0x100))
						results->push_back(Point2i(x+8, y));
					if(!(features & 0x200))
						results->push_back(Point2i(x+9, y));
				}
				if((features & 0xFC00) != 0xFC00 && x+13 < input.cols) {
					if(!(features & 0x400))
						results->push_back(Point2i(x+10, y));
					if(!(features & 0x800))
						results->push_back(Point2i(x+11, y));
					if(!(features & 0x1000))
						results->push_back(Point2i(x+12, y));
					if(!(features & 0x2000))
						results->push_back(Point2i(x+13, y));
					if(!(features & 0x4000))
						results->push_back(Point2i(x+14, y));
					if(!(features & 0x8000))
						results->push_back(Point2i(x+15, y));
				}
			}
		}
	}

	// ECCV Version
	void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
				
		for(int y = border; y<input.rows-border-1; y++) // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
			
			const char* center = &input(y, x);
			
			// Copy the 16 center values for each circle and add or substracts the threshold
			v16qi centersPlus = __builtin_ia32_loaddqu(center);
			v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
			centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

			v16qi byte1Plus, byte2Plus;
			v16qi byte1Minus, byte2Minus;
			
			// Load the 16 pixel for each of the 16 circles
			// For all circle compare each of the 16 pixel if its brighter/darker than the center
			v16qi pixel = __builtin_ia32_loaddqu(center + offsets[0]);
			v16qi bitConst = const0x01; // 0x01
			byte1Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[8]);
			byte2Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[9]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x02
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[1]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
						
			pixel = __builtin_ia32_loaddqu(center + offsets[2]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x04
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[10]);
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[11]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x08
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[3]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[4]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x10
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[12])); //Aliged!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[13])); //Aligned!
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x20
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[5]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[6]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x40
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[14])); //Aligned!
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[15]);
			bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x80
			byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			pixel = __builtin_ia32_loaddqu(center + offsets[7]);
			byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
			byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
			
			// Create 16-Bit Bitstrings
			v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Plus, byte2Plus); //low
			v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Plus, byte2Plus); //high
			
			// Lookup longest arcs
			v16qi longestBrighter = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// Create 16-Bit Bitstrings
			bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Minus, byte2Minus); //low
			bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Minus, byte2Minus); //high
			
			// Lookup longest arcs
			v16qi longestDarker = {
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
				lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
			};
			
			// Decide for longest arc
			v16qi longestArcs = __builtin_ia32_pmaxub128(longestDarker, longestBrighter);
			
			// Find arcs that are above maxlength
			int features = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(longestArcs, minLengthVec));
			
			// Store the results if any of the comparisons were true
			if(features != 0) {
				if(features & 0x001F) {
					if(features & 0x1)
						results->push_back(Point2i(x, y));
					if(features & 0x2)				
						results->push_back(Point2i(x+1, y));	
					if(features & 0x4)
						results->push_back(Point2i(x+2, y));
					if(features & 0x8)
						results->push_back(Point2i(x+3, y));
					if(features & 0x10)
						results->push_back(Point2i(x+4, y));
				}
				if(features & 0x03E0) {
					if(features & 0x20)
						results->push_back(Point2i(x+5, y));
					if(features & 0x40)
						results->push_back(Point2i(x+6, y));
					if(features & 0x80)					
						results->push_back(Point2i(x+7, y));
					if(features & 0x100)
						results->push_back(Point2i(x+8, y));
					if(features & 0x200)
						results->push_back(Point2i(x+9, y));
				}
				if(features & 0xFC00 && x+13 < input.cols) {
					if(features & 0x400)
						results->push_back(Point2i(x+10, y));
					if(features & 0x800)
						results->push_back(Point2i(x+11, y));
					if(features & 0x1000)
						results->push_back(Point2i(x+12, y));
					if(features & 0x2000)
						results->push_back(Point2i(x+13, y));
					if(features & 0x4000)
						results->push_back(Point2i(x+14, y));
					if(features & 0x8000)
						results->push_back(Point2i(x+15, y));
				}
			}
		}
	}
*/
	
	void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
		if(input.type() != CV_8S)
			throw Exception("Image data has to be of type char!");
		
		// Create offsets for circle pixel
		initOffsets(input.step);
		
		for(int y = border; y<input.rows-border-1; y++) { // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
				
				const char* center = &input(y, x);
				
				// Copy the 16 center values for each circle and add or substract the threshold
				v16qi centersPlus = __builtin_ia32_loaddqu(center);
				v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
				centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

				v16qi byte1Plus, byte2Plus;
				v16qi byte1Minus, byte2Minus;
				
				// Load the 16 pixel for each of the 16 circles
				// For all circle compare each of the 16 pixel if its brighter/darker than the center
				// and store the comparison result in a bit string
				v16qi bitConst = const0x01; // 0x01
				v16qi pixel = __builtin_ia32_loaddqu(center + offsets[0]);
				byte1Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[8]);
				byte2Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				// Test for early rejection
				if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
					byte1Plus | byte1Minus | byte2Plus | byte2Minus, bitConst)) == 0)
					continue;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x02
				pixel = __builtin_ia32_loaddqu(center + offsets[9]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[1]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x04
				pixel = __builtin_ia32_loaddqu(center + offsets[2]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[10]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;

				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x08			
				pixel = __builtin_ia32_loaddqu(center + offsets[11]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[3]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x10
				pixel = __builtin_ia32_loaddqu(center + offsets[4]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[12])); //Aliged!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x20
				
				// Test for early rejection
				if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
					byte1Plus | byte1Minus | byte2Plus | byte2Minus,
					bitConst + __builtin_ia32_pcmpeqb128(bitConst, bitConst) /*-1*/ )) == 0)
					continue;

				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[13])); //Aligned!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[5]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[6]);
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x40
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[14])); //Aligned!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x80
				pixel = __builtin_ia32_loaddqu(center + offsets[15]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[7]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				// Test for early rejection again
				//v16qi combined = v16qi(byte1Minus | byte1Plus | byte2Minus | byte2Plus);
				//if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
				//	combined, __builtin_ia32_pcmpeqb128(combined, combined) )) == 0) // == 0xFF
				//	continue;
				
				bool lookupBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(byte1Plus | byte2Plus, const255));
				bool lookupDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(byte1Minus | byte2Minus, const255));
				
				int features = 0;
				
				// Lookup brighter arcs
				if(lookupBrighter) {
					// Create 16-Bit Bitstrings
					v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Plus, byte2Plus); //low
					v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Plus, byte2Plus); //high
					
					v16qi brighterArcs = {
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
					};
					
					features = __builtin_ia32_pmovmskb128(brighterArcs);
				}
				
				// Lookup darker arcs
				if(lookupDarker) {
					// Create 16-Bit Bitstrings
					v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Minus, byte2Minus); //low
					v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Minus, byte2Minus); //high
					
					// Lookup longest arcs
					v16qi darkerArcs = {
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
						lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
						lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
					};
					
					features |= __builtin_ia32_pmovmskb128(darkerArcs);
				}
				
				// Store the results if any of the comparisons were true
				if(features != 0) {
					if(features & 0x001F) {
						if(features & 0x1)
							results->push_back(Point2i(x, y));
						if(features & 0x2)
							results->push_back(Point2i(x+1, y));
						if(features & 0x4)
							results->push_back(Point2i(x+2, y));
						if(features & 0x8)
							results->push_back(Point2i(x+3, y));
						if(features & 0x10)
							results->push_back(Point2i(x+4, y));
					}
					if(features & 0x03E0) {
						if(features & 0x20)
							results->push_back(Point2i(x+5, y));
						if(features & 0x40)
							results->push_back(Point2i(x+6, y));
						if(features & 0x80)
							results->push_back(Point2i(x+7, y));
						if(features & 0x100)
							results->push_back(Point2i(x+8, y));
						if(features & 0x200)
							results->push_back(Point2i(x+9, y));
					}
					if(features & 0xFC00 && x+13 < input.cols) {
						if(features & 0x400)
							results->push_back(Point2i(x+10, y));
						if(features & 0x800)
							results->push_back(Point2i(x+11, y));
						if(features & 0x1000)
							results->push_back(Point2i(x+12, y));
						if(features & 0x2000)
							results->push_back(Point2i(x+13, y));
						if(features & 0x4000)
							results->push_back(Point2i(x+14, y));
						if(features & 0x8000)
							results->push_back(Point2i(x+15, y));
					}
				}
			}
		}
	}
	
	// Version with integrated nonmax suppression
	/*void SSEFast::detect(const Mat_<char>& input, unsigned char threshold, unsigned char minLength, vector<KeyPoint>* results, bool nonmaxSuppression) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
		if(input.type() != CV_8S)
			throw Exception("Image data has to be of type char!");
		
		// Create offsets for circle pixel
		initOffsets(input.step);
		
		// We need to keep scores for 3 rows
		int* scoresBuffer = new int[3*input.cols];
		memset(scoresBuffer, 0, sizeof(int)*3*input.cols);
		int* scoresCurr = scoresBuffer;
		int* scoresPrev = &scoresBuffer[input.cols];
		int* scoresPrevPrev = &scoresBuffer[2*input.cols];
		
		// Also keep the corners X locations for two rows
		int* cornersXBuffer = new int[2*input.cols];
		int* cornersXCurr = scoresBuffer;
		int* cornersXPrev = &scoresBuffer[input.cols];
		
		unsigned int numCorners = 0;
		unsigned int numCornersPrev = 0;
			
		for(int y = border; y<input.rows-border-1; y++) { // We skip the last image row to avoid range checking
			
			// Rotate score buffers
			int* tmp = scoresCurr;
			scoresCurr = scoresPrevPrev;
			scoresPrev = tmp;
			scoresPrevPrev = scoresPrev;
			
			memset(scoresCurr, 0, sizeof(int)*input.cols); // Initialize new scores buffer
			
			// Rotate corners X buffer
			tmp = cornersXCurr;
			cornersXCurr = cornersXPrev;
			cornersXPrev = cornersXCurr;
			numCornersPrev = numCorners;
			numCorners = 0;
			
			for(int x=3; x<input.cols-3; x+=16) {
				
				const char* center = &input(y, x);
				
				// Copy the 16 center values for each circle and add or substract the threshold
				v16qi centersPlus = __builtin_ia32_loaddqu(center);
				v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
				centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

				v16qi byte1Plus, byte2Plus;
				v16qi byte1Minus, byte2Minus;
				
				// Load the 16 pixel for each of the 16 circles
				// For all circle compare each of the 16 pixel if its brighter/darker than the center
				// and store the comparison result in a bit string
				v16qi bitConst = const0x01; // 0x01
				v16qi pixel = __builtin_ia32_loaddqu(center + offsets[0]);
				byte1Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[8]);
				byte2Plus = __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus = __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				// Test for early rejection
				if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
					byte1Plus | byte1Minus | byte2Plus | byte2Minus, bitConst)) == 0)
					continue;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x02
				pixel = __builtin_ia32_loaddqu(center + offsets[9]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[1]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x04
				pixel = __builtin_ia32_loaddqu(center + offsets[2]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[10]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;

				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x08			
				pixel = __builtin_ia32_loaddqu(center + offsets[11]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[3]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x10
				pixel = __builtin_ia32_loaddqu(center + offsets[4]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[12])); //Aliged!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x20
				
				// Test for early rejection
				if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
					byte1Plus | byte1Minus | byte2Plus | byte2Minus,
					bitConst + __builtin_ia32_pcmpeqb128(bitConst, bitConst) )) == 0) // + -1
					continue;

				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[13])); //Aligned!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[5]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[6]);
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x40
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = (v16qi)_mm_load_si128((__m128i*)(center + offsets[14])); //Aligned!
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				bitConst = __builtin_ia32_paddb128(bitConst, bitConst); // 0x80
				pixel = __builtin_ia32_loaddqu(center + offsets[15]);
				byte2Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte2Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				pixel = __builtin_ia32_loaddqu(center + offsets[7]);
				byte1Plus |= __builtin_ia32_pcmpgtb128(pixel, centersPlus) & bitConst;
				byte1Minus |= __builtin_ia32_pcmpgtb128(centersMinus, pixel) & bitConst;
				
				// Test for early rejection again
				v16qi combined = v16qi(byte1Minus | byte1Plus | byte2Minus | byte2Plus);
				if(__builtin_ia32_pmovmskb128(__builtin_ia32_pcmpeqb128(
					combined, __builtin_ia32_pcmpeqb128(combined, combined) )) == 0) // == 0xFF
					continue;
				
				// Create 16-Bit Bitstrings
				v8hi bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Plus, byte2Plus); //low
				v8hi bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Plus, byte2Plus); //high
				
				// Lookup longest arcs
				v16qi longestBrighter = {
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
				};
				
				// Create 16-Bit Bitstrings
				bitStr1 = (v8hi) __builtin_ia32_punpcklbw128(byte1Minus, byte2Minus); //low
				bitStr2 = (v8hi) __builtin_ia32_punpckhbw128(byte1Minus, byte2Minus); //high
				
				// Lookup longest arcs
				v16qi longestDarker = {
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 2)],
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 5)],
					lookupTable[(unsigned short)SIMD::element8(bitStr1, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr1, 7)], 
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 0)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 1)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 2)],
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 3)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 4)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 5)],
					lookupTable[(unsigned short)SIMD::element8(bitStr2, 6)], lookupTable[(unsigned short)SIMD::element8(bitStr2, 7)]
				};
				
				// Decide for longest arc
				v16qi longestArcs = __builtin_ia32_pmaxub128(longestDarker, longestBrighter);
				
				// Find arcs that are above maxlength
				int features = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(longestArcs, minLengthVec));
				
				// Store the results if any of the comparisons were true
				if(features != 0) {
					if(features & 0x001F) {
						if(features & 0x1) {
							cornersXCurr[numCorners++] = x;
							if(nonmaxSuppression)
								scoresCurr[x] = calcSingleScoreSSE(loadCircleSSE(input, x, y), SIMD::scalar16(input(y, x)), thresholdVec, minLength);
						}
						if(features & 0x2) {
							int cornerX = x+1;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x4) {
							int cornerX = x+2;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x8) {
							int cornerX = x+3;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x10) {
							int cornerX = x+4;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
					}
					if(features & 0x03E0) {
						if(features & 0x20) {
							int cornerX = x+5;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x40) {
							int cornerX = x+6;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x80) {
							int cornerX = x+7;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x100) {
							int cornerX = x+8;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x200) {
							int cornerX = x+9;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
					}
					if(features & 0xFC00 && x+13 < input.cols) {
						if(features & 0x400) {
							int cornerX = x+10;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x800) {
							int cornerX = x+11;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x1000) {
							int cornerX = x+12;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x2000) {
							int cornerX = x+13;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x4000) {
							int cornerX = x+14;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
						if(features & 0x8000) {
							int cornerX = x+15;
							cornersXCurr[numCorners++] = cornerX;
							if(nonmaxSuppression)
								scoresCurr[cornerX] = calcSingleScoreSSE(loadCircleSSE(input, cornerX, y), SIMD::scalar16(input(y, cornerX)), thresholdVec, minLength);
						}
					}
				}
			}
			
			// Store corners of the previous row
			for(unsigned int i=0; i<numCornersPrev; i++) {
				int x = cornersXPrev[i];
				int score = scoresPrev[x];
				if(!nonmaxSuppression || (
					score > scoresPrev[x-1] && score > scoresPrev[x+1] &&
					score > scoresPrevPrev[x] && score > scoresPrevPrev[x-1] && score > scoresPrevPrev[x+1] && 
					score > scoresCurr[x] && score > scoresCurr[x-1] && score > scoresCurr[x+1])) {
					
					results->push_back(KeyPoint(Point2f(x, y-1), 6.f, -1.f, score));
				}
			}
		}
		
		delete []scoresBuffer;
		delete []cornersXBuffer;
	}*/
	
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		v16qi minLengthVec = SIMD::scalar16(minLength-1);
		
		if(input.cols%16 != 0)
			throw Exception("Image width has to be a multiple of 16");
				
		for(int y = border; y<input.rows-border-1; y++) // We skip the last image row to avoid range checking
			for(int x=3; x<input.cols-3; x+=16) {
			
			const char* center = &input(y, x);
			
			// Copy the 16 center values for each circle and add or substracts the threshold
			v16qi centersPlus = __builtin_ia32_loaddqu(center);
			v16qi centersMinus = __builtin_ia32_psubsb128(centersPlus, thresholdVec);
			centersPlus = __builtin_ia32_paddsb128(centersPlus, thresholdVec);

			v16qi pixel, cmpRes, len0, len1, maxLen0, maxLen1, startLen0, startLen1;
			
			// Now process the 16 circle pixels one after another
			
			// Pixel 0
			// For the first pixel statLen is initialized with the comparison result
			// 0x00 or 0xFF, and len and maxLen with 0 or 1 depending on the comparison. 
			pixel = __builtin_ia32_loaddqu(center + offsets[0]);
			startLen0 = __builtin_ia32_pcmpgtb128(pixel, centersPlus);
			len0 = -startLen0;
			maxLen0 = len0;
			
			startLen1 = __builtin_ia32_pcmpgtb128(centersMinus, pixel);
			len1 = -startLen1;
			maxLen1 = len1;
			
			for(int i=1; i<16; i++) {
				pixel = __builtin_ia32_loaddqu(center + offsets[i]);
				cmpRes = __builtin_ia32_pcmpgtb128(pixel, centersPlus);
				len0 = cmpRes & (len0 - cmpRes);
				maxLen0 = __builtin_ia32_pmaxub128(maxLen0, len0);
				//startLen0 = (v16qi)__builtin_ia32_pmaxsb128(startLen0,
				//	(v16qi)__builtin_ia32_pand128((v2di)startLen0, (v2di)(len0 + cmpRes)));
					
				cmpRes = __builtin_ia32_pcmpgtb128(centersMinus, pixel);
				len1 = cmpRes & (len1 - cmpRes);
				maxLen1 = __builtin_ia32_pmaxub128(maxLen1, len1);
				//startLen1 = (v16qi)__builtin_ia32_pmaxsb128(startLen1,
				//	(v16qi)__builtin_ia32_pand128((v2di)startLen1, (v2di)(len1 + cmpRes)));
			}
			
			// Now we add the start length to the last length and see if we find
			// yet a longer arc length.
			
			//len0 = __builtin_ia32_pmaxsb128(len0, len0 + startLen0);
			//len1 = __builtin_ia32_pmaxsb128(len1, len1 + startLen1);
			//maxLen0 = __builtin_ia32_pmaxsb128(maxLen0, len0);
			//maxLen1 = __builtin_ia32_pmaxsb128(maxLen1, len1);
			
			// Decide for longest arc
			v16qi longestArcs = __builtin_ia32_pmaxub128(len0, len1);
			
			// Find arcs that are above the minimum length
			int features = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(longestArcs, minLengthVec));
			
			// Store the results if any of the comparisons were true
			if(features != 0) {
				if(features & 0x001F) {
					if(features & 0x1)
						results->push_back(Point2i(x, y));
					if(features & 0x2)				
						results->push_back(Point2i(x+1, y));	
					if(features & 0x4)
						results->push_back(Point2i(x+2, y));
					if(features & 0x8)
						results->push_back(Point2i(x+3, y));
					if(features & 0x10)
						results->push_back(Point2i(x+4, y));
				}
				if(features & 0x03E0) {
					if(features & 0x20)
						results->push_back(Point2i(x+5, y));
					if(features & 0x40)
						results->push_back(Point2i(x+6, y));
					if(features & 0x80)					
						results->push_back(Point2i(x+7, y));
					if(features & 0x100)
						results->push_back(Point2i(x+8, y));
					if(features & 0x200)
						results->push_back(Point2i(x+9, y));
				}
				if(features & 0xFC00 && x+13 < input.cols) {
					if(features & 0x400)
						results->push_back(Point2i(x+10, y));
					if(features & 0x800)
						results->push_back(Point2i(x+11, y));
					if(features & 0x1000)
						results->push_back(Point2i(x+12, y));
					if(features & 0x2000)
						results->push_back(Point2i(x+13, y));
					if(features & 0x4000)
						results->push_back(Point2i(x+14, y));
					if(features & 0x8000)
						results->push_back(Point2i(x+15, y));
				}
			}
		}
	}*/
	
	// Alternative implementation presented in the paper. This version is very slow and should not be used!
	/*void SSEFast::detectCorners(const Mat_<char> &input, unsigned char threshold, unsigned char minLength, vector<Point2i>* results) {
		v16qi thresholdVec = SIMD::scalar16(threshold);
		
		for(int y = 3; y < input.rows -3; y++)
			for(int x = 3; x < input.cols -3; x++) {
				const char* center = &(input(y, x));
				
				// Load one circle
				v16qi circle = {*(center + offsets[0]), *(center + offsets[1]), *(center + offsets[2]), *(center + offsets[3]), 
					*(center + offsets[4]), *(center + offsets[5]), *(center + offsets[6]), *(center + offsets[7]), 
					*(center + offsets[8]), *(center + offsets[9]), *(center + offsets[10]), *(center + offsets[11]), 
					*(center + offsets[12]), *(center + offsets[13]), *(center + offsets[14]), *(center + offsets[15]) 
				};
				
				// Find longest brighter arc
				v16qi centersMinus = SIMD::scalar16(*center);
				v16qi centersPlus = __builtin_ia32_paddsb128(centersMinus, thresholdVec);
				int arcBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(circle, centersPlus));
				
				if(lookupTable[arcBrighter] >= minLength)
					results->push_back(Point2i(x, y));
				else {
					// Find longest darker arc
					centersMinus = __builtin_ia32_psubsb128(centersMinus, thresholdVec);
					int arcDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(centersMinus, circle));
					if(lookupTable[arcDarker] >= minLength)
						results->push_back(Point2i(x, y));
				}
			}
	}*/
	
	void SSEFast::calcScores(const Mat_<char>& input, const vector<Point2i>& corners, int bstart, unsigned char minLength,
		vector<unsigned char>* scores) {
		const v16qi bstartVec = SIMD::scalar16(bstart);
		
		for(unsigned int i=0; i<corners.size(); i++) {
			int x = corners[i].x, y = corners[i].y;
		
			(*scores)[i] = calcSingleScoreSSE(loadCircleSSE(input, x, y), SIMD::scalar16(input(y, x)), bstartVec, minLength);
				//fast9.cornerScore(&input(y, x), input(y, x), bstart);
		}
	}
}
