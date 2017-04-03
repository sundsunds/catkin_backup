#ifndef KS_SSEFAST_INL_H
#define KS_SSEFAST_INL_H

#include "libks/feature/ssefast.h"
#include "libks/base/simd.h"

namespace ks {
	template <typename T>
	__always_inline v16qi SSEFast::loadCircleSSE(const cv::Mat_<T>& input, int x, int y) {
		const char* center = (const char*)&(input(y, x));
		
		v16qi circle = {*(center + offsets[0]), *(center + offsets[1]), *(center + offsets[2]), *(center + offsets[3]), 
			*(center + offsets[4]), *(center + offsets[5]), *(center + offsets[6]), *(center + offsets[7]), 
			*(center + offsets[8]), *(center + offsets[9]), *(center + offsets[10]), *(center + offsets[11]), 
			*(center + offsets[12]), *(center + offsets[13]), *(center + offsets[14]), *(center + offsets[15]) 
		}; // Why is this any faster???
		
		/*v16qi circle = {center[offsets[0]], center[offsets[1]], center[offsets[2]], center[offsets[3]], 
			center[offsets[4]], center[offsets[5]], center[offsets[6]], center[offsets[7]], 
			center[offsets[8]], center[offsets[9]], center[offsets[10]], center[offsets[11]], 
			center[offsets[12]], center[offsets[13]], center[offsets[14]], center[offsets[15]] 
		};*/
		
		return circle;
	}
	
	__always_inline unsigned char SSEFast::calcSingleScoreSSE(const v16qi& circle, v16qi center, const v16qi& bstartVec, unsigned char minLength) {
		v16qi bmin = bstartVec;
		v16qi bmax = const255;
		v16qi b = __builtin_ia32_pavgb128 (bmax, bmin);
		center += const128; 
		
		//Compute the score using binary search
		for(;;)
		{
			// Find brighter arc
			v16qi centerPlus = __builtin_ia32_paddusb128(center, b) - const128;
			int arcBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(circle, centerPlus));
			
			if(lookupTable[arcBrighter] >= minLength)
				bmin = b; // corner
			else {
				// Find darker arc
				v16qi centerMinus = __builtin_ia32_psubusb128(center, b) - const128;
				int arcDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(centerMinus, circle));
				if(lookupTable[arcDarker] >= minLength)
					bmin = b; // corner
				else
					bmax = b; // Not a corner
			}
			
			unsigned char singleBMin = SIMD::element16(bmin, 0), singleBMax = SIMD::element16(bmax, 0);
			if(singleBMin == singleBMax || singleBMin == singleBMax - 1)
				return (unsigned char)SIMD::element16(bmin, 0);
			
			// Update threshold	
			b = __builtin_ia32_pavgb128 (bmax, bmin);
		}
	}
	
	__always_inline bool SSEFast::detectSingleCornerSSE(const v16qi& circle, const v16qi& center, const v16qi& threshold,
		unsigned char minLength) {
		
		// Find longest brighter arc
		v16qi centersPlus = __builtin_ia32_paddsb128(center, threshold);
		int arcBrighter = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(circle, centersPlus));
				
		if(lookupTable[arcBrighter] >= minLength)
			return true;
		else {
			// Find longest darker arc
			v16qi centersMinus = __builtin_ia32_psubsb128(center, threshold);
			int arcDarker = __builtin_ia32_pmovmskb128(__builtin_ia32_pcmpgtb128(centersMinus, circle));
			if(lookupTable[arcDarker] >= minLength)
				return true;
		}
		
		return false;
	}
}

#endif
