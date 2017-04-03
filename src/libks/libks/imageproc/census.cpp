#include "libks/imageproc/census-inl.h"

namespace ks {
	using namespace std;
	using namespace cv;

	template <>
	void Census::transform9x3<char>(const Mat_<char>& input, Mat_<unsigned int>* output) {
		transform9x3SSE(input, output);
	}
	
	// Simpler but slightly slower implementation
	/*void Census::transform9x3SSE(const Mat_<char>& input, Mat_<unsigned int>* output) {
		for(int y=1; y<input.rows-1; y++) {
			for(int x=1; x<input.cols-1; x++) {
				// Create a vector for 13 comparisons to the center value
				char c = input(y, x);
				v16qi centoid = {c, c, c, c, c, c, c, c, c, c, c, c, c, 0, 0, 0}; 
				
				// Compare first set of pixels
				const char* row1Ptr = &input(y-1, x-4);
				const char* row2Ptr = row1Ptr + input.step;
				v16qi compSet1 = {*(row2Ptr+3), *(row2Ptr+2), *(row2Ptr+1), *row2Ptr,
					*(row1Ptr+8), *(row1Ptr+7), *(row1Ptr+6), *(row1Ptr+5), *(row1Ptr+4),
					*(row1Ptr+3), *(row1Ptr+2), *(row1Ptr+1), *row1Ptr, 1, 1, 1};
					
				v16qi compRes1 = __builtin_ia32_pcmpgtb128(centoid, compSet1);
				int census1 = __builtin_ia32_pmovmskb128(compRes1);
				
				// Compare second set of pixels
				row1Ptr = &input(y, x-4);
				row2Ptr = row1Ptr + input.step;
				v16qi compSet2 = {*(row2Ptr+8), *(row2Ptr+7), *(row2Ptr+6), *(row2Ptr+5),
					*(row2Ptr+4), *(row2Ptr+3), *(row2Ptr+2), *(row2Ptr+1), *row2Ptr,
					*(row1Ptr+8), *(row1Ptr+7), *(row1Ptr+6), *(row1Ptr+5), 1, 1, 1};
					
				v16qi compRes2 = __builtin_ia32_pcmpgtb128(centoid, compSet2);
				int census2 = __builtin_ia32_pmovmskb128(compRes2);

				(*output)(y,x) = (census1 << 14) | census2;
			}
		}
	}*/
	
	void Census::transform9x3SSE(const Mat_<char>& input, Mat_<unsigned int>* output) {
		// THIS CODE IS NO LONGER MAINTAINED! 5x5 transformation has been optimized further!
	
		// Predeclare required constants
		const v16qi const01 = SIMD::scalar16(0x01), const02 = SIMD::scalar16(0x02), const04 = SIMD::scalar16(0x04),
			const08 = SIMD::scalar16(0x08), const10 = SIMD::scalar16(0x10), const20 = SIMD::scalar16(0x20),
			const40 = SIMD::scalar16(0x40), const80 = SIMD::scalar16(0x80);
		
		// We skip one row at the beginning and end to avoid range checking
		//#pragma omp parallel for default(shared) private(censusBits)
		for(int y=2; y<input.rows-2; y++) {
		
			for(int x=0; x<input.cols; x+=16) {
				// Get 16 centoids
				v16qi centoid = (v16qi)_mm_load_si128((__m128i*)&(input)(y,x));
				
				// Row 1
				const char* rowPtr = &input(y-1, x-4);
				// Byte 4 starts
				v16qi byte4 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & const04;
				byte4 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & const02;
				byte4 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 2)) & const01;
				// Byte 3 starts
				v16qi byte3 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & const80;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 4))) & const40; //Aligned
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 5)) & const20;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 6)) & const10;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 7)) & const08;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 8)) & const04;
				
				// Row 2
				rowPtr += input.step;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & const02;
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & const01;
				// Byte 2 starts
				v16qi byte2 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 2)) & const80;
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & const40;
				// Center
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 5)) & const10;
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 6)) & const08;
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 7)) & const04;
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 8)) & const02;
				
				// Row 3
				rowPtr += input.step;
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & const01;
				// Byte 1 starts
				v16qi byte1 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & const80;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 2)) & const40;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & const20;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 4))) & const10; //Aligned
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 5)) & const08;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 6)) & const04;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 7)) & const02;
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 8)) & const01;
				
				storeSSEVec(byte4, byte3, byte2, byte1, (char*) &(*output)(y,x));
			}
		}
	}
	
	template <>
	void Census::transform5x5<char>(const Mat_<char>& input, Mat_<unsigned int>* output) {
		transform5x5SSE(input, output);
	}
	
	void Census::transform5x5SSE(const Mat_<char>& input, Mat_<unsigned int>* output) {
		// Predeclare required constants
		const v16qi const01 = SIMD::scalar16(0x01);

		// We skip one row at the beginning and end to avoid range checking
		//#pragma omp parallel for default(shared) shared(output) private(censusBits)
		for(int y=2; y<input.rows-2; y++)
			for(int x=0; x<input.cols; x+=16) {
				// Get 16 centoids
				v16qi centoid = (v16qi)_mm_load_si128((__m128i*)&(input)(y,x));
				
				// Row 5
				const char* rowPtr = &input(y+2, x-2);
				v16qi bitConst = const01;
				v16qi byte1 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x02
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x04
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Alinged
				bitConst += bitConst; // 0x08
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x10
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 4
				rowPtr -= input.step;
				bitConst += bitConst; // 0x20
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x40
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x80
				byte1 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				// Byte 2 starts
				bitConst = const01;
				v16qi byte2 = __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x02
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 3
				rowPtr -= input.step;
				bitConst += bitConst; // 0x04
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x08
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x10
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x20
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 2
				rowPtr -= input.step;
				bitConst += bitConst; // 0x40
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x80
				byte2 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				// Byte 3 starts
				bitConst = const01;
				v16qi byte3 = __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				bitConst += bitConst; // 0x02
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x04
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
				
				// Row 1
				rowPtr -= input.step;
				bitConst += bitConst; // 0x08
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 4)) & bitConst;
				bitConst += bitConst; // 0x10
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 3)) & bitConst;
				bitConst += bitConst; // 0x20
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, (v16qi)_mm_load_si128((__m128i*)(rowPtr + 2))) & bitConst; //Aligned
				bitConst += bitConst; // 0x40
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr + 1)) & bitConst;
				bitConst += bitConst; // 0x80
				byte3 |= __builtin_ia32_pcmpgtb128(centoid, __builtin_ia32_loaddqu(rowPtr)) & bitConst;
												
				storeSSEVec(byte3^byte3 /*0*/, byte3, byte2, byte1, (char*) &(*output)(y,x));
			}
	}
	
	__always_inline void Census::storeSSEVec(const v16qi& byte4, const v16qi& byte3, const v16qi& byte2, const v16qi& byte1, char* dst) {
		// Combine bytes to shorts
		v8hi high1 = (v8hi) __builtin_ia32_punpcklbw128(byte3, byte4);
		v8hi high2 = (v8hi) __builtin_ia32_punpckhbw128(byte3, byte4);
		v8hi low1 = (v8hi) __builtin_ia32_punpcklbw128(byte1, byte2);
		v8hi low2 = (v8hi) __builtin_ia32_punpckhbw128(byte1, byte2);
		
		// Combine shorts to ints
		__builtin_ia32_storedqu(dst, (v16qi)__builtin_ia32_punpcklwd128(low1, high1));
		__builtin_ia32_storedqu(dst + 4*4, (v16qi)__builtin_ia32_punpckhwd128(low1, high1));
		__builtin_ia32_storedqu(dst + 8*4, (v16qi)__builtin_ia32_punpcklwd128(low2, high2));
		__builtin_ia32_storedqu(dst + 12*4, (v16qi)__builtin_ia32_punpckhwd128(low2, high2));
	}
}
