#ifndef KS_CENSUSFEATURE_H
#define KS_CENSUSFEATURE_H

#include <opencv2/opencv.hpp>
#include "libks/base/hammingdistance.h"

namespace ks {
	class CensusFeature {
	public:
		void compute5x5(const cv::Mat_<unsigned int>& input, cv::Mat_<unsigned char>* output = NULL);
		
		void compute8x8(const cv::Mat_<unsigned long long>& input, cv::Mat_<unsigned char>* output = NULL);
		
		void compute9x7(const cv::Mat_<unsigned long long>& input, cv::Mat_<unsigned char>* output = NULL);
		
	private:
		HammingDistance hammingDist;
		cv::Mat_<unsigned char> matchesBuffer;
		
		unsigned int shiftLeft5x5(unsigned int input) {
			return (input << 1) & 0x01EF7BDE /*0b1111011110111101111011110*/;
		}
		
		unsigned long long shiftLeft8x8(unsigned long long input) {
			return (input << 1) & 0xFEFEFEFEFEFEFEFELL;
		}
		
		unsigned long long shiftLeft9x7(unsigned long long input) {
			return (input << 1) & 0b0111111110111111110111111110111111110111111110111111110111111110LL;
		}
		
		unsigned int shiftRight5x5(int input) {
			return (input >> 1) & 0x00F7BDEF /*0b0111101111011110111101111*/;
		}
		
		unsigned int shiftUp5x5(unsigned int input) {
			return input<<5;
		}
		
		unsigned long long shiftUp8x8(unsigned long long input) {
			return input<<8;
		}
		
		unsigned long long shiftUp9x7(unsigned long long input) {
			return input<<9;
		}
		
		unsigned int shiftDown5x5(unsigned int input) {
			return (input>>5) & 0x000FFFFF /*0b0000011111111111111111111*/;
		}
		
		unsigned char horizontal5x5Trans(unsigned int input) {
			return hammingDist.countBits(input^shiftRight5x5(input));
		}
		
		unsigned char vertical5x5Trans(unsigned int input) {
			return hammingDist.countBits(input^shiftDown5x5(input));
		}
		
		unsigned char connected5x5(unsigned int input) {
			return std::min(count0Windows5x5(input), count1Windows5x5(input));
		}
		
		unsigned char connected8x8(unsigned long long input) {
			return std::min(count0Windows8x8(input), count1Windows8x8(input));
		}

		unsigned char connected9x7(unsigned long long input) {
			return std::min(count0Windows9x7(input), count1Windows9x7(input));
		}
		
		void print5x5(unsigned int input);
		void print8x8(unsigned long long input);
		
		unsigned char count0Windows5x5(unsigned int input) {
			return count1Windows5x5((~input) & 0x01FFFFFF /*0b1111111111111111111111111*/);
		}
		
		unsigned char count0Windows8x8(unsigned long long input) {
			return count1Windows8x8(~input);
		}

		unsigned char count0Windows9x7(unsigned long long input) {
			return count1Windows9x7(~input);
		}
		
		unsigned char count1Windows5x5(unsigned int input) {
			// Accumulate horizontally
			unsigned int h = input & shiftLeft5x5(input);
			// Accumulate vertically
			return hammingDist.countBits(h & shiftUp5x5(h));
		}
		
		unsigned char count1Windows8x8(unsigned long long input) {
			// Accumulate horizontally
			unsigned long long h = input & shiftLeft8x8(input);
			// Accumulate vertically
			return hammingDist.countBits(h & shiftUp8x8(h));
		}
		
		unsigned char count1Windows9x7(unsigned long long input) {
			// Accumulate horizontally
			unsigned long long h = input & shiftLeft9x7(input);
			// Accumulate vertically
			return hammingDist.countBits(h & shiftUp9x7(h));
		}
	};
}
#endif
