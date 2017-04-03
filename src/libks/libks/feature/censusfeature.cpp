#include "libks/feature/censusfeature.h"
#include <iostream>

namespace ks {
	using namespace std;
	using namespace cv;
	
	// TODO: Fix the duplicated code
	
	void CensusFeature::compute5x5(const cv::Mat_<unsigned int>& input, Mat_<unsigned char>* output) {
		if(matchesBuffer.size() != input.size())
			matchesBuffer = Mat_<unsigned char>(input.size());
		
		if(output == NULL)
			output = &matchesBuffer;
	
		// Find census values matching our selection critereon
		for(int y=0; y<input.rows; y++) {
			for(int x=0; x<input.cols; x++) {
				int connected = connected5x5(input(y,x));
				(*output)(y, x) = connected;//*connected;
			}
		}

		// Extract features
		/*features->clear();
		for(int y=2; y<input.rows-2; y++)
			for(int x=2; x<input.cols-2; x++)
				if((*output)(y,x) > threshold)
					features->push_back(Point2i(x,y));*/
	}
	
	void CensusFeature::compute8x8(const cv::Mat_<unsigned long long>& input, Mat_<unsigned char>* output) {
		if(matchesBuffer.size() != input.size())
			matchesBuffer = Mat_<unsigned char>(input.size());
		
		if(output == NULL)
			output = &matchesBuffer;
	
		// Find census values matching our selection critereon
		for(int y=0; y<input.rows; y++) {
			for(int x=0; x<input.cols; x++) {
				int connected = connected8x8(input(y,x));
				(*output)(y, x) = connected;//*connected;
			}
		}
	}
	
	void CensusFeature::compute9x7(const cv::Mat_<unsigned long long>& input, Mat_<unsigned char>* output) {
		if(matchesBuffer.size() != input.size())
			matchesBuffer = Mat_<unsigned char>(input.size());
		
		if(output == NULL)
			output = &matchesBuffer;
	
		// Find census values matching our selection critereon
		for(int y=0; y<input.rows; y++) {
			for(int x=0; x<input.cols; x++) {
				int connected = connected9x7(input(y,x));
				(*output)(y, x) = connected;//*connected;
			}
		}
	}
	
	void CensusFeature::print5x5(unsigned int input) {
		unsigned int shiftedY = input;
		for(int y=0;y<5;y++) {
			unsigned int shiftedX = shiftedY;
			for(int x=0;x<5;x++) {
				cout << (int)((shiftedX & 0x80000000) > 0);
				shiftedX = shiftLeft5x5(shiftedX);
			}
			cout << endl;
			shiftedY = shiftUp5x5(shiftedY);
		}
		
		cout << endl;
	}
	
	void CensusFeature::print8x8(unsigned long long input) {
		cout << hex << input << dec << endl;
	
		unsigned long long shiftedY = input;
		for(int y=0;y<8;y++) {
			unsigned long long shiftedX = shiftedY;
			for(int x=0;x<8;x++) {
				cout << (int)((shiftedX & 0x8000000000000000LL) > 0);
				shiftedX = shiftLeft8x8(shiftedX);
			}
			cout << endl;
			shiftedY = shiftUp8x8(shiftedY);
		}
		
		cout << endl;
	}

}
