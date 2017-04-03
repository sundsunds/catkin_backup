#include "libks/imageproc/noiserobustcensus.h"
#include "libks/imageproc/census-inl.h"
#include <algorithm>

namespace ks {
	using namespace cv;
	using namespace std;
	
	void NoiseRobustCensus::average9x3(Mat_<unsigned char>* input, Mat_<unsigned int>* output, bool preserveInput) {
		Mat_<unsigned char> avgImg;
		calcAvgImage<9, 3>(*input, &avgImg);
		
		Census::transform9x3(input, output, preserveInput, &avgImg);
	}
	
	void NoiseRobustCensus::median9x3(Mat_<unsigned char>* input, Mat_<unsigned int>* output, bool preserveInput) {
		Mat_<unsigned char> medImg;
		calcMedianImage<9, 3>(*input, &medImg);
		
		Census::transform9x3(input, output, preserveInput, &medImg);
	}
	 
	void NoiseRobustCensus::average5x5(Mat_<unsigned char>* input, Mat_<unsigned int>* output, bool preserveInput) {
		Mat_<unsigned char> avgImg;
		calcAvgImage<5, 5>(*input, &avgImg);
		Census::transform5x5(input, output, preserveInput, &avgImg);
	}
	
	void NoiseRobustCensus::median5x5(Mat_<unsigned char>* input, Mat_<unsigned int>* output, bool preserveInput) {
		Mat_<unsigned char> medImg;
		calcMedianImage<5, 5>(*input, &medImg);
		Census::transform5x5(input, output, preserveInput, &medImg);
	}
	
	template <int W, int H>
	void NoiseRobustCensus::calcAvgImage(const Mat_<unsigned char>& input, Mat_<unsigned char>* output) {
		Mat_<int> intImg(input.rows+1, input.cols+1);
		(*output) = Mat_<unsigned char>(input.rows, input.cols);
		
		integral(input, intImg, CV_32S);
		
		for(int y = H/2; y<input.rows-H/2; y++) {
			for(int x = W/2; x<input.cols-W/2; x++) {
				(*output)(y,x) = (intImg(y-H/2, x-W/2) + intImg(y+H/2+1, x+W/2+1)
					- intImg(y+H/2+1, x-W/2) - intImg(y-H/2, x+W/2+1)) / (W*H);
			}
		}
	}
	
	template <int W, int H>
	void NoiseRobustCensus::calcMedianImage(const Mat_<unsigned char>& input, Mat_<unsigned char>* output) {
		(*output) = Mat_<unsigned char>(input.rows, input.cols);
		
		unsigned char sortBuffer[W*H];
		
		for(int y = H/2; y<input.rows-H/2; y++)
			for(int x = W/2; x<input.cols-W/2; x++) {
			
				for(int dy=-H/2; dy <= H/2; dy++)
					for(int dx=-W/2; dx <= W/2; dx++)
						sortBuffer[(dy+H/2)*W + dx+W/2] = input(y+dy, x+dx);
						
				sort(sortBuffer, sortBuffer + (W*H));
				(*output)(y,x) = sortBuffer[W*H/2];
			}
	}
}
