#include <opencv2/opencv.hpp>
#include <libks/imageproc/colorcoder.h>
#include <iostream>

#define COLOR_STEPS	1024
#define THRESHOLD	10

using namespace ks;
using namespace cv;
using namespace std;

int main(int argc, char** argv) {
	if(argc != 4) {
		cout << "Usage: " << argv[0] << " input-file output-file max-disp" << endl;
		return -1;
	}
	
	ColorCoder rainbowCoder(0, COLOR_STEPS, false, false, ColorCoder::RAINBOW);
	ColorCoder blueRedCoder(0, COLOR_STEPS, false, false, ColorCoder::BLUE_WHITE_RED);
	
	Mat_<Vec3b> src = imread(argv[1]);
	if(src.data == NULL) {
		cerr << "Unable to open input image" << endl;
		return -1;
	} else if(src.channels() != 3) {
		cerr << "Input image is not a color image" << endl;
		return -1;
	}
	
	ColorCoder borderCoder(0, atof(argv[3]), false, false, ColorCoder::BLUE_WHITE_RED);
	Mat_<Vec3b> dst = borderCoder.createLegendBorder(640, 480);
	
	for(int y=0; y<480; y++)
		for(int x=0; x<640; x++) {
			int minDist = 255*3;
			int minIdx = 0;
			for(int i=0; i<COLOR_STEPS; i++) {
				Vec3b col = rainbowCoder.getColor((float)i);
				int dist = abs(col[0] - src(y,x)[0])
					+ abs(col[1] - src(y,x)[1])
					+ abs(col[2] - src(y,x)[2]);
				
				if(dist < minDist) {
					minDist = dist;
					minIdx = i;
				}
			}
			
			if(minDist < THRESHOLD)
				dst(y,x) = blueRedCoder.getColor((float)minIdx);
			else dst(y,x) = src(y,x);
		}
	
	imwrite(argv[2], dst);
	
	return 0;
}
