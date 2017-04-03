#include  "libks/imageproc/colorcoder.h"
#include <cstdio>
#include <string>

namespace ks {
	using namespace cv;
	using namespace std;

	ColorCoder::ColorCoder(float min, float max, bool shadowLess, bool shadowGreater, ColorScale scale)
		:minVal(min), maxVal(max), shadowLess(shadowLess), shadowGreater(shadowGreater), scale(scale)
	{
		for(int i=0; i<256; i++)
			byteLookup[i] = getColor((float)i);
		
		if(shadowLess && min == 0)
			byteLookup[255] = Vec3b(0,0,0);
	}
	
	template <>
	void ColorCoder::codeImage(const cv::Mat_<unsigned char>& input, cv::Mat_<cv::Vec3b> &output) {
		for(int y = 0; y<input.rows; y++)
			for(int x = 0; x<input.cols; x++)
				output(y,x) = getColor(input(y,x));
	}
	
	Mat_<Vec3b> ColorCoder::createLegendBorder(unsigned int srcWidth, unsigned int srcHeight) {
		int gap = 2;
	
		// Find the number of decimal digits to print
		int maxDecimals = 0, minDecimals = 0;
		float minBuf = minVal, maxBuf = maxVal;
		for(int i=0; i<4; i++)
		{
			if(minBuf - int(minBuf) != 0)
				minDecimals++;
			if(maxBuf - int(maxBuf) != 0)
				maxDecimals++;
			minBuf*=10.0;
			maxBuf*=10.0;	
		}
	
		// Create label strings
		char minStr[10], maxStr[10];
		snprintf(minStr, sizeof(minStr), "%.*f", minDecimals, minVal);
		snprintf(maxStr, sizeof(minStr), "%.*f", maxDecimals, maxVal);
		
		// Find label size	
		int baseline;
		Size fontSize = getTextSize(strlen(maxStr) > strlen(minStr) ? maxStr : minStr,
			CV_FONT_HERSHEY_TRIPLEX, 0.9, 4.0, &baseline);
		
		// Draw color gradient
		Mat_<Vec3b> dst(srcHeight, srcWidth + gap + fontSize.width, Vec3b(0, 0, 0));
		for(int y = 0; y< dst.rows; y++)
			for(int x = srcWidth + gap; x < dst.cols; x++) {
				dst(y, x) = codeRelativeValue(y/double(srcHeight));
		}
		
		// Print labels
		putText(dst, maxStr,
			cvPoint(srcWidth + gap, dst.rows - baseline),
			CV_FONT_HERSHEY_TRIPLEX, 0.9, /*font scale*/
			(Scalar)Vec3b(255, 255, 255), 4 /*thickness*/, CV_AA);
		putText(dst, maxStr,
			cvPoint(srcWidth + gap, dst.rows - baseline),
			CV_FONT_HERSHEY_TRIPLEX, 0.9, /*font scale*/
			(Scalar)Vec3b(0, 0, 0), 1 /*thickness*/, CV_AA);
		putText(dst, minStr,
			cvPoint(srcWidth + gap, fontSize.height + baseline),
			CV_FONT_HERSHEY_TRIPLEX, 0.9, /*font scale*/
			(Scalar)Vec3b(255, 255, 255), 4 /*thickness*/, CV_AA);
		putText(dst, minStr,
			cvPoint(srcWidth + gap, fontSize.height + baseline),
			CV_FONT_HERSHEY_TRIPLEX, 0.9, /*font scale*/
			(Scalar)Vec3b(0, 0, 0), 1 /*thickness*/, CV_AA);
		
		return dst;
	}
}