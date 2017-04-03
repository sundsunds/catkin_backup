#ifndef KS_COLORCODER_H
#define KS_COLORCODER_H

#include <opencv2/opencv.hpp>

namespace ks {
	// Performs a color coding for given values
	class ColorCoder {
	public:
		enum ColorScale {
			RAINBOW,
			BLUE_WHITE_RED
		};
	
		ColorCoder(float min, float max, bool shadowLess, bool shadowGreater, ColorScale scale = RAINBOW);
			
		// Color-codes a single value
		cv::Vec3b getColor(float val) {
			if(val<minVal) {
				if(shadowLess)
					return cv::Vec3b(0, 0, 0);
				else val = minVal;
			} else if(val > maxVal) {
				if(shadowGreater)
					return cv::Vec3b(0, 0, 0);
				else val = maxVal;
			}
			
			double relVal = (val - minVal)/(maxVal-minVal);
			return codeRelativeValue(relVal);
		}
		
		// Optimized method for unsigned chars
		cv::Vec3b getColor(unsigned char val) {
			return byteLookup[val];
		}
		
		// Color codes an entire image
		template <typename T>
		void codeImage(const cv::Mat_<T>& input, cv::Mat_<cv::Vec3b>& output) {
			for(int y = 0; y<input.rows; y++)
				for(int x = 0; x<input.cols; x++)
					output(y,x) = getColor((float)input(y,x));
		}
		
		// Creates a new image with a legend at the right border
		cv::Mat_<cv::Vec3b> createLegendBorder(unsigned int srcWidth, unsigned int srcHeight);
	
	private:
		float minVal;
		float maxVal;
		bool shadowLess;
		bool shadowGreater;
		ColorScale scale;
		// Lookup table for converting byte values
		cv::Vec3b byteLookup[256];
		
		cv::Vec3b codeRelativeValue(float i) {
			if(scale == RAINBOW)
				return codeRelativeValueRainbow(i);
			else return codeRelativeValueBlueWhiteRed(i);
		}
		
		cv::Vec3b codeRelativeValueRainbow(float i) {
			// Color coding is based on HSV conversion
			// See http://en.wikipedia.org/wiki/HSL_and_HSV
			
			// Also reduce the satturation by up to -1/div
			double div = 2.0;
			double s = i/div + (div-1)/div;
			s = std::max(0.0, std::min(1.0, s)); // clamp satturation to 0 - 1
			
			// We morph the value to shorten the green color region
			double reducedStart = 0, reducedEnd = 0.75;
			double factor = 2.1;
			double scaling = 1.0 - 1.0/factor*(reducedEnd - reducedStart);
			
			i = i*scaling;
			double reducedMax = reducedStart + 1.0/factor*(reducedEnd - reducedStart);
			if(i > reducedStart && i <= reducedMax)
				i = reducedStart + factor*(i - reducedStart);
			else if( i> reducedMax)
				i += (factor-1)/factor*(reducedEnd - reducedStart);
			
			double v = 1.0;
		
			// Adjust hue
			double h = 360 - i * 360.0; //hue
			h = std::max(0.0, std::min(360.0, h)); // clamp hue to 0 - 360
			
			// Convert to RGB
			double c = v*s; //chroma				
			double hp = h/ (360.0/4.0); //H'
			double k = c*(1.0 - fabs(fmod(hp, 2.0) - 1.0));
			
			cv::Vec3f col;
			if(hp <= 1)
				col = cv::Vec3f(0, k, c);
			else if(hp <= 2)
				col = cv::Vec3f(0, c, k);
			else if(hp <= 3)
				col = cv::Vec3f(k, c, 0);
			else col = cv::Vec3f(c, k, 0);
			
			double m = v - c;
			return cv::Vec3b(255*(col[0] + m),
				255*(col[1] + m), 255*(col[2] + m));
		}
		
		cv::Vec3b codeRelativeValueBlueWhiteRed(float i) {
			// See Moreland, K.: Divergin Color Maps for Scientific Visualization
			static const unsigned char lookup[][3] = {
				{59, 76, 192},
				{68, 90, 204},
				{77, 104, 215},
				{87, 117, 225},
				{98, 130, 234},
				{108, 142, 241},
				{119, 154, 247},
				{130, 165, 251},
				{141, 176, 254},
				{152, 185, 255},
				{163, 194, 255},
				{174, 201, 253},
				{184, 208, 249},
				{194, 213, 244},
				{204, 217, 238},
				{213, 219, 230},
				{221, 221, 221},
				{229, 216, 209},
				{236, 211, 197},
				{241, 204, 185},
				{245, 196, 173},
				{247, 187, 160},
				{247, 177, 148},
				{247, 166, 135},
				{244, 154, 123},
				{241, 141, 111},
				{236, 127, 99},
				{229, 112, 88},
				{222, 96, 77},
				{213, 80, 66},
				{203, 62, 56},
				{192, 40, 47},
				{180, 4, 38}
			};
			static const int tableSize = 33;
		
			int min = std::min(tableSize-1, std::max(0, int(i * tableSize)));
			int max = std::min(tableSize-1, std::max(0, int(i * tableSize)+1));
			
			double f2 = (i*tableSize) - min;
			double f1 = 1.0 - f2;
			return cv::Vec3b((unsigned char)(f1 * lookup[min][2] + f2 * lookup[max][2] + 0.5),
				(unsigned char)(f1 * lookup[min][1] + f2 * lookup[max][1] + 0.5),
				(unsigned char)(f1 * lookup[min][0] + f2 * lookup[max][0] + 0.5));
		}
	};
	
	// Optimized method for byte images
	template <>
	void ColorCoder::codeImage(const cv::Mat_<unsigned char>& input, cv::Mat_<cv::Vec3b>& output);
}

#endif
