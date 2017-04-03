#include <iostream>
#include <iomanip>
#include "libks/feature/extendedfast.h"
#include "libks/feature/fast9-inl.h"
#include "libks/base/sdlwindow.h"
#include "libks/base/exception.h"
#include "libks/imageproc/bilinearinterpolation.h"
#include "libks/base/subpixelinterpolation.h"

//#define COLLECT_STATISTICS
#define ADAPTIVE_THRESHOLD
//#define ENFORCE_MIN_DISTANCE
//#define CENTRAL_VALUE(img, y, x) img(y, x) 
#define CENTRAL_VALUE(img, y, x) (((int)img(y, x) + (int)img(y-1, x) + (int)img(y+1, x) + (int)img(y, x-1) + (int)img(y, x+1))/5)

namespace ks {
	using namespace cv;
	
#ifdef COLLECT_STATISTICS
	unsigned int thresholdsHistogram[256] = {
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
		0,0,0,0,0,0};
		
	void printStatistics() {
		int last = 255;
		for(;thresholdsHistogram[last] == 0; last--){
		}
		
		unsigned int sum = 0;
		for(int i=0; i<=last; i++)
			sum += thresholdsHistogram[i];
		
		cout << "Extended FAST thresholds histogram: " << endl;
		for(int i=0; i<=last; i++)
			cout << i << "\t" << fixed << (thresholdsHistogram[i]/double(sum)*100) << endl;		
	}
#endif

	ExtendedFAST::ExtendedFAST(bool nonmaxSuppression, unsigned char minThreshold, float adaptivity, bool subpixelPrecision, int minBorder)
		: nonmaxSuppression(nonmaxSuppression), minThreshold(minThreshold), adaptivity(adaptivity),
		subpixelPrecision(subpixelPrecision) {
		  
		border = max(minBorder, subpixelPrecision ? 4 : 3); // For subpixel suppression we need a larger border
		
#ifdef COLLECT_STATISTICS
		static bool registered = false;
		if(!registered) {
			registered = true;
			atexit(printStatistics);
		}
#endif		  	
	}
	
	ExtendedFAST::~ExtendedFAST() {}

	void ExtendedFAST::detectImpl(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask) {
		if(mask.data != NULL)
			throw Exception("Feature detection masks not supported!");
		else if(image.type() != CV_8U)
			throw Exception("Image data has to be of type unsigned char!");
		
		fast9.setStep((int)image.step);
		
		// Clear previous results
		const int reserve = 512;
		corners.reserve(reserve); scores.reserve(reserve);
		
		// This code was adapted from cv::FAST
		fast9Detect(image, nonmaxSuppression);
	 
		if(nonmaxSuppression)
		{
			// Perform nonmax suppression
			vector<int> nonmaxPoints;
			fast9.nonMaxSuppression(corners, scores, nonmaxPoints);
			
#ifdef ENFORCE_MIN_DISTANCE
			vector<int> nonClosePoints;
			removeCloseFeatures(nonmaxPoints, &nonClosePoints);
			vector<int>& resultPoints = nonClosePoints;
#else
			vector<int>& resultPoints = nonmaxPoints;
#endif
			
			// Copy and optionally refine result
			keypoints.reserve(resultPoints.size());
			for(unsigned int i=0; i<resultPoints.size(); i++) {
				int index = resultPoints[i];
				Point2f pt = subpixelPrecision ? subpixelRefine(image, corners[index].x, corners[index].y, scores[index], true, true) :
					Point2f(corners[index].x, corners[index].y);
				keypoints.push_back(KeyPoint(pt, 6.f, -1.f, scores[index]));
			}
		}
		else
		{   
			// Copy everything
			size_t i, n = corners.size();
			keypoints.resize(n);
			for( i = 0; i < n; i++ )
				keypoints[i] = KeyPoint(corners[i], 6.f, -1.f, -1000);
		}
		
		// Clear buffers
		corners.clear(); scores.clear();
	}
	
	__always_inline unsigned char ExtendedFAST::getAdaptiveThreshold(const unsigned char* p) {
		// Standard deviation
		/*float avg = 0;
		for(int i=0; i<16; i++)
			avg += p[pixel[i]];
		avg /= 16.0;*/
		
		/*float var = 0;
		for(int i=0;i<16;i++)
			var += abs(p[pixel[i]] - c);
		return saturate_cast<unsigned char>(adaptivity * (var / 16.0));*/
		
		// Peak to peak contrast
		/*unsigned char min = p[pixel[0]], max = p[pixel[0]];
		for(int i=1; i<16; i++) {
			unsigned char val = p[pixel[i]];
			if(val > max)
				max = val;
			else if(val < min)
				min = val;
		}
		return std::min(maxThreshold, saturate_cast<unsigned char>((max-min)*adaptivity));*/
		
		// RMS or RMS-Like contrast	with floats
		/*float avg = 0;
		for(int i=0; i<16; i++)
			avg += p[fast9.pixel[i]];
		avg /= 16.0;
		
		float sad = 0;
		for(int i=0; i<16; i++)
			sad += fabs(p[fast9.pixel[i]] - avg);
			//sad += (p[pixel[i]] - avg) * (p[pixel[i]] - avg);*/
			
		// With integer precision		
		int sum = 0;
		for(int i=0; i<16; i++)
			sum += p[fast9.pixel[i]];
		int avg = sum/16;
		
		int sad = 0;
		for(int i=0; i<16; i++)
			sad += std::abs(p[fast9.pixel[i]] - avg);
		
		//return saturate_cast<unsigned char>(adaptivity * sqrt(sad));
		return saturate_cast</*unsigned*/ char>(sad * adaptivity / 16);
	}
	
	void ExtendedFAST::fast9Detect(const Mat_<unsigned char>& img, bool calculateScore)
	{
		int xsize = img.cols - border, ysize = img.rows - border;
		
		for(int y=border; y < ysize; y++)
			for(int x=border; x < xsize; x++) {
				const uchar* p = &img(y,x);
				// First test with a low threshold
				if(fast9.cornerTest(p, *p, minThreshold)) {
					int centralIntensity = CENTRAL_VALUE(img, y, x);
#ifdef ADAPTIVE_THRESHOLD
					// Then test again with an adaptive threshold
					unsigned char threshold = getAdaptiveThreshold(p);
					if(fast9.cornerTest(p, centralIntensity, threshold)) {
#else
					unsigned char threshold = minThreshold;	
					/*if*/ {
#endif
						corners.push_back(Point(x,y));
#ifdef COLLECT_STATISTICS
						thresholdsHistogram[threshold]++;
#endif
						if(calculateScore)
							// Compute score if we perform nonmax suppression
							scores.push_back(fast9.cornerScore(p, centralIntensity, threshold) - threshold);
					}
				}
			}
	}
	
	bool ExtendedFAST::testFeature(const Mat& image, Point2i pt) {
		// This code has to be kept consistent with fast9Detect()
		if(pt.y<3 || pt.y>= image.rows - 3 || pt.x<3 || pt.x>= image.cols-3)
			return false;
		
		fast9.setStep((int)image.step);
		const uchar* p = &image.at<unsigned char>(pt.y, pt.x);
		int centralIntensity = CENTRAL_VALUE(((Mat_<unsigned char>)image), pt.y, pt.x);
#ifdef ADAPTIVE_THRESHOLD
		unsigned char threshold = getAdaptiveThreshold(p);
#else
		unsigned char threshold = minThreshold;
#endif

		return fast9.cornerTest(p, centralIntensity, threshold);
	}
	
	void ExtendedFAST::subpixelRefine(const Mat_<unsigned char>& image, KeyPoint* keyPt, bool horizontal, bool vertical) {
		fast9.setStep((int)image.step);
		
		int x = int(keyPt->pt.x+0.5), y = int(keyPt->pt.y+0.5);
		if(y<4 || y>= image.rows - 4 || x<4 || x>= image.cols-4)
			return; // Too close to the border for refinement
		else keyPt->pt = subpixelRefine(image, x, y, (int)keyPt->response, horizontal, vertical);
	}
	
	Point2f ExtendedFAST::subpixelRefine(const Mat_<unsigned char>& image, int x, int y, int score, bool horizontal, bool vertical) {
		int c2 = score;
		int threshold = getAdaptiveThreshold(&image(y, x));
		
		if(c2 < -900) {
			int centralInt = CENTRAL_VALUE(image, y, x);
			c2 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
		}
	
		// Horizontal refinement
		float xOffset = 0;
		if(horizontal) {
			int centralInt = CENTRAL_VALUE(image, y, x-1);
			int c1 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			centralInt = CENTRAL_VALUE(image, y, x+1);
			int c3 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			xOffset = SubpixelInterpolation::maxOffset(c1, c2, c3, SubpixelInterpolation::QUADRATIC);
		}
		
		// Vertical refinement
		float yOffset = 0;
		if(vertical) {
			int centralInt = CENTRAL_VALUE(image, y-1, x);
			int c1 = fast9.cornerScore(&image(y-1, x), centralInt,  0) - threshold;
			centralInt = CENTRAL_VALUE(image, y, x);
			int c3 = fast9.cornerScore(&image(y, x), centralInt,  0) - threshold;
			yOffset = SubpixelInterpolation::maxOffset(c1, c2, c3, SubpixelInterpolation::QUADRATIC);
		}
		
		Point2f ret(x + xOffset, y + yOffset);
		//showDebugWindow(image, Point2i(x,y), -1, -1, Point2f(-1, -1), Point2f(-1, -1), ret);
		
		return ret;
	}
	
	/*void ExtendedFAST::showDebugWindow(const cv::Mat_<unsigned char>& image, cv::Point2i center, int arcStart, int arcEnd,
		Point2f subpixArcStart, Point2f subpixArcEnd, Point2f featurePos) {
		
		// Displays debugging information
		
		static bool initialized = false;
		if(!initialized)
			namedWindow("Debug");
		
		Mat_<Vec3b> debug(350, 700);
		BilinearInterpolation<unsigned char> interp(image);
		for(int y=0; y<350; y++)
			for(int x=0; x<350; x++) {
				float imgX = center.x + x/50.0 - 3;
				float imgY = center.y + y/50.0 - 3;
				unsigned char i1 = (unsigned char) interp(imgY, imgX);
				unsigned char i2 = image((int)imgY, (int)imgX);
				debug(y, x) = Vec3b(i1, i1, i1);
				debug(y, x+350) = Vec3b(i2, i2, i2);
			}
			
		rectangle(debug, Point2f(175-2, 175-2), Point2f(175+2, 175+2), (Scalar) Vec3b(0, 0, 255), CV_FILLED);
		rectangle(debug, Point2f(175-2 + 350, 175-2), Point2f(175+2 + 350, 175+2), (Scalar) Vec3b(0, 0, 255), CV_FILLED);
		for(int j=0; j<=15; j++) {
			int x = (pixelOffsetsX[j] + 3) * 50;
			int y = (pixelOffsetsY[j] + 3) * 50;
			rectangle(debug, Point2f(x, y), Point2f(x + 50, y + 50), (Scalar) Vec3b(0, 0, 255), 1);
			rectangle(debug, Point2f(x + 350, y), Point2f(x + 50 + 350, y + 50), (Scalar) Vec3b(0, 0, 255), 1);
		}
		
		if(arcStart >=0) {
			int startX = (pixelOffsetsX[arcStart] + 3) * 50 + 25;
			int startY = (pixelOffsetsY[arcStart] + 3) * 50 + 25;
			rectangle(debug, Point2f(startX-10, startY-10), Point2f(startX+10, startY+10), (Scalar) Vec3b(0, 255, 0), 1);
			rectangle(debug, Point2f(startX-10 + 350, startY-10), Point2f(startX+10 + 350, startY+10), (Scalar) Vec3b(0, 255, 0), 1);
		}
		if(arcEnd >=0) {
			int endX = (pixelOffsetsX[arcEnd] + 3) * 50 + 25;
			int endY = (pixelOffsetsY[arcEnd] + 3) * 50 + 25;
			rectangle(debug, Point2f(endX-10, endY-10), Point2f(endX+10, endY+10), (Scalar) Vec3b(255, 0, 0), 1);
			rectangle(debug, Point2f(endX-10 + 350, endY-10), Point2f(endX+10 + 350, endY+10), (Scalar) Vec3b(255, 0, 0), 1);
		}
		if(subpixArcStart != Point2f(-1, -1)) {
			int startX = (subpixArcStart.x - center.x + 3)*50 + 25;
			int startY = (subpixArcStart.y - center.y + 3)*50 + 25;
			
			rectangle(debug, Point2f(startX-2, startY-2), Point2f(startX+2, startY+2), (Scalar) Vec3b(0, 255, 0), CV_FILLED);
			rectangle(debug, Point2f(startX-2 + 350, startY-2), Point2f(startX+2 + 350, startY+2), (Scalar) Vec3b(0, 255, 0), CV_FILLED);
		}
		if(subpixArcEnd != Point2f(-1, -1)) {
			int endX = (subpixArcEnd.x - center.x + 3)*50 + 25;
			int endY = (subpixArcEnd.y - center.y + 3)*50 + 25;
			rectangle(debug, Point2f(endX-2, endY-2), Point2f(endX+2, endY+2), (Scalar) Vec3b(255, 0, 0), CV_FILLED);
			rectangle(debug, Point2f(endX-2 + 350, endY-2), Point2f(endX+2 + 350, endY+2), (Scalar) Vec3b(255, 0, 0), CV_FILLED);
		}
		
		int featX = (featurePos.x - center.x + 3)*50 + 25;
		int featY = (featurePos.y - center.y + 3)*50 + 25;
		rectangle(debug, Point2f(featX-2, featY-2), Point2f(featX+2, featY+2), (Scalar) Vec3b(0, 255, 255), CV_FILLED);
		
		imshow("Debug", debug);
		waitKey();
	}
	
	// Finds the circular arc start or end point with subpixel accuracy
	double ExtendedFAST::findSubpixelArcAngle(const Mat_<unsigned char>& image, Point2i center, unsigned char absThreshold, 
		int circleIndex, bool greater, bool start) {
		// Find the correct indices for the circular arc
		int index1, index2;
		if(start) {
			index1 = circleIndex - 1;
			if(index1 < 0)
				index1 = 15;
			index2 = circleIndex;
		} else {
			index1 = circleIndex;
			index2 = circleIndex + 1;
			if(index2 > 15)
				index2 = 0;
		}
		
		const double r = 3;
		
		// Find points angles from the center
		double beta1 = atan2(pixelOffsetsY[index1], pixelOffsetsX[index1]);
		double beta2 = atan2(pixelOffsetsY[index2], pixelOffsetsX[index2]);
		
		// Calculate interpolation constant
		//double i1 = image(center.y + pixelOffsetsY[index1], center.x + pixelOffsetsX[index1]);
		//double i2 = image(center.y + pixelOffsetsY[index2], center.x + pixelOffsetsX[index2]);
		
		BilinearInterpolation<unsigned char> interp(image);
		float i1 = interp(center.y + 0.5 + r*sin(beta1), center.x + r*cos(beta1));
		float i2 = interp(center.y + 0.5 + r*sin(beta2), center.x + r*cos(beta2));
	
		double c;
		if((greater && !start) || (!greater && start))
			c = (i1 - absThreshold) / (i1 - i2);
		else c = (absThreshold - i1) / (i2 - i1);
		
		// Find angle to interpolated point
		double betaDiff = beta2 - beta1;
		if(betaDiff > M_PI)
			betaDiff -= 2*M_PI;
		else if(betaDiff < -M_PI)
			betaDiff += 2*M_PI;
		double gamma = beta1 + betaDiff*c;
		
		// Find subpixel accurate angle
		Point2d ret(r * cos(gamma), r * sin(gamma));
		
		return gamma;
	}*/
	

	void ExtendedFAST::removeCloseFeatures(const std::vector<int>& features, std::vector<int>* result) {
		static const int minDist = adaptivity;
		result->clear();
		
		// TODO optimize if this method shows to be of any use
		for(unsigned int i=0; i<features.size(); i++) {
			bool skipPoint = false;
			for(unsigned int j=0; j<features.size(); j++) {
				if(i==j)
					continue;
					
				Point2i& p1 = corners[features[i]];
				Point2i& p2 = corners[features[j]];
				int dx = p1.x - p2.x;
				int dy = p1.y - p2.y;
				
				if(dx*dx + dy*dy < minDist*minDist && scores[features[i]] < scores[features[j]]) {
					skipPoint = true;
					break;
				}
			}
			
			if(!skipPoint)
				result->push_back(features[i]);
		}
	}
}
