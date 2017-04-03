#ifndef KS_PLANEESMESTIMATOR_H
#define KS_PLANEESMESTIMATOR_H

#include <opencv2/opencv.hpp>
#include "libks/stereo/cameracalibration.h"

namespace ks {
	// Estimates movement between two planes with ESM
	class PlaneESMEstimator {
	public:
		PlaneESMEstimator(const CalibrationResult& calib,
			int imageWidth, int imageHeight, double minDelta,
			int maxIter, double maxDelta);
		
		// Sets a new target image for the next ESM estimation
		void setTargetImage(const cv::Mat_<unsigned char>& img);
			
		// Performs ESM estimation
		bool estimate(double lastHeight, double currentHeight, double* dx, double* dy, double* dYaw);
			
		// Accessors for intermediate results
		cv::Mat_<unsigned char> getSmallRectified() const {return rectified;}
		cv::Mat_<unsigned char> getLastSmallRectified() const {return lastImage;}
		cv::Mat_<unsigned char> getSmallWarped();
		
	private:
		CalibrationResult calib;
		double minDelta;
		int maxIter;
		double maxDelta;
		bool frameSkipped, frameProcessed;
			
		cv::Mat_<unsigned char> lastImage;
		cv::Mat imageRectificationMap[2];
		cv::Mat_<double> rectifiedCameraMatrix;
		cv::Mat_<unsigned char> rectified;	
		cv::Mat_<double> lastH;
	};
}

#endif
