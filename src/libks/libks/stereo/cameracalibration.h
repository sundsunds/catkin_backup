#ifndef KS_CAMERACALIBRATION_H
#define KS_CAMERACALIBRATION_H

#include <vector>
#include <utility>
#include <opencv2/opencv.hpp>

namespace ks {
	// Stores the calibration results
	struct CalibrationResult {
		cv::Mat_<double> cameraMatrix[2], distCoeffs[2];
		cv::Mat_<double> R[2], P[2], Q, T, rotM;
		cv::Size imageSize;
		
		CalibrationResult() {
			cameraMatrix[0] = cv::Mat_<double>::eye(3, 3);
			cameraMatrix[1] = cv::Mat_<double>::eye(3, 3);
		}
        CalibrationResult(const char* file, double defaultApha = 0.5);
		
		void writeToFile(const char* file) const;
		void calcRectification(double alpha = -1);
	};

	// Class for performing stereo camera calibration
	class CameraCalibration {
	public:
		// Board size is number of internal corners; square size in meters
		CameraCalibration(cv::Size calibSize, double squareSize, bool aucklandBoard = false,
			int maxScale = 4);
	
		// Adds another image pair
		void addCalibrationPair(const std::pair<cv::Mat_<unsigned char>, cv::Mat_<unsigned char> >& pair,
			std::pair<cv::Mat_<cv::Vec3b>, cv::Mat_<cv::Vec3b> >* visualOutput = NULL) {
			addCalibrationPair(pair.first, pair.second,
				visualOutput != NULL ? &visualOutput->first : NULL,
				visualOutput != NULL ? &visualOutput->second : NULL);
		}
		void addCalibrationPair(const cv::Mat_<unsigned char>& left, const cv::Mat_<unsigned char> right,
			cv::Mat_<cv::Vec3b>* leftOutput = NULL, cv::Mat_<cv::Vec3b>* rightOutput = NULL);
			
		// Performs the actual calibration
		CalibrationResult calibrate();
	
	private:
		cv::Size calibSize;
		double squareSize;
		bool aucklandBoard;
		int maxScale;
		cv::Size imageSize;
	
		// Detected points in image space
		std::vector<std::vector<cv::Point2f> > leftImagePoints, rightImagePoints;
		
		// Finds the calibration pattern corners
		bool findCalibPatternCorners(const cv::Mat_<unsigned char>& image, std::vector<cv::Point2f>* corners,
			cv::Mat_<cv::Vec3b>* visualOutput);
		// Converts the image points to object space
		std::vector<cv::Point3f> getObjectPoints();
		// Finds the best size for subpixel window optimization
		int findSubPixWinSize(const std::vector<cv::Point2f>& corners);
		// Rotate the calibration pattern if wrongly aligned
		void rotateIfNeccessary(std::vector<cv::Point2f>* corners);
		// Calculates the average reprojection error for the given calibration
		double calcAvgReprojectionErr(const CalibrationResult& res, const cv::Mat_<double>& F);
	};
}

#endif
