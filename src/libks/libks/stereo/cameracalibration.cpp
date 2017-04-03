#include "libks/stereo/cameracalibration.h"
#include <iostream>
#include <climits>
#include <cmath>
#include "libks/base/exception.h"

namespace ks {
	using namespace cv;
	using namespace std;
	
	CalibrationResult::CalibrationResult(const char* file, double defaultAlpha) {
		FileStorage fs(file, CV_STORAGE_READ);
		if(!fs.isOpened())
			throw Exception("Unable to read calibration results");
		
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs["R1"] >> R[0];
		fs["R2"] >> R[1];
		fs["P1"] >> P[0];
		fs["P2"] >> P[1];
		fs["Q"] >> Q;
		fs["T"] >> T;
		fs["R"] >> rotM;
		
		Mat_<int> sz(2, 1);
		fs["size"] >> sz;
		imageSize.width = sz(0, 0);
		imageSize.height = sz(1, 0);
		
		// If the rotation matrix is provided, we recompute the rectification
		if(rotM.rows != 0)
			calcRectification(defaultAlpha);
		
		fs.release();
	}

	void CalibrationResult::calcRectification(double alpha) {
		Rect validRoi[2];
		stereoRectify(cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1], imageSize, rotM, T,
			R[0], R[1], P[0], P[1], Q,
			// Parameter order has changed in OpenCV 2.3
#if CV_MAJOR_VERSION*100 + CV_MINOR_VERSION <= 202
			alpha, imageSize, &validRoi[0], &validRoi[1], CALIB_ZERO_DISPARITY);
#else
			CALIB_ZERO_DISPARITY, alpha, imageSize, &validRoi[0], &validRoi[1]);
#endif
	}
	
	void CalibrationResult::writeToFile(const char * file) const {
		FileStorage fs(file, CV_STORAGE_WRITE);
		if(!fs.isOpened())
			throw Exception("Unable to store calibration results");
		
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0]
			<< "M2" << cameraMatrix[1] << "D2" << distCoeffs[1]
			<< "R1" << R[0] << "R2" << R[1] << "P1" << P[0] << "P2" << P[1] << "Q" << Q
			<< "T" << T << "R" << rotM;
			
		Mat_<int> sz(2, 1);
		sz(0, 0) = imageSize.width;
		sz(1, 0) = imageSize.height;
		fs << "size" << sz;
		
		fs.release();
	}
	
	CameraCalibration::CameraCalibration(Size calibSize, double squareSize, bool aucklandBoard, int maxScale)
		:calibSize(calibSize), squareSize(squareSize), aucklandBoard(aucklandBoard), maxScale(maxScale) {
		if(aucklandBoard) {
			// Ignore border on auckland board
			this->calibSize.width -= 2;
			this->calibSize.height -= 2;
		}
	}
	
	void CameraCalibration::addCalibrationPair(const Mat_<unsigned char>& left, const Mat_<unsigned char> right,
		Mat_<Vec3b>* leftOutput, Mat_<Vec3b>* rightOutput) {
		
		if(imageSize == Size(0, 0))
			imageSize = left.size();
		if(left.size() != right.size() || imageSize != left.size())
			throw Exception("Images have unequal size");
		
		vector<Point2f> leftPoints, rightPoints;
		
		if(findCalibPatternCorners(left, &leftPoints, leftOutput)) {
			if(findCalibPatternCorners(right, &rightPoints, rightOutput)) {
				leftImagePoints.push_back(leftPoints);
				rightImagePoints.push_back(rightPoints);
			}
			else cerr << "Calibration pattern not found in right image" << endl;
		}
		else {
			cerr << "Calibration pattern not found in left image (right image skipped)" << endl;
			if(rightOutput != NULL)
				(*rightOutput) = Mat_<Vec3b>(right.size(), Vec3b(0, 0, 0));
		}
	}
	
	CalibrationResult CameraCalibration::calibrate() {
		if(leftImagePoints.size() < 2)
			throw Exception("Not enough valid image pairs for calibration");
		cout << "Calibration with " << leftImagePoints.size() << " image pairs" << endl;
	
		// Collect object points
		vector<vector<Point3f> > objectPoints;
		for(unsigned int i=0; i<leftImagePoints.size(); i++)
			objectPoints.push_back(getObjectPoints());

		CalibrationResult res;
		res.imageSize = imageSize;

		Mat_<double> E, F;
		double rms = stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints,
			res.cameraMatrix[0], res.distCoeffs[0],
			res.cameraMatrix[1], res.distCoeffs[1],
			res.imageSize, res.rotM, res.T, E, F,
			TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			0 /*CV_CALIB_RATIONAL_MODEL +
			CV_CALIB_FIX_ASPECT_RATIO +
			CV_CALIB_ZERO_TANGENT_DIST +  
			CV_CALIB_SAME_FOCAL_LENGTH +
			CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/);
	
		cout << "Calibrated with RMS error: " << rms << endl;  
		cout << "Average reprojection error: " << calcAvgReprojectionErr(res, F) << endl;
		
		res.calcRectification();

		return res;
	}
	
	double CameraCalibration::calcAvgReprojectionErr(const CalibrationResult& res, const Mat_<double>& F) {
		// The following code was adapted from the OpenCV calibration example
	
		// CALIBRATION QUALITY CHECK
		// because the output fundamental matrix implicitly
		// includes all the output information,
		// we can check the quality of calibration using the
		// epipolar geometry constraint: m2^t*F*m1=0

		double err = 0;
		vector<Vec3f> leftLines, rightLines;
		for(unsigned int i = 0; i < leftImagePoints.size(); i++ ) {
			vector<Point2f> leftPoints(leftImagePoints[i].size()),
				rightPoints(rightImagePoints[i].size());
			
			undistortPoints(leftImagePoints[i], leftPoints, res.cameraMatrix[0], res.distCoeffs[0], Mat(), res.cameraMatrix[0]);
			computeCorrespondEpilines(leftPoints, 1, F, leftLines);
			
			undistortPoints(rightImagePoints[i], rightPoints, res.cameraMatrix[1], res.distCoeffs[1], Mat(), res.cameraMatrix[1]);
			computeCorrespondEpilines(rightPoints, 2, F, rightLines);
		
			for(unsigned int j = 0; j < leftImagePoints[i].size(); j++ ) {
				err +=
					fabs(leftImagePoints[i][j].x*rightLines[j][0] +
					leftImagePoints[i][j].y*rightLines[j][1] + rightLines[j][2]) +
					fabs(rightImagePoints[i][j].x*leftLines[j][0] +
					rightImagePoints[i][j].y*leftLines[j][1] + leftLines[j][2]);
			}
		}
		return  err/(leftImagePoints.size()*leftImagePoints[0].size());
	}
	
	bool CameraCalibration::findCalibPatternCorners(const Mat_<unsigned char>& image,
		vector<Point2f>* corners, Mat_<Vec3b>* visualOutput) {
		
		Size boardSize = aucklandBoard ? Size(calibSize.width + 2, calibSize.height + 2) : calibSize;
		
		bool found = false;
		// We search for corners at different scales
		for(int scale = 1; scale <= maxScale && !found; scale*=2 ) {
			cout << "Detecting corners at scale " << scale << endl;
			Mat_<unsigned char> searchImage;
			if(scale == 1)
				searchImage = image;
			else resize(image, searchImage, Size(), scale, scale);
			
			found = findChessboardCorners(searchImage, boardSize, *corners, CV_CALIB_CB_ADAPTIVE_THRESH |
				CV_CALIB_CB_NORMALIZE_IMAGE /*| CV_CALIB_CB_FILTER_QUADS*/);
			if(scale != 1) {
				Mat cornersMat(*corners);
				cornersMat *= 1.0/scale;
			}
		}

		if(found) {
			// We have found all corners
			
			if(aucklandBoard) {
				// The UoA board has a frame we need to cut off
				vector<Point2f> newCorners;
				for(int y = 1; y < boardSize.height -1; y++)
					for(int x = 1; x <boardSize.width -1; x++)
						newCorners.push_back((*corners)[y*boardSize.width + x]);
				
				*corners = newCorners;
			}
		
			// With a square calibration board we might have to rotate the matches
			//rotateIfNeccessary(corners);
			
			// Refine matches
			int winSize = findSubPixWinSize(*corners);
			cout << "Subpixel window size: " << winSize << endl;
			if(winSize > 1)
				cornerSubPix(image, *corners, Size(winSize, winSize), Size(-1, -1),
					TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
			
		}
		
		if(visualOutput != NULL) {
			// Visualize
			cvtColor(image, *visualOutput, CV_GRAY2RGB);
			drawChessboardCorners(*visualOutput, calibSize, Mat(*corners), found);
		}
		
		return found;
	}
	
	vector<Point3f> CameraCalibration::getObjectPoints() {
		vector<Point3f> ret;
		for(int y=0; y<calibSize.height; y++)
			for(int x=0; x<calibSize.width; x++) {
				ret.push_back(Point3f(y*squareSize, x*squareSize, 0));
			}
		return ret;
	}
	
	int CameraCalibration::findSubPixWinSize(const vector<Point2f>& corners) {
		// Find minimal distance
		double dmin = INT_MAX;
		
		// Find board orientation
		int horizontalVotes = 0, verticalVotes = 0;
		for(unsigned int i=0; i<corners.size() - calibSize.width; i++) {
			double dx1 = fabs(corners[i].x - corners[i+1].x);
			double dx2 = fabs(corners[i].x - corners[i+calibSize.width].x);
			if(dx1 > dx2)
				horizontalVotes ++;
			else verticalVotes++;
		}
		
		// Find minimum distance
		for(unsigned int i=0; i<corners.size() - calibSize.width; i++) {
			double dx, dy;
			if(horizontalVotes > verticalVotes) {
				dx = fabs(corners[i].x - corners[i+1].x);
				dy = fabs(corners[i].y - corners[i+calibSize.width].y);
			} else {
				dx = fabs(corners[i].x - corners[i+calibSize.width].x);
				dy = fabs(corners[i].y - corners[i+1].y);
			}
			
			if(dx < dmin)
				dmin = dx;
			if(dy < dmin)
				dmin = dy;
		}
		
		return (int) round(dmin * 0.8) - 1; // Apply some tolerance
	}
	
	void CameraCalibration::rotateIfNeccessary(vector<Point2f>* corners) { 
		// Only neccessary for square boards
	
		// Find top-left board corner
		Point2f boardCorners[4] = {
			corners->front(),
			(*corners)[calibSize.width-1],
			(*corners)[corners->size() - calibSize.width],
			corners->back()};
		
		// Find top and bottom corners with a slow bubble sort
		int cornerIndices[4] = {0, 1, 2, 3};
		bool changed = true;
		
		while(changed) {
			changed = false;
			for(int i=0; i<3; i++) {
				if(boardCorners[cornerIndices[i+1]].y < boardCorners[cornerIndices[i]].y) {
					int buffer = cornerIndices[i];
					cornerIndices[i] = cornerIndices[i+1];
					cornerIndices[i+1] = buffer;
					changed = true;
				}
			}
		}
		
		// Order corners
		int topLeft = boardCorners[cornerIndices[0]].x < boardCorners[cornerIndices[1]].x ? cornerIndices[0] : cornerIndices[1];
		int topRight = boardCorners[cornerIndices[0]].x < boardCorners[cornerIndices[1]].x ? cornerIndices[1] : cornerIndices[0];
		int bottomLeft = boardCorners[cornerIndices[2]].x < boardCorners[cornerIndices[3]].x ? cornerIndices[2] : cornerIndices[3];
		int bottomRight = boardCorners[cornerIndices[2]].x < boardCorners[cornerIndices[3]].x ? cornerIndices[3] : cornerIndices[2];
		
		// Select the right orientation / mirror operation
			
		if(topLeft == 0 && topRight == 1 && bottomLeft == 2 && bottomRight == 3)
			return; // Correct orientation

		int start=0, step1=1, step2=calibSize.width;
	
		if(topLeft == 3 && topRight == 2 && bottomLeft == 1 && bottomRight == 0) {
			// Rotate 180 deg
			start = corners->size()-1;
			step1 = -1;
			step2 = -calibSize.width;
		}
		else if(topLeft == 2 && topRight == 3 && bottomLeft == 0 && bottomRight == 1) {
			// Rotate 180 deg and mirror
			start = corners->size() - calibSize.width;
			step1 = 1;
			step2 = -calibSize.width;
		}
		else if(topLeft == 1 && topRight == 3 && bottomLeft == 0 && bottomRight == 2) {
			// Rotate -90 deg
			start = calibSize.width-1;
			step1 = calibSize.width;
			step2 = -1;
		} 
		else if(topLeft == 0 && topRight == 2 && bottomLeft == 1 && bottomRight == 3) {
			// Rotate -90 deg and mirror
			start = 0;
			step1 = calibSize.width;
			step2 = 1;
		}
		else if(topLeft == 2 && topRight == 0 && bottomLeft == 3 && bottomRight == 1) {
			// Rotate +90 deg
			start = corners->size() - calibSize.width;
			step1 = -calibSize.width;
			step2 = 1;
		} 
		else if(topLeft == 3 && topRight == 1 && bottomLeft == 2 && bottomRight == 0) {
			// Rotate +90 deg and mirror
			start = corners->size() - 1;
			step1 = -calibSize.width;
			step2 = -1;
		}
		
		if(abs(step2) < abs(step1)) {
			// We need to flip the size
			int width = calibSize.width;
			calibSize.width = calibSize.height;
			calibSize.height = width;
		}
		
		vector<Point2f> rotated(corners->size());
		for(int y=0; y<calibSize.height; y++) {
			for(int x=0; x<calibSize.width; x++)
				rotated[y*calibSize.width + x] = (*corners)[start + y*step2 + x*step1];
		}
		(*corners) = rotated;
	}
}
