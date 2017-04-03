#include "libks/plane/planeesmestimator.h"
#include <cvd/esm.h>
#include <iostream>
#include <cmath>
#include <ros/ros.h>

namespace ks {
	using namespace cv;
	using namespace std;
	
	PlaneESMEstimator::PlaneESMEstimator(const CalibrationResult& calib,
		int imageWidth, int imageHeight, double minDelta, int maxIter, double maxDelta)
		: calib(calib), minDelta(minDelta), maxIter(maxIter), maxDelta(maxDelta), frameSkipped(false),
		frameProcessed(false) {

		rectifiedCameraMatrix = getOptimalNewCameraMatrix(calib.cameraMatrix[0], calib.distCoeffs[0],
			calib.imageSize, 0, cv::Size(imageWidth, imageHeight));
		initUndistortRectifyMap(calib.cameraMatrix[0], calib.distCoeffs[0], calib.R[0],
			rectifiedCameraMatrix, cv::Size(imageWidth, imageHeight), CV_32FC1/*CV_16SC2*/,
			imageRectificationMap[0], imageRectificationMap[1]);
	}
	
	void PlaneESMEstimator::setTargetImage(const cv::Mat_<unsigned char> & img) {
		if(!frameProcessed)
			frameSkipped = true;
		else frameSkipped = false;
		
		frameProcessed = false;
	
		lastImage = rectified;
		rectified = Mat_<double>();
		
		remap(img, rectified, imageRectificationMap[0], imageRectificationMap[1], INTER_LINEAR,
			BORDER_REPLICATE);
	}
	
	bool PlaneESMEstimator::estimate(double lastHeight, double currentHeight, double* dx, double* dy, double* dYaw) {
		// Rectify and scale input image
		bool ret = false;
		if(lastImage.rows > 0 && lastImage.data != NULL) {
			// Convert to cvd sub-image (no data is copied)
			CVD::SubImage<CVD::byte> cvdImg(rectified.data,
				CVD::ImageRef(rectified.cols, rectified.rows), rectified.step[0]);
			CVD::SubImage<CVD::byte> cvdLast(lastImage.data,
				CVD::ImageRef(lastImage.cols, lastImage.rows), lastImage.step[0]);

			// Calculate gradient
			/*CVD::Image<TooN::Vector<2, typename CVD::Pixel::traits<CVD::byte>::wider_type> >
				gradImg(CVD::ImageRef(rectified.cols, rectified.rows));

			Mat_<short> sobelX, sobelY;
			Sobel(lastImage, sobelX, CV_16S, 1, 0, CV_SCHARR);
			Sobel(lastImage, sobelY, CV_16S, 0, 1, CV_SCHARR);
			for(int y = 0; y<rectified.rows; y++)
				for(int x = 0; x<rectified.cols; x++) {
					gradImg[y][x][0] = sobelX(y,x);
					gradImg[y][x][1] = sobelY(y,x);
			}*/
			
			CVD::ESMEstimator<CVD::Homography<3>, CVD::OffsetAppearance> esmEstimator;
			
			if(!frameSkipped) {
				// Find a homography that maps the last plane rotation to the current one
				Point3f lastT(0, 0, lastHeight);
				Point3f lastU(1/*cos(lastPitch)*/, 0, 0/*sin(lastPitch)*/);
				Point3f lastV(0, -1/*-cos(lastRoll)*/, 0/*sin(lastRoll)*/);
				
				Point3f currentT(0, 0, currentHeight);
				Point3f currentU(1/*cos(pitch)*/, 0, 0/*sin(pitch)*/);
				Point3f currentV(0, -1/*-cos(roll)*/, 0/*sin(roll)*/);
				
				vector<Point3f> lastPoints3D;
				lastPoints3D.push_back(lastT + 0.1*lastU + 0.1*lastV);
				lastPoints3D.push_back(lastT - 0.1*lastU + 0.1*lastV);
				lastPoints3D.push_back(lastT - 0.1*lastU - 0.1*lastV);
				lastPoints3D.push_back(lastT + 0.1*lastU - 0.1*lastV);

				vector<Point3f> currentPoints3D;
				currentPoints3D.push_back(currentT + 0.1*currentU + 0.1*currentV);
				currentPoints3D.push_back(currentT - 0.1*currentU + 0.1*currentV);
				currentPoints3D.push_back(currentT - 0.1*currentU - 0.1*currentV);
				currentPoints3D.push_back(currentT + 0.1*currentU - 0.1*currentV);
				
				vector<Point2f> lastPoints2D, currentPoints2D;
				Mat_<double> nullVec3 = Mat::zeros(3, 1, CV_32F);
				Mat_<double> nullVec4 = Mat::zeros(4, 1, CV_32F);
				projectPoints(lastPoints3D, nullVec3, nullVec3, rectifiedCameraMatrix,
					nullVec4, lastPoints2D);
				projectPoints(currentPoints3D, nullVec3, nullVec3, rectifiedCameraMatrix,
					nullVec4, currentPoints2D);

				Mat_<double> initialH = findHomography(lastPoints2D, currentPoints2D);

				TooN::Matrix<3> toonInitialH;
				for(int r = 0; r<3; r++)
					for(int c = 0; c<3; c++)
						toonInitialH[r][c] = initialH[r][c];
				
				esmEstimator.transform = CVD::Homography<3>(toonInitialH);
			}
			
			// Perform ESM
			//CVD::ESMEstimator<CVD::HomographyPrefix<3>, CVD::OffsetAppearance> esmEstimator;
			//esmEstimator.transform = CVD::HomographyPrefix<3>(toonInitialH);
			esmEstimator.set_image(cvdLast);
			esmEstimator.min_delta = minDelta;
			esmEstimator.max_iterations = maxIter;
			CVD::ESMResult esmRes = esmEstimator.optimize(cvdImg);
			//CVD::ESMResult esmRes = esmEstimator.optimize(cvdLast, gradImg, cvdImg);
			
			//cout << "iter: " << esmRes.iterations << "; delta: " << esmRes.delta << "; err: " << esmRes.error << endl;
			
			TooN::Matrix<3> H = esmEstimator.transform.get_matrix();
			lastH = (Mat_<double>(3, 3) <<
				H[0][0], H[0][1], H[0][2],
				H[1][0], H[1][1], H[1][2],
				H[2][0], H[2][1], H[2][2]);
			
			// Decompose homography
			double hTX = lastH(0,2)/lastH(2,2);
			double hTY = lastH(1,2)/lastH(2,2);
			
			Mat_<double> yawVec = lastH* (Mat_<double>(3,1) << 1e100, 0,1);
			*dYaw = atan2(yawVec(1)/yawVec(2) - hTX, yawVec(0)/yawVec(2) -hTY);
			
			// Convert pixel to m and correct pitch/roll pseudo translation
			*dx = -hTX * currentHeight / rectifiedCameraMatrix(0, 0);/*f_x*/
				//+ height*tan(pitch - lastPitch);
			*dy = -hTY * currentHeight / rectifiedCameraMatrix(1, 1);/*f_y*/
				//+ height*tan(roll - lastRoll);
			ret = (esmRes.delta < maxDelta);
		}
		
		frameProcessed = true;
		
		return ret;
	}
	
	Mat_<unsigned char> PlaneESMEstimator::getSmallWarped() {
		if(lastImage.data == NULL || lastImage.rows == 0 || lastH.rows == 0)
			return Mat_<unsigned char>();
		else {
			Mat_<unsigned char> warped;
			warpPerspective(lastImage, warped, lastH, rectified.size());
			return warped;
		}
	}
}
