#ifndef KS_BILINEARINTERPOLATION_H
#define KS_BILINEARINTERPOLATION_H

#include <opencv2/opencv.hpp>

namespace ks {
	// Provides a way to perform a bilinear interpolated
	// lookup for matrix elements at subpixel positions
	template<typename T>
	class BilinearInterpolation: public cv::Mat_<T> {
	public:
		// Constructors initializing from existing matrix
		BilinearInterpolation(const cv::Mat_<T>& mat)
			: cv::Mat_<T>(mat) {}
		
		// Perform bilinear interopolation
		float operator() (float row, float col) const {
			float cx = col - int(col);
			float cy = row - int(row);
			
			float ix1 = (1.0F - cx) * ((const cv::Mat_<T>&)(*this))(int(row), int(col)) + cx * ((const cv::Mat_<T>&)(*this))(int(row), int(col)+1);
			float ix2 = (1.0F - cx) * ((const cv::Mat_<T>&)(*this))(int(row)+1, int(col)) + cx * ((const cv::Mat_<T>&)(*this))(int(row)+1, int(col)+1);
			return (1.0F - cy)*ix1 + cy*ix2;
		}
	};
}
#endif
