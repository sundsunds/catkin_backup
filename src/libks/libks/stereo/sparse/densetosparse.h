#ifndef KS_DENSETOSPARSE_H
#define KS_DENSETOSPARSE_H

namespace ks {
	// Runs a dense stereo algorithm and only pick selected sparse points from it
	template <class DENSE_ALG>
	class DenseToSparse {
	public:
		DenseToSparse(const DENSE_ALG& dense)
			: dense(dense) {}
		
		void match(const cv::Mat& left, const cv::Mat& right,
			const std::vector<cv::KeyPoint>& leftFeatures, std::vector<SparseMatch>* matches) {
			
			cv::Mat_<unsigned char> out;
			dense.match(left, right, &out);
			
			for(unsigned int i=0; i< leftFeatures.size(); i++) {
				unsigned char disparity = out(leftFeatures[i].pt.y, leftFeatures[i].pt.x);
				if(disparity != 255)
					matches->push_back(SparseMatch(&leftFeatures[i], NULL, leftFeatures[i].pt,
						cv::Point2f(leftFeatures[i].pt.x-disparity, leftFeatures[i].pt.y)));
			}
		}
			
	private:
		DENSE_ALG dense;
	};
}

#endif