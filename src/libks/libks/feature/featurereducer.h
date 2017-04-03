#ifndef KS_FEATUREREDUCER_H
#define KS_FEATUREREDUCER_H

#include <opencv2/opencv.hpp>
#include <vector>

namespace ks {
	class FeatureReducer {
	public:
		FeatureReducer(unsigned int maxFeatures, int imgWidth, int imgHeight);
	
		void reduce(const std::vector<cv::KeyPoint>& in,
			std::vector<cv::KeyPoint>* out);
			
	private:
		unsigned int maxFeatures;
		int imgWidth, imgHeight;
		
		struct KeyPointComp {
			bool operator() (const cv::KeyPoint* a, const cv::KeyPoint* b) const {
				return a->octave > b->octave || (a->octave == b->octave && a->response > b->response);
			}
		};
		
		/*cv::Mat_<unsigned int> offsets;
		void initOffsets(const std::vector<cv::KeyPoint>& points);
		
		//Just for testing
		void testOffsets(const std::vector<cv::KeyPoint>& points);*/
	};
}

#endif
