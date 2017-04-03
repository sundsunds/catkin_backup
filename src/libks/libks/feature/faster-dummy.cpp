#include "libks/feature/faster.h"

void faster_detect(const cv::Mat_<unsigned char>& i, std::vector<cv::Point2i>& corners, int b) {}
void faster_score(const cv::Mat_<unsigned char>& i, const std::vector<cv::Point2i>& corners, int b,
	std::vector<int>& scores) {}
