#ifndef KS_SPARSECOMPLETION_H
#define KS_SPARSECOMPLETION_H

#include "libks/stereo/sparse/sparsematch.h"

struct triangulateio;

namespace ks {
	// Performs stereo matching by first taking a set of features
	// from sparse stereo matching
	class SparseCompletion {
	public:
		SparseCompletion(int maxDisp, int subsampling, StereoRectification* rect = NULL);
	
		void match(const std::vector<SparseMatch>& sparseMatches,
			const cv::Mat_<unsigned int>& left, const cv::Mat_<unsigned int>& right,
			cv::Mat_<float>& dispMap);
	
	private:
		int maxDisp;
		int subsampling;
		StereoRectification* rect;
		cv::Mat_<unsigned char> minDispMap, maxDispMap;
	
		// Helper method for pointInTriangle
		inline float sign(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3) {
			return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
		}
		
		// Returns true if the given point is inside the given triangle
		inline bool pointInTriangle(const cv::Point2f& pt, const cv::Point2f& v1,
			const cv::Point2f& v2, const cv::Point2f& v3) {
			bool b1, b2, b3;

			b1 = sign(pt, v1, v2) < 0.0F;
			b2 = sign(pt, v2, v3) < 0.0F;
			b3 = sign(pt, v3, v1) < 0.0F;
			
			return ((b1 == b2) && (b2 == b3));
		}
		
		// Computes Barycentric coordinates
		inline cv::Point3f baryCoords(const cv::Point2f& vec, const cv::Point2f& a,
			const cv::Point2f& b, const cv::Point2f& c) {
			cv::Point3f lambda;
			double den = 1.0F / ((b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y));

			lambda.x = ((b.y - c.y) * (vec.x - c.x) + (c.x - b.x) * (vec.y - c.y)) * den;
			lambda.y = ((c.y - a.y) * (vec.x - c.x) + (a.x - c.x) * (vec.y - c.y)) * den;
			lambda.z = 1 - lambda.x - lambda.y;

			return lambda;
		}
		
		// Performs interpolation of the sparse stereo matches
		void interpolateMatches(const std::vector<SparseMatch>& sparseMatches,
			cv::Mat_<float>& dispMap);
		// Renders a triangle with color interpolation
		void interpolateTriangle(const SparseMatch& a, const SparseMatch& b, 
			const SparseMatch& c, cv::Mat_<float>& dst);
		// Filters erroneous triangles from the triangulation result
		void filterTriangles(const std::vector<SparseMatch>& sparseMatches, triangulateio& tri);
		// Refines the disparity map through stereo matching
		void refineDispMap(cv::Mat_<float>& dispMap, const cv::Mat_<unsigned int>& left,
			const cv::Mat_<unsigned char>& right);
	};
}

#endif
