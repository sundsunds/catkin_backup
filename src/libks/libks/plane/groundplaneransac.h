#ifndef KS_GROUNDPLANERANSAC_H
#define KS_GROUNDPLANERANSAC_H

#include <tf/tf.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/point_types.h>

namespace ks {
	// Extracts the camera pose from plane points using RANSAC
	class GroundPlaneRansac {
	public:
		GroundPlaneRansac(double yawVariance, double depthErrorScale, double rollOffset,
			double pitchOffset);
		
		// Estimates the camera pose with RANSAC plane detectoin
		bool estimatePose(const sensor_msgs::PointCloud::ConstPtr& msg, double yaw,
			double* roll, double* pitch, double* height);
			
		// Returns the inliers of the last estimation
		const std::vector<pcl::PointXYZ>& getInliers() {return inliers;}
		
		// Calculates the variances for the previous RANSAC estimation
		void estimateVariances(double* heightVar, tf::Quaternion* orientationVar);
	
	private:
		double depthErrorScale;
		double rollOffset, pitchOffset;
	
		std::vector<pcl::PointXYZ> inliers; //RANSAC inliers
		std::vector<tf::Quaternion> quatVarSamples;
		std::vector<double> heightVarSamples;
		std::vector<double> rollVarSamples;
		std::vector<double> pitchVarSamples;
		
		// Plane representations
		double plane[4];
		
		// Results
		double pitch, roll, yaw, pitchVar, rollVar;
		tf::Quaternion orientationVar;
		double height, heightVar;
		
		// yaw sampling
		boost::mt19937 gen;
		boost::scoped_ptr<boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > > yawRandom;
		
		// Performs RANSAC plane estimation
		bool performRANSAC(const sensor_msgs::PointCloud::ConstPtr& msg);
		
		// Median vertical distance of all points in the message
		double calcMedianDistance(sensor_msgs::PointCloud::ConstPtr msg);
		
		// Converts a plane from 3 points to vector/parameter representation. t-vector will be p2
		void planePointsToVectors(const tf::Vector3& p1, const tf::Vector3& p2,
			const tf::Vector3& p3,  tf::Vector3& u, tf::Vector3& v);
		
		// Extracts pitch and roll angle from a vector/parameter represented plane
		void planeVectorsToAngles(const tf::Vector3& u, const tf::Vector3& v, double& roll, double& pitch);

		// Aligns the plane vectors such that t=(0, ?, 0); u=(?, ?, 0); v=(0, ?, ?)
		void alignPlaneVectors(const tf::Vector3& t1, const tf::Vector3& u1, const tf::Vector3& v1,
			tf::Vector3& t2, tf::Vector3& u2, tf::Vector3& v2);
			
		// Try to shuffle samples such that always 3 are far away from each others
		void shuffleInliers();
		
		// Calculates the variance of roll and pitch by sampling
		void sampleRollPitchVar();
		
		void sampleHeightVar();
		
		// Extracts the camera/plane pose from the plane equation
		void calculatePose();
	};
}

#endif
