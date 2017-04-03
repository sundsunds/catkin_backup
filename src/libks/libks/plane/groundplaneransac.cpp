#include "libks/plane/groundplaneransac.h"
#include "libks/base/taitbryan.h"
#include <boost/bind.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace ks {
	using namespace std;
	using namespace tf;
	using namespace boost::accumulators;
	using namespace boost;
	using namespace pcl;

	GroundPlaneRansac::GroundPlaneRansac(double yawVariance, double depthErrorScale, double rollOffset,
		double pitchOffset): depthErrorScale(depthErrorScale), rollOffset(rollOffset), pitchOffset(pitchOffset) {
		
		yawRandom.reset(new variate_generator<mt19937&, normal_distribution<> >(
			gen, normal_distribution<>(0, yawVariance / 180.0 * M_PI)));
		plane[0] = plane[2] = plane[3] = 0; plane[1] = 1;
	}
	
	bool GroundPlaneRansac::estimatePose(const sensor_msgs::PointCloud::ConstPtr& msg, double yaw,
		double* roll, double* pitch, double* height) {
		
		this->yaw = yaw;
		if(performRANSAC(msg)) {
			//calcRollPitch();
			//calcHeight();
			
			calculatePose();
						
			(*roll) = this->roll + rollOffset;
			(*pitch) = this->pitch + pitchOffset;
			(*height) = this->height;
			
			return true;
		} else return false;
	}
	
	double GroundPlaneRansac::calcMedianDistance(sensor_msgs::PointCloud::ConstPtr msg) {
		vector<double> dist(msg->points.size());
		for(unsigned int i=0; i<msg->points.size(); i++)
			dist[i] = msg->points[i].y;
		sort(dist.begin(), dist.end());
		return dist[dist.size()/2];
	}
	
	void GroundPlaneRansac::estimateVariances(double* heightVar, tf::Quaternion* orientationVar) {
		sampleRollPitchVar();
		sampleHeightVar();
	
		(*heightVar) = this->heightVar;
		(*orientationVar) = this->orientationVar;
	}
	
	bool GroundPlaneRansac::performRANSAC(const sensor_msgs::PointCloud::ConstPtr& msg) {
		// Initialize cloud
		PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
		cloud->width = msg->points.size();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		
		for(unsigned int i=0; i<msg->points.size(); i++) {
			cloud->points[i].x = msg->points[i].x;
			cloud->points[i].y = msg->points[i].y;
			cloud->points[i].z = msg->points[i].z;
		}
		
		// Perform RANSAC
		SampleConsensusModelPlane<PointXYZ>::Ptr planeModel(new SampleConsensusModelPlane<PointXYZ> (cloud));
		RandomSampleConsensus<PointXYZ> ransac(planeModel);
		
		double medianDist = calcMedianDistance(msg);
		ransac.setDistanceThreshold (depthErrorScale * medianDist*medianDist);
	
		if(ransac.computeModel()) {
			// Check if inliers minimum was reached
			vector<int> inliersIdx;
			ransac.getInliers(inliersIdx);

			// Copy inliers
			inliers.resize(inliersIdx.size());
			for(unsigned int i = 0; i< inliersIdx.size(); i++)
				inliers[i] = cloud->points[inliersIdx[i]];
			
			// Get plane model
			Eigen::VectorXf modelCoef;
			ransac.getModelCoefficients(modelCoef);
			
			// Optimize
			Eigen::VectorXf optimizedCoef = modelCoef;
			planeModel->optimizeModelCoefficients(inliersIdx, modelCoef, optimizedCoef);
			
			if(optimizedCoef.size() != 4)
				return false;
				
			plane[0] = optimizedCoef[0];
			plane[1] = optimizedCoef[1];
			plane[2] = optimizedCoef[2];
			plane[3] = optimizedCoef[3];
			
			return true;
		} else return false;
	}
	
	void GroundPlaneRansac::planePointsToVectors(const Vector3& p1, const Vector3& p2, const Vector3& p3,  Vector3& u, Vector3& v) {
		u = p2 - p1;
		v = p3 - p1;
		
		// Make sure plane vectors always point the same direction
		if(u.getX() < 0)
			u = -u;
		if(v.getZ() < 0)
			v = -v;
	}

	void GroundPlaneRansac::planeVectorsToAngles(const Vector3& u, const Vector3& v,
		double& roll, double& pitch) {
		
		pitch = atan2(u.getY(), u.getX());
		roll = atan2(v.getY(), v.getZ());
	}
	
	void GroundPlaneRansac::alignPlaneVectors(const Vector3& t1, const Vector3& u1, const Vector3& v1,
		Vector3& t2, Vector3& u2, Vector3& v2) {
		Vector3 normal = u1.cross(v1);
		double c = (-normal.x()*t1.x() - normal.y()*t1.y() - normal.z()*t1.z());
		
		t2 = Vector3(0, -c/normal.y(), 0);
		planePointsToVectors(t2, Vector3(-c/normal.x(), 0, 0),
			Vector3(0, 0, -c/normal.z()), u2, v2);
	}

	// Try to shuffle samples such that always 3 are far away from each others
	void GroundPlaneRansac::shuffleInliers() {
		static const unsigned int offsetRange = 5;
	
		random_shuffle(inliers.begin(), inliers.end());
		
		for(unsigned int i=1; i<inliers.size(); i++) {
			double maxDist = 0;
			int maxDistOffset = 0;
			unsigned int maxOffset = std::min(offsetRange, (unsigned int)(inliers.size()-i-1));
			
			for(unsigned int offset=0; offset < maxOffset; offset++) {
				double dx1 = inliers[i+offset].x - inliers[i-1].x;
				double dz1 = inliers[i+offset].z - inliers[i-1].z;
				
				double dx2 = dx1, dz2 = dz1;
				if(i >=3) {
					dx2 = inliers[i+offset].x - inliers[i-3].x;
					dz2 = inliers[i+offset].z - inliers[i-3].z;
				}
				
				double dist = std::min(dx1*dx1 + dz1*dz1, dx2*dx2 + dz2*dz2);
				if(dist > maxDist) {
					maxDist = dist;
					maxDistOffset = offset;
				}				
			}
			
			if(maxDistOffset != 0)
				swap(inliers[i], inliers[i+maxDistOffset]);
		}
	}
	
	// Calculates the variance of roll and pitch by sampling
	/*void GroundPlaneRansac::calcRollPitch() {
		shuffleInliers();
		
		quatVarSamples.resize(inliers.size());
		rollVarSamples.resize(inliers.size());
		pitchVarSamples.resize(inliers.size());
		
		// Collect samples
		for(unsigned int i=0; i<inliers.size(); i++) {
			int idx1 = i, idx2 = (i+1)%inliers.size(),
				idx3 = (i+3)%inliers.size();
				
			// Get first plane representation
			Vector3 u1, v1, t1 = Vector3(inliers[idx2].x, inliers[idx2].y, inliers[idx2].z);
			planePointsToVectors(t1, Vector3(inliers[idx1].x, inliers[idx1].y, inliers[idx1].z),
				Vector3(inliers[idx3].x, inliers[idx3].y, inliers[idx3].z), u1, v1);
				
			// Get plane representation aligned with roll and pitch axis
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			// Calculate roll and pitch angle
			double currRoll, currPitch;
			planeVectorsToAngles(u2, v2, currRoll, currPitch);
			
			// Sample orientation (we already consider the offsets here)
			Matrix3x3 rotM;
			rotM.setEulerYPR(yaw + (*yawRandom)(), currRoll + rollOffset, currPitch + pitchOffset);
			rotM.getRotation(quatVarSamples[i]);
				
			rollVarSamples[i] = currRoll;
			pitchVarSamples[i] = currPitch;
		}

		// Calculate roll pitch and corresponding variances
		accumulator_set<double, stats<tag::variance> > rollAcc, pitchAcc;
		for_each(rollVarSamples.begin(), rollVarSamples.end(), bind<void>(ref(rollAcc), _1));
		for_each(pitchVarSamples.begin(), pitchVarSamples.end(), bind<void>(ref(pitchAcc), _1));
		roll = accumulators::mean(rollAcc);
		pitch = accumulators::mean(pitchAcc);
		rollVar = variance(rollAcc) / inliers.size();
		pitchVar = variance(rollAcc) / inliers.size();
		
		// Calculate quaternion variance
		Quaternion meanQ(0, 0, 0, 0);
		for(unsigned int i=0; i<quatVarSamples.size(); i++) {
			meanQ += quatVarSamples[i];
		}
		meanQ *= 1.0/quatVarSamples.size();
		
		orientationVar = Quaternion(0, 0, 0, 0);
		for(unsigned int i=0; i<quatVarSamples.size(); i++) {
			double dx = quatVarSamples[i].x() - meanQ.x();
			double dy = quatVarSamples[i].y() - meanQ.y();
			double dz = quatVarSamples[i].z() - meanQ.z();
			double dw = quatVarSamples[i].w() - meanQ.w();
			
			orientationVar.setX(orientationVar.x() + dx*dx);
			orientationVar.setY(orientationVar.y() + dy*dy);
			orientationVar.setZ(orientationVar.z() + dz*dz);
			orientationVar.setW(orientationVar.w() + dw*dw);
		}
		orientationVar *= (1.0/(quatVarSamples.size()-1)) / quatVarSamples.size();
		
		// Calculate quaternion
		Matrix3x3 rotM;
		rotM.setEulerYPR(yaw, roll + rollOffset, pitch + pitchOffset);
		rotM.getRotation(orientation);
	}*/

	void GroundPlaneRansac::sampleRollPitchVar() {
		shuffleInliers();
		
		quatVarSamples.resize(inliers.size());
		rollVarSamples.resize(inliers.size());
		pitchVarSamples.resize(inliers.size());
		
		// Collect samples
		for(unsigned int i=0; i<inliers.size(); i++) {
			int idx1 = i, idx2 = (i+1)%inliers.size(),
				idx3 = (i+3)%inliers.size();
				
			// Get first plane representation
			Vector3 u1, v1, t1 = Vector3(inliers[idx2].x, inliers[idx2].y, inliers[idx2].z);
			planePointsToVectors(t1, Vector3(inliers[idx1].x, inliers[idx1].y, inliers[idx1].z),
				Vector3(inliers[idx3].x, inliers[idx3].y, inliers[idx3].z), u1, v1);
				
			// Get plane representation aligned with roll and pitch axis
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			// Calculate roll and pitch angle
			double currRoll, currPitch;
			planeVectorsToAngles(u2, v2, currRoll, currPitch);
			
			// Sample orientation (we already consider the offsets here)
			/*Matrix3x3 rotM;
			rotM.setEulerYPR(yaw + (*yawRandom)(), currRoll + rollOffset, currPitch + pitchOffset);
			rotM.getRotation(quatVarSamples[i]);*/
			quatVarSamples[i] = TaitBryan::toTfQuaternion(yaw + (*yawRandom)(), currPitch + pitchOffset, currRoll + rollOffset);
				
			rollVarSamples[i] = currRoll;
			pitchVarSamples[i] = currPitch;
		}

		// Calculate roll pitch and corresponding variances
		accumulator_set<double, stats<tag::variance> > rollAcc, pitchAcc;
		for_each(rollVarSamples.begin(), rollVarSamples.end(), bind<void>(ref(rollAcc), _1));
		for_each(pitchVarSamples.begin(), pitchVarSamples.end(), bind<void>(ref(pitchAcc), _1));
		rollVar = variance(rollAcc) / inliers.size();
		pitchVar = variance(rollAcc) / inliers.size();
		
		// Calculate quaternion variance
		Eigen::Vector4d meanQ(0, 0, 0, 0);
		for(unsigned int i=0; i<quatVarSamples.size(); i++) {
			meanQ += Eigen::Vector4d(quatVarSamples[i].x(), quatVarSamples[i].y(), quatVarSamples[i].z(),
				quatVarSamples[i].w());
		}
		meanQ *= 1.0/quatVarSamples.size();
		
		orientationVar = Quaternion(0, 0, 0, 0);
		for(unsigned int i=0; i<quatVarSamples.size(); i++) {
			double dx = quatVarSamples[i].x() - meanQ[0];
			double dy = quatVarSamples[i].y() - meanQ[1];
			double dz = quatVarSamples[i].z() - meanQ[2];
			double dw = quatVarSamples[i].w() - meanQ[3];
			
			orientationVar.setX(orientationVar.x() + dx*dx);
			orientationVar.setY(orientationVar.y() + dy*dy);
			orientationVar.setZ(orientationVar.z() + dz*dz);
			orientationVar.setW(orientationVar.w() + dw*dw);
		}
		orientationVar *= (1.0/(quatVarSamples.size()-1)) / quatVarSamples.size();
	}
	
	// Calculates the variance of roll and pitch by sampling
	/*void GroundPlaneRansac::sampleRollPitchVar() {
		rollVarSamples.resize(inliers.size());
		pitchVarSamples.resize(inliers.size());
		
		orientationVar = Quaternion(0, 0, 0, 0);
		double rollSum, pitchSum;
		
		// Collect samples
		Vector3 t(0, height, 0);
		for(unsigned int i=0; i<inliers.size(); i++) {
			int idx1 = i, idx2 = (i+1)%inliers.size(),
				idx3 = (i+3)%inliers.size();
		
			// Get first plane representation
			Vector3 u1, v1, t1 = Vector3(inliers[idx2].x, inliers[idx2].y, inliers[idx2].z);
			planePointsToVectors(t1, Vector3(inliers[idx1].x, inliers[idx1].y, inliers[idx1].z),
				Vector3(inliers[idx3].x, inliers[idx3].y, inliers[idx3].z), u1, v1);
				
			// Get plane representation aligned with roll and pitch axis
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			// Calculate roll and pitch angle
			double currRoll, currPitch;
			planeVectorsToAngles(u2, v2, currRoll, currPitch);
			
			// Sample orientation (we already consider the offsets here)
			Matrix3x3 rotM;
			rotM.setEulerYPR(yaw + (*yawRandom)(), currRoll + rollOffset, currPitch + pitchOffset);
			Quaternion quatSample;
			rotM.getRotation(quatSample);
			
			double dx = quatSample.x() - orientation.x();
			double dy = quatSample.y() - orientation.y();
			double dz = quatSample.z() - orientation.z();
			double dw = quatSample.w() - orientation.w();
			
			orientationVar.setX(orientationVar.x() + dx*dx);
			orientationVar.setY(orientationVar.y() + dy*dy);
			orientationVar.setZ(orientationVar.z() + dz*dz);
			orientationVar.setW(orientationVar.w() + dw*dw);
				
			rollSum += (roll - currRoll)*(roll - currRoll);
			pitchSum += (pitch - currPitch)*(pitch - currPitch);
		}

		// Calculate roll pitch and corresponding variances
		double factor = (1.0/(inliers.size()-1)) / inliers.size();
		rollVar *= factor;
		pitchVar *= factor;
		orientationVar *= factor;
	}*/
	
	/*void GroundPlaneRansac::calcHeight() {
		variate_generator<mt19937&, normal_distribution<> > 
			rollRand(gen, normal_distribution<>(roll, sqrt(rollVar))),
			pitchRand(gen, normal_distribution<>(pitch, sqrt(pitchVar)));
	
		heightVarSamples.resize(inliers.size());
		
		// For each inlier, calculate the distance of the corresponding plane
		// to the camera
		for(unsigned int i=0; i<inliers.size(); i++) {
			double currPitch = pitchRand();
			double currRoll = rollRand();
			
			Vector3 u1(cos(currPitch), sin(currPitch), 0);
			Vector3 v1(0, sin(currRoll), cos(currRoll));
	
			Vector3 t1(inliers[i].x, inliers[i].y, inliers[i].z);
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			heightVarSamples[i] = t2.y();
		}
		
		accumulator_set<double, stats<tag::variance> > acc;
		for_each(heightVarSamples.begin(), heightVarSamples.end(), bind<void>(ref(acc), _1));
		height = accumulators::mean(acc);
		heightVar = variance(acc) / inliers.size();
	}*/

	void GroundPlaneRansac::sampleHeightVar() {
		variate_generator<mt19937&, normal_distribution<> > 
			rollRand(gen, normal_distribution<>(roll, sqrt(rollVar))),
			pitchRand(gen, normal_distribution<>(pitch, sqrt(pitchVar)));
	
		heightVarSamples.resize(inliers.size());
		
		// For each inlier, calculate the distance of the corresponding plane
		// to the camera
		for(unsigned int i=0; i<inliers.size(); i++) {
			double currPitch = pitchRand();
			double currRoll = rollRand();
			
			Vector3 u1(cos(currPitch), sin(currPitch), 0);
			Vector3 v1(0, sin(currRoll), cos(currRoll));
	
			Vector3 t1(inliers[i].x, inliers[i].y, inliers[i].z);
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			heightVarSamples[i] = t2.y();
		}
		
		accumulator_set<double, stats<tag::variance> > acc;
		for_each(heightVarSamples.begin(), heightVarSamples.end(), bind<void>(ref(acc), _1));
		heightVar = variance(acc) / inliers.size();
	}
	
	/*void GroundPlaneRansac::sampleHeightVar() {
		variate_generator<mt19937&, normal_distribution<> > 
			rollRand(gen, normal_distribution<>(roll, sqrt(rollVar))),
			pitchRand(gen, normal_distribution<>(pitch, sqrt(pitchVar)));
	
		// For each inlier, calculate the distance of the corresponding plane
		// to the camera
		double sumDev = 0;
		for(unsigned int i=0; i<inliers.size(); i++) {
			double currPitch = pitchRand();
			double currRoll = rollRand();
			
			Vector3 u1(cos(currPitch), sin(currPitch), 0);
			Vector3 v1(0, sin(currRoll), cos(currRoll));
	
			Vector3 t1(inliers[i].x, inliers[i].y, inliers[i].z);
			Vector3 t2, u2, v2;
			alignPlaneVectors(t1, u1, v1, t2, u2, v2); 
			
			sumDev += (t2.y() - height)*(t2.y() - height);
		}
		
		heightVar = sumDev / (inliers.size() - 1) / inliers.size();
	}*/
	
	void GroundPlaneRansac::calculatePose() {		
		// Convert plane to normalized parametric form
		Vector3 p1(-plane[3]/plane[0], 0, 0); 
		Vector3 p2(0, -plane[3]/plane[1], 0);
		Vector3 p3(0, 0, -plane[3]/plane[2]);
		
		Vector3 tVec = p2, uVec, vVec;
		planePointsToVectors(p2, p1, p3, uVec, vVec);
		
		// Calculate roll and pitch angle
		planeVectorsToAngles(uVec, vVec, roll, pitch);
		
		height = tVec.y();
	}
}
