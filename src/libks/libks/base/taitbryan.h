#ifndef KS_TAITBRYAN_H
#define KS_TAITBRYAN_H

#include <tf/tf.h>
#include <Eigen/Eigen>

// Converts between quaternion and Tait-Bryan angles.
// heading is like yaw,
// attitude like pitch,
// bank like roll

// See:
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
class TaitBryan {
public:
	static void fromQuaternion(double x, double y, double z, double w,
		double& heading, double& attitude, double& bank) {
		heading = atan2(2*z*w-2*y*x , 1 - 2*z*z - 2*x*x);
		attitude = asin(2*y*z + 2*x*w);
		bank =  atan2(2*y*w-2*z*x , 1 - 2*y*y - 2*x*x);
	}
	
	static void fromQuaternion(const tf::Quaternion& q, double& heading,
		double& attitude, double& bank) {
		fromQuaternion(q.x(), q.y(), q.z(), q.w(), heading, attitude, bank);
	}
	
	static void fromQuaternion(const Eigen::Quaterniond& q, double& heading,
		double& attitude, double& bank) {
		fromQuaternion(q.x(), q.y(), q.z(), q.w(), heading, attitude, bank);
	}
	
	static void toQuaternion(double heading, double attitude, double bank,
		double& x, double& y, double& z, double& w) {
		//y=z w=w x=y z=x
		
		double c1 = cos(heading / 2.0);
		double c2 = cos(attitude / 2.0);
		double c3 = cos(bank / 2.0);
		double s1 = sin(heading / 2.0);
		double s2 = sin(attitude / 2.0);
		double s3 = sin(bank / 2.0);
		
		w = c1*c2*c3 - s1*s2*s3;
		y = s1*s2*c3 + c1*c2*s3;
		z = s1*c2*c3 + c1*s2*s3;
		x = c1*s2*c3 - s1*c2*s3;
	}
	
	static tf::Quaternion toTfQuaternion(double heading, double attitude, double bank) {
		double x, y, z, w;
		toQuaternion(heading, attitude, bank, x, y, z, w);
		return tf::Quaternion(x, y, z, w);
	}
	
	static Eigen::Quaterniond toEigenQuaternion(double heading, double attitude, double bank) {
		double x, y, z, w;
		toQuaternion(heading, attitude, bank, x, y, z, w);
		return Eigen::Quaterniond(w, x, y, z);
	}
};

#endif
