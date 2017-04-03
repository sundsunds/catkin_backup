#include <iostream>
#include <tf/tf.h>
#include <cmath>

using namespace tf;
using namespace std;

double to_deg(double d) {
	return d / M_PI * 180.0;
}

int main(int, char**) {
	for(float f=0; f<360; f+=60) {
	//Quaternion q(0.2, 0.3, 0.7, 0.1);
	Quaternion q(0.1, 0.0, 0.9, 0.1);
	q.normalize();
	q.setEulerZYX(f/180.0*M_PI,
		10.0/180.0*M_PI,
		5.0/180.0*M_PI);
	cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
	
	Matrix3x3 R(q);
    double roll, pitch, yaw;
    R.getEulerYPR(yaw, pitch, roll);
    cout << "Bullet: " << to_deg(yaw) << ", " << to_deg(pitch) << ", " << to_deg(roll) << endl;
    
    // See http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
    /*double heading = atan2(2*q.z()*q.w()-2*q.x()*q.y() , 1 - 2*q.z()*q.z() - 2*q.y()*q.y()); // like yaw
    double attitude = asin(2*q.x()*q.z() + 2*q.y()*q.w()); // like pitch
    double bank =  atan2(2*q.x()*q.w()-2*q.z()*q.y() , 1 - 2*q.x()*q.x() - 2*q.y()*q.y()); // like roll*/
    
    double heading = atan2(2*q.z()*q.w()-2*q.y()*q.x() , 1 - 2*q.z()*q.z() - 2*q.x()*q.x()); // like yaw
    double attitude = asin(2*q.y()*q.z() + 2*q.x()*q.w()); // like pitch
    double bank =  atan2(2*q.y()*q.w()-2*q.z()*q.x() , 1 - 2*q.y()*q.y() - 2*q.x()*q.x()); // like roll
    
    cout << "Bryan: " << to_deg(heading) << ", " << to_deg(attitude) << ", " << to_deg(bank) << endl;
	cout << endl;
	}
	
	return 0;
}
