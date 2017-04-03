#ifndef PROJECTION_H
#define PROJECTION_H

// Projects or unprojects to/from 3d-space
class Projection {
public:
	Projection(double baseLine, double focalLen, double x0,
		double y0, double sx, double sy)
		:baseLine(baseLine), focalLen(focalLen), x0(x0), y0(y0), sx(sx), sy(sy) {
	}	

	inline void project(double u, double v, double disp,
		double* x, double* y, double* z) const {
		*x = projectU(u, disp);
		*y = projectV(v, disp);
		*z = projectDisp(disp);
	}
	
	inline double projectU(double u, double disp) const {
		return baseLine * (u-x0) / disp;
	}
	
	inline double projectV(double v, double disp) const {
		return baseLine * (y0-v) / disp;
	}
	
	inline double projectDisp(double disp) const {
		return baseLine * (focalLen/sx) / disp;
	}
		
	inline void unproject(double x, double y, double z, double* u,
		double* v, double* disp) const {
		*u = unprojectX(x, z);
		*v = unprojectY(y, z);
		*disp = unprojectZ(z);
	}
	
	inline double unprojectZ(double z) const {
		return baseLine * (focalLen/sx) / z;
	}
	
	inline double unprojectX(double x, double z) const {
		return x0 + focalLen/sx * x/z;
	}
	
	inline double unprojectY(double y, double z) const {
		return y0 - focalLen/sx * y/z;
	}
	
private:
	double baseLine;
	double focalLen;
	double x0;
	double y0;
	double sx;
	double sy;
};

#endif
