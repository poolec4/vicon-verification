#ifndef _FDCL_CONTROL_H
#define _FDCL_CONTROL_H

#include "Eigen/Dense"
#include "main_Jetson.h"
#include "fdcl_param.h"

class fdcl_control
{
public:
	double t, t_pre, dt;
	double kR, kW, kI, m, c2;
	Matrix3 J;
	Vector3 eR, eW, eI, M, eI_integrand_pre, eI_integrand;
	double c_tf, l;
	Eigen::Matrix<double, 4, 4> A, Ainv;
	Eigen::Matrix<double, 4, 1> f_motor, fM;
	double f_total; // total thurst for attitude control 

	fdcl_control();
	~fdcl_control();
	void init(void );
	void load_config(fdcl_param& );
	Vector3 attitude_control(double, Matrix3, Vector3, Matrix3, Vector3, Vector3);
	double gettime();	
private:
	struct timespec tspec_INIT, tspec_curr;
};

#endif
