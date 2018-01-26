#include <iostream>

#include "vectornav.h"
#include "Eigen/Dense"
#include "fdcl_param.h"
#include "fdcl_EKF.h"
#include "misc_matrix_func.h"

extern fdcl_EKF UAV;

typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 3, 1> Vector3;

class fdcl_vn100
{
public:
	fdcl_vn100();
	~fdcl_vn100();

	string port; //"/dev/ttyTHS2";
	int baud_rate;

	static void callback(void* sender, VnDeviceCompositeData* data);

	void load_config(fdcl_param& ); 
	void open();
	void open(string port, const int baud_rate);
	void close();

private:
	VN_ERROR_CODE errorCode;
	Vn100 vn100;	
};


