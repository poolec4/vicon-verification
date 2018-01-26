#ifndef _FDCL_I2C_H
#define _FDCL_I2C_H

#include <iostream>
#include <termios.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <string>

#include "fdcl_param.h"
#include "Eigen/Dense"

typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 3, 1> Vector3;

using namespace std;

class fdcl_i2c
{
public:
	fdcl_i2c();
  	~fdcl_i2c();

	int	fhi2c;
	int mtr_addr[4]; //= {41, 42, 43, 44};// Motor addresses 1-6
	int thr[4];// i2c motor commands
  	string port; // /dev/i2c-?
	Vector3 calib; // thr=calib[0]*f[i]^2+ calib[1]*f[i]+ calib[2]
	int i_test;	
	
	void load_config(fdcl_param& );
	void write_throttle(bool MOTOR_ON);
	void write_force(Vector4 , bool MOTOR_ON);
	void idle(bool );
	void warmup(bool );
	void open();
	void test(bool ,bool );
private:
	void sat(Eigen::Matrix<double,4,1> &x, double x_min, double x_max);

};

#endif
