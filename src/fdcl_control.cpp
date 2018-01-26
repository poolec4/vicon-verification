#include "fdcl_control.h"
#include "fdcl_param.h"
#include "misc_matrix_func.h"
 
fdcl_control::fdcl_control()
{
	//fdcl_control::load_gain();
		
	init();
	kR=0.0/0.0;
	kW=0.0/0.0;
	kI=0.0/0.0;	
	clock_gettime(CLOCK_REALTIME, &tspec_INIT);

};

fdcl_control::~fdcl_control()
{
  //
};

void fdcl_control::init(void )
{
	eR.setZero();
	eW.setZero();
	eI.setZero();
	eI_integrand_pre.setZero();
	eI_integrand.setZero();
	M.setZero();
	f_motor.setZero();
	fM.setZero();

}

void fdcl_control::load_config(fdcl_param& cfg)
{
	cfg.read("Control.kR",kR);
	cfg.read("Control.kW",kW);
	cfg.read("Control.kI",kI);
	cfg.read("Control.c2",c2);
	cfg.read("Control.c_tf",c_tf);
	cfg.read("Control.l",l);	
	cfg.read("Control.f_total",f_total);	
	
	cout << "CTRL: kR = " << kR << endl;
	cout << "CTRL: kW = " << kW << endl;
	cout << "CTRL: kI = " << kI << endl;
	cout << "CTRL: c2 = " << kI << endl;
	cout << "CTRL: c_tf = " << c_tf << endl;
	cout << "CTRL: l = " << l << endl;
	cout << "CTRL: f_total = " << f_total << endl;
	
	cfg.read("UAV.m", m);
	cfg.read("UAV.J", J);

	cout << "CTRL: m = " << m << endl;
	cout << "CTRL: J = " << J << endl;	
	
	A << 1.0, 1.0, 1.0, 1.0,
		0.0, -l, 0.0, l,
		l, 0.0, -l, 0.0,
		-c_tf, c_tf, -c_tf, c_tf;
	Ainv=A.inverse();
	
};

Vector3 fdcl_control::attitude_control(double dt, Matrix3 R, Vector3 W, Matrix3 Rd, Vector3 Wd, Vector3 Wd_dot)
{

	eR=1./2.*vee(Rd.transpose()*R-R.transpose()*Rd);
	eW=W-R.transpose()*Rd*Wd;
	eI_integrand_pre=eI_integrand;
	eI_integrand=eW+c2*eR;
	//eI_integrand=c2*eR;
	eI+=(eI_integrand_pre+eI_integrand)*dt/2.0;
	
	M=-kR*eR-kW*eW-kI*eI + hat(R.transpose()*Rd*Wd)*J*R.transpose()*Rd*Wd + J*R.transpose()*Rd*Wd_dot;
	
	fM(0)=f_total;
	fM.block<3,1>(1,0)=M;
	f_motor=Ainv*fM;
	return M;
};

double fdcl_control::gettime()
{
	double t;
	clock_gettime(CLOCK_REALTIME, &tspec_curr);
	t=(double) tspec_curr.tv_sec+ ((double)tspec_curr.tv_nsec)/1.e9;	
	t-=(double) tspec_INIT.tv_sec+ ((double)tspec_INIT.tv_nsec)/1.e9;
	return t;
}

