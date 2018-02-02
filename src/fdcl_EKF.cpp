#include "fdcl_EKF.h"

fdcl_EKF::fdcl_EKF()
{
	clock_gettime(CLOCK_REALTIME, &tspec_INIT);
	init();
}

void fdcl_EKF::init()
{
	x.setZero();	v.setZero();	a.setZero();	a_pre.setZero();
	W.setZero();	W_pre.setZero();	R.setIdentity();	R_pre.setIdentity();
	b_a.setZero();	b_a_pre.setZero();	b_W.setZero();	b_W_pre.setZero();
	a_i_pre.setZero(); W_i_pre.setZero();

	// initialize covariance P
	P.setZero();
	// initial covariance for x
	P(0,0)=pow(0.1,2);
	P(1,1)=pow(0.1,2);
	P(2,2)=pow(0.1,2);
	// initial covariance for v
	P(3,3)=pow(0.1,2);
	P(4,4)=pow(0.1,2);
	P(5,5)=pow(0.1,2);
	// initial covariance for R
	P(6,6)=pow(10.*M_PI/180.,2);
	P(7,7)=pow(10.*M_PI/180.,2);
	P(8,8)=pow(10.*M_PI/180.,2);
	// initial covariance for b_a
	P(9,9)=pow(1.,2);
	P(10,10)=pow(1.,2);
	P(11,11)=pow(1.,2);
	// initial covariance for b_W
	P(12,12)=pow(1.,2);
	P(13,13)=pow(1.,2);
	P(14,14)=pow(1.,2);

	// IMU covariacne for W	measurement
	Q_W.setZero();
	Q_W(0,0)=pow(0.1,2);
	Q_W(1,1)=pow(0.1,2);
	Q_W(2,2)=pow(0.1,2);

	// IMU covariance for acceleration measurement
	Q_a.setZero();
	Q_a(0,0)=pow(1,2);
	Q_a(1,1)=pow(1,2);
	Q_a(2,2)=pow(1,2);

	// IMU covariacne for acceleration bias	b_a_dot
	Q_b_a.setZero();
	Q_b_a(0,0)=pow(0.01,2);
	Q_b_a(1,1)=pow(0.01,2);
	Q_b_a(2,2)=pow(0.01,2);

	// IMU covariacne for acceleration bias	b_W_dot
	Q_b_W.setZero();
	Q_b_W(0,0)=pow(0.01,2);
	Q_b_W(1,1)=pow(0.01,2);
	Q_b_W(2,2)=pow(0.01,2);

	// IMU covariance for angles
	V_R_ni.setZero();
	V_R_ni(0,0)=pow(5.*M_PI/180.,2);
	V_R_ni(1,1)=pow(5.*M_PI/180.,2);
	V_R_ni(2,2)=pow(5.*M_PI/180.,2);

	// VICON covariance for X
	V_x.setZero();
	V_x(0,0)=pow(0.02,2);
	V_x(1,1)=pow(0.02,2);
	V_x(2,2)=pow(0.02,2);

	// VICON covaraiance for R
	V_R_vm.setZero();
	V_R_vm(0,0)=pow(2.*M_PI/180.,2);
	V_R_vm(1,1)=pow(2.*M_PI/180.,2);
	V_R_vm(2,2)=pow(2.*M_PI/180.,2);

	R_fv << 0., 1., 0.,
	 		1., 0., 0.,
	 		0., 0., -1.;
	R_bi << cos(M_PI/4.), -sin(M_PI/4.), 0.,
			sin(M_PI/4.), cos(M_PI/4.), 0.,
			0., 0., 1.;

	R_nv.setIdentity();
	R_mi.setIdentity();
	g_ave=9.81;

	// GPS measurements
	llh.setZero();
	llh_prev.setZero();
	S_llh_h = 0.0;
	S_llh_v = 0.0;

	rtk_x.setZero();
	S_rtk_x_h = 0.0;
  S_rtk_x_v = 0.0;

	rtk_v.setZero();
	S_rtk_v_h = 0.0;
  S_rtk_v_v = 0.0;

	utc = 0.0;
	status = 0;
	sats = 0;

}

void fdcl_EKF::reset(fdcl_param& cfg)
{
	init();
	load_config(cfg);
}

void fdcl_EKF::load_config(fdcl_param& cfg)
{
	cfg.read("CAL.R_nv",R_nv);
	cfg.read("CAL.R_mi",R_mi);
	cfg.read("CAL.g_ave",g_ave);

	cout << "CAL: R_nv " << R_nv << endl;
	cout << "CAL: R_mi " << R_mi << endl;
	cout << "CAL: g_ave " << g_ave << endl;

}

double fdcl_EKF::gettime()
{
	double t;
	clock_gettime(CLOCK_REALTIME, &tspec_curr);
	t=(double) tspec_curr.tv_sec+ ((double)tspec_curr.tv_nsec)/1.e9;
	t-=(double) tspec_INIT.tv_sec+ ((double)tspec_INIT.tv_nsec)/1.e9;
	return t;
}



void fdcl_EKF::callback_IMU(Vector3 a_i, Vector3 W_i, Matrix3 R_ni)
{
	t_IMU_pre=t_IMU;
	t_IMU=gettime();
	dt_IMU=t_IMU-t_IMU_pre;

//	cout << "fdcl_EKF::callback_IMU   " << t_IMU << ", " << dt_IMU << endl;
	this->a_i=a_i;
	this->W_i=W_i;
	this->R_ni=R_ni;

	W_b_IMU=R_bi*W_i;
	R_fb_IMU=R_fv * R_nv.transpose() * R_ni * R_bi.transpose();
	a_f_IMU=R_bi*a_i;

	double residual=acos(((R.transpose()*R_fb_IMU).trace()-1.0)/2.0)*180./M_PI;
	if(CALIBRATE_ON==false)
	{
		pthread_mutex_lock(&UAV_data_mutex);
		Estimation_Prediction();
		Estimation_Correction_IMU();
		pthread_mutex_unlock(&UAV_data_mutex);

		//if (residual > 10.)
		//	cout << "WARNING: IMU.R_bf measurement residual too large " << residual << " deg" << endl;

	}

//	cout << "callback_IMU: " << t_IMU << " :" << R_ni << endl;

}

void fdcl_EKF::callback_VICON(Vector3 x_v, Vector4 q_v, Matrix3 R_vm)
{
	t_VICON_pre=t_VICON;
	t_VICON=gettime();
	dt_VICON=t_VICON-t_VICON_pre;

//	cout << "fdcl_EKF::callback_VICON " << t_VICON << ", " << dt_VICON << endl;
	this->x_v=x_v;
	this->q_v=q_v;
	this->R_vm=R_vm;


	x_f_VICON=R_fv*x_v;
	R_fb_VICON=R_fv * R_vm * R_mi * R_bi.transpose();

	double residual_R=acos(((R.transpose()*R_fb_VICON).trace()-1.0)/2.0)*180./M_PI;
	double residual_x=(x-x_f_VICON).norm();
	if(CALIBRATE_ON==false)
	{
		pthread_mutex_lock(&UAV_data_mutex);
		Estimation_Prediction();
		Estimation_Correction_VICON();
		pthread_mutex_unlock(&UAV_data_mutex);

		//if (residual_R > 10.)
		//	cout << "WARNING: VICON.R_bf measurement residual too large " << residual_R << " deg" << endl;
		//if (residual_x > 0.1)
		//	cout << "WARNING: VICON.x_f measurement residual too large " << residual_x << " m" << endl;

	}
}


void fdcl_EKF::Estimation_Prediction()
{
	t_pre=t;
	t=gettime();
	dt=t-t_pre;

//	cout << "fdcl_EKF::prediction " << t << ", " << dt << endl;

	Vector3 e3(0.,0.,1.);
	Matrix3 eye3;
	Matrix15  A, Psi, A_d;
	Eigen::Matrix<double, 15, 12>  F, F_d;
	Eigen::Matrix<double, 12, 12>  Q;

	eye3.setIdentity();

	// Prediction with a_i and W_i
	// Prediction of mean

	b_a_pre=b_a; // save the value of the previous step
	b_W_pre=b_W;
	b_a=b_a_pre; // prediction (can be skipped)
	b_W=b_W_pre;

	W_pre=W;
	W=R_bi * (W_i+b_W);

	R_pre=R;
	R=R_pre*expm_SO3(dt/2.0*(W_pre + W));

	a_pre=a;
	a = R * R_bi * (a_i+b_a)+g_ave*e3;

	x+= dt*v + dt*dt/2.0*a_pre;
	v+= dt/2.0*(a_pre+a);

	// Prediction of covariance
	// (A,F) matrices for the continuous time linearized equation
	A.setZero();
	A.block<3,3>(0,3)=eye3;
	A.block<3,3>(3,6)=-R_pre*hat(R_bi*(a_i_pre+b_a_pre));
	A.block<3,3>(3,9)=R_pre*R_bi;
	A.block<3,3>(6,6)=-hat(R_bi*(W_i_pre+b_W_pre));
	A.block<3,3>(6,12)=R_bi;

	F.setZero();
	F.block<3,3>(3,0)=R_pre*R_bi;
	F.block<3,3>(6,3)=R_bi;
	F.block<3,3>(9,6)=eye3;
	F.block<3,3>(12,9)=eye3;

	// (Ad,Fd) for the discrete time linearized equation
	Psi.setIdentity();
	Psi+=dt/2.0*A;
	A_d.setIdentity();
	A_d+=dt*A*Psi;
	F_d=dt*Psi*F;

	// covariance for w_a, w_\Omega, w_{b_a}, w_{b_\Omega}
	Q.setZero();
	Q.block<3,3>(0,0)=Q_a;
	Q.block<3,3>(3,3)=Q_W;
	Q.block<3,3>(6,6)=Q_b_a;
	Q.block<3,3>(9,9)=Q_b_W;

	P=A_d*P*A_d.transpose()+F_d*Q*F_d.transpose();
}


void fdcl_EKF::Estimation_Correction_IMU()
{
	// correction for observable subspace

	Matrix3 tmp, S, eye3;
	Vector3 delz;
	Eigen::Matrix<double, 15,15>  T, eye15;
	Eigen::Matrix<double, 6, 15>  T_o;
	Eigen::Matrix<double, 3, 15>  H;
	Eigen::Matrix<double, 6,  3>  K_o;
	Eigen::Matrix<double, 6,  1>  delchi;
	Eigen::Matrix<double, 3,  6>  H_o;
	Eigen::Matrix<double, 6,  6>  P_o, eye6;
	Eigen::Matrix<double, 15, 3>  K;

	eye3.setIdentity();
	eye6.setIdentity();
	eye15.setIdentity();

	T.setZero();
	T.block<3,3>(0,6)=eye3;
	T.block<3,3>(3,12)=eye3;
	T.block<3,3>(6,0)=eye3;
	T.block<3,3>(9,3)=eye3;
	T.block<3,3>(12,9)=eye3;
	T_o.setZero();
	T_o.block<3,3>(0,0)=eye3;
	T_o.block<3,3>(3,3)=eye3;


	// Correction with R_ni
	tmp=R.transpose()*R_fv*R_nv.transpose()*R_ni*R_bi.transpose(); // rot from b to b
	delz=vee(tmp-tmp.transpose())/2.0; // residual
	H.setZero();
	H.block<3,3>(0,6)=eye3;
	S=H*P*H.transpose()+R_bi*V_R_ni*R_bi.transpose(); // resiual covariance

	P_o=T_o*T*P*T.transpose()*T_o.transpose();
	H_o=H*T.transpose()*T_o.transpose();
	K_o=P_o*H_o.transpose()*S.inverse();

	delchi=K_o*delz;

	R=R*expm_SO3(delchi.block<3,1>(0,0));
	b_W+=delchi.block<3,1>(3,0);

	K=T.transpose()*T_o.transpose()*K_o;
	P=(eye15-K*H)*P*(eye15-K*H).transpose()+K*R_bi*V_R_ni*R_bi.transpose()*K.transpose();
}


void fdcl_EKF::Estimation_Correction_VICON()
{
	Vector6 delz;
	Matrix3 eye3, tmp;
	Matrix6 G, V, S;

	Eigen::Matrix<double, 15,  1> delx;
	Eigen::Matrix<double, 15,  6> K;
	Eigen::Matrix<double,  6, 15> H;
	Matrix15 eye15;

	eye3.setIdentity();
	eye15.setIdentity();

	tmp=R.transpose() * R_fv * R_vm * R_mi * R_bi.transpose(); // rot from b to b

	delz.block<3,1>(0,0)=vee(tmp-tmp.transpose())/2.0; // residual
	delz.block<3,1>(3,0)=x_v-R_fv.transpose()*x;

	H.setZero();
	H.block<3,3>(0,6)=eye3;
	H.block<3,3>(3,0)=R_fv.transpose();
	G.setZero();
	G.block<3,3>(0,0)=R_bi * R_mi.transpose();
	G.block<3,3>(3,3)=eye3;
	V.setZero();
	V.block<3,3>(0,0)=V_R_vm;
	V.block<3,3>(3,3)=V_x;

	S=H*P*H.transpose() + G*V*G.transpose(); // residual covariance

	K=P*H.transpose()*S.inverse(); // Kalman gain
	delx=K*delz;

	x+=delx.block<3,1>(0,0);
	v+=delx.block<3,1>(3,0);
	R=R*expm_SO3(delx.block<3,1>(6,0));
	b_a+=delx.block<3,1>(9,0);

	b_W+=delx.block<3,1>(12,0);

	P=(eye15-K*H)*P*(eye15-K*H).transpose()+K*G*V*G.transpose()*K.transpose();

}
