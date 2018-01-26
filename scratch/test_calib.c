#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include <iostream>
#include "misc_matrix_func.h"

Matrix3 R_nv, R_mi;

void measurement(Matrix3& R_ni, Matrix3& R_vm, double t)
{
	R_vm << cos(t), -cos(t)*sin(t), sin(t)*sin(t),
	    cos(t)*sin(t), pow(cos(t),3) - pow(sin(t),2), -cos(t)*sin(t) - pow(cos(t),2)*sin(t),
	    pow(sin(t),2), cos(t)*sin(t) + pow(cos(t),2)*sin(t), pow(cos(t),2) - cos(t)*pow(sin(t),2);
	R_ni=R_nv*R_vm*R_mi;
}
int main(void)
{
	Vector3 v(0.1, 0.2, 0.3);
	Vector3 v1(0.1, 1.2, -0.3);
	Vector3 v2(M_PI, 4.0, 2.0);
	Vector3 v3(-1.5*M_PI, 0.0, 0.0);
/*	cout << vee(hat(v)) << endl;
	cout << expm_SO3(v) << endl;
	
	
	cout << logm_SO3(expm_SO3(v)) << endl;
	cout << logm_SO3(expm_SO3(v1)) << endl;
	cout << logm_SO3(expm_SO3(v2)) << endl;
	cout << logm_SO3(expm_SO3(v3)) << endl;

	
*/	
	R_nv=expm_SO3(v1);
	R_mi=expm_SO3(v2);
	
	int N=10, i, j;
	Matrix3 ZX,ZY,S,R_nv_new,R_mi_new;	
	Matrix3 *R_ni = new Matrix3[N];
	Matrix3 *R_vm = new Matrix3[N];
	Vector3 r_ij,q_ij;
	ZX.setZero();
	ZY.setZero();
	
	for(i=0;i<N;i++)
		measurement(R_ni[i],R_vm[i],0.1*((double) i));
		
	for(i=0;i<N;i++)
	{
		for(j=0;j<N;j++)
		{
			r_ij=logm_SO3(R_ni[i]*R_ni[j].transpose());
			q_ij=logm_SO3(R_vm[i]*R_vm[j].transpose());
			ZX+=r_ij*q_ij.transpose();
			r_ij=logm_SO3(R_ni[j].transpose()*R_ni[i]);
			q_ij=logm_SO3(R_vm[j].transpose()*R_vm[i]);
			ZY+=r_ij*q_ij.transpose();
			
		}
	}
		
	Eigen::JacobiSVD<Matrix3> svd_ZX(ZX, Eigen::ComputeFullU | Eigen::ComputeFullV);
	S.setIdentity();
	S(2,2)=svd_ZX.matrixU().determinant()*svd_ZX.matrixV().determinant();
	R_nv_new=svd_ZX.matrixU()*S*svd_ZX.matrixV().transpose();

	Eigen::JacobiSVD<Matrix3> svd_ZY(ZY, Eigen::ComputeFullU | Eigen::ComputeFullV);	
	S.setIdentity();
	S(2,2)=svd_ZY.matrixU().determinant()*svd_ZY.matrixV().determinant();
	R_mi_new=svd_ZY.matrixV()*S*svd_ZY.matrixU().transpose();
	
	
	
	cout << R_nv << endl;
	cout << (R_nv-R_nv_new).norm() << endl << endl;

	cout << R_mi << endl;	
	cout << (R_mi_new-R_mi).norm() << endl << endl;
	
	
	return 0;
}