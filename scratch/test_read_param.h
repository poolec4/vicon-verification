#include "Eigen/Dense"

void read_param_double(FILE *fp, const char* var_name, double& value);

template<typename Derived>
void read_param_eigen(FILE *fp, const char* var_name, Eigen::MatrixBase<Derived>& M);

void write_param_double(FILE *fp, const char* var_name, double value);

template<typename Derived>
void write_param_eigen(FILE *fp, const char* var_name, Eigen::MatrixBase<Derived> M);
