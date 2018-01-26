#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include "Eigen/Dense"

#define MAX_BUF_LINE 4096

using namespace std;

class fdcl_save
{
public:
	fdcl_save();
	fdcl_save(string filename);
	~fdcl_save();
	ofstream fd;
	stringstream buf;
	string filename;
	
	void open(string filename);
	void write(double );
	void write(float );
	void write(int );	
	template<typename Derived>	void write(Eigen::MatrixBase<Derived>& M);	

	void endl();
	void close();
private:
	char buf_line[MAX_BUF_LINE];
	int buf_line_loc;
	void is_buf_line_full();
	bool buf_line_full;
};
