#ifndef _FDCL_SERIAL_H
#define _FDCL_SERIAL_H

#include <iomanip>
#include <iostream>
#include <vector>
#include <typeinfo>
#include <stdio.h>

#include "Eigen/Dense"

using namespace std;

class fdcl_serial
{
public:
//	unsigned char* buf=new unsigned char[0];
	vector<unsigned char> buf;
	
	unsigned int loc;
	
	fdcl_serial();
	fdcl_serial(unsigned char* buf_received, int size);
	~fdcl_serial();
	void clear();
	void init(unsigned char* buf_received, int size);
	void reserve(int size);
	void pack(double&);
	void pack(int&);
	void pack(float&);
	void pack(bool&);
	template<typename Derived>
	void pack(Eigen::MatrixBase<Derived>& M);
	
	void unpack(double&);
	void unpack(int&);
	void unpack(float&);
	void unpack(bool&);
	template<typename Derived>
	void unpack(Eigen::MatrixBase<Derived>& M);
	
	int size();
	unsigned char* data();

	// the following functions are copied from http://beej.us/guide/bgnet/html/multi/advanced.html#serialization
private:
	unsigned long long int pack754(long double f, unsigned bits, unsigned expbits);
	long double unpack754(unsigned long long int i, unsigned bits, unsigned expbits);
	void packi16(unsigned char *buf, unsigned int i);
	void packi32(unsigned char *buf, unsigned long int i);
	void packi64(unsigned char *buf, unsigned long long int i);
	int unpacki16(unsigned char *buf);
	unsigned int unpacku16(unsigned char *buf);
	long int unpacki32(unsigned char *buf);
	unsigned long int unpacku32(unsigned char *buf);
	long long int unpacki64(unsigned char *buf);
	unsigned long long int unpacku64(unsigned char *buf);	
};

#endif