#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>

#include <iomanip>
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using namespace std;

typedef Eigen::Matrix<double, 3, 3> Matrix3l;

// macros for packing floats and doubles:
#define pack754_16(f) (pack754((f), 16, 5))
#define pack754_32(f) (pack754((f), 32, 8))
#define pack754_64(f) (pack754((f), 64, 11))
#define unpack754_16(i) (unpack754((i), 16, 5))
#define unpack754_32(i) (unpack754((i), 32, 8))
#define unpack754_64(i) (unpack754((i), 64, 11))

/*
** pack754() -- pack a floating point number into IEEE-754 format
*/ 
unsigned long long int pack754(long double f, unsigned bits, unsigned expbits)
{
	long double fnorm;
	int shift;
	long long sign, exp, significand;
	unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	if (f == 0.0) return 0; // get this special case out of the way

	// check sign and begin normalization
	if (f < 0) { sign = 1; fnorm = -f; }
	else { sign = 0; fnorm = f; }

	// get the normalized form of f and track the exponent
	shift = 0;
	while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
	while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
	fnorm = fnorm - 1.0;

	// calculate the binary form (non-float) of the significand data
	significand = fnorm * ((1LL<<significandbits) + 0.5f);

	// get the biased exponent
	exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

	// return the final answer
	return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

/*
** unpack754() -- unpack a floating point number from IEEE-754 format
*/ 
long double unpack754(unsigned long long int i, unsigned bits, unsigned expbits)
{
	long double result;
	long long shift;
	unsigned bias;
	unsigned significandbits = bits - expbits - 1; // -1 for sign bit

	if (i == 0) return 0.0;

	// pull the significand
	result = (i&((1LL<<significandbits)-1)); // mask
	result /= (1LL<<significandbits); // convert back to float
	result += 1.0f; // add the one back on

	// deal with the exponent
	bias = (1<<(expbits-1)) - 1;
	shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
	while(shift > 0) { result *= 2.0; shift--; }
	while(shift < 0) { result /= 2.0; shift++; }

	// sign it
	result *= (i>>(bits-1))&1? -1.0: 1.0;

	return result;
}

/*
** packi16() -- store a 16-bit int into a char buffer (like htons())
*/ 
void packi16(unsigned char *buf, unsigned int i)
{
	*buf++ = i>>8; *buf++ = i;
}

/*
** packi32() -- store a 32-bit int into a char buffer (like htonl())
*/ 
void packi32(unsigned char *buf, unsigned long int i)
{
	*buf++ = i>>24; *buf++ = i>>16;
	*buf++ = i>>8;  *buf++ = i;
}

/*
** packi64() -- store a 64-bit int into a char buffer (like htonl())
*/ 
void packi64(unsigned char *buf, unsigned long long int i)
{
	*buf++ = i>>56; *buf++ = i>>48;
	*buf++ = i>>40; *buf++ = i>>32;
	*buf++ = i>>24; *buf++ = i>>16;
	*buf++ = i>>8;  *buf++ = i;
}

/*
** unpacki16() -- unpack a 16-bit int from a char buffer (like ntohs())
*/ 
int unpacki16(unsigned char *buf)
{
	unsigned int i2 = ((unsigned int)buf[0]<<8) | buf[1];
	int i;

	// change unsigned numbers to signed
	if (i2 <= 0x7fffu) { i = i2; }
	else { i = -1 - (unsigned int)(0xffffu - i2); }

	return i;
}

/*
** unpacku16() -- unpack a 16-bit unsigned from a char buffer (like ntohs())
*/ 
unsigned int unpacku16(unsigned char *buf)
{
	return ((unsigned int)buf[0]<<8) | buf[1];
}

/*
** unpacki32() -- unpack a 32-bit int from a char buffer (like ntohl())
*/ 
long int unpacki32(unsigned char *buf)
{
	unsigned long int i2 = ((unsigned long int)buf[0]<<24) |
		((unsigned long int)buf[1]<<16) |
			((unsigned long int)buf[2]<<8)  |
				buf[3];
	long int i;

	// change unsigned numbers to signed
	if (i2 <= 0x7fffffffu) { i = i2; }
	else { i = -1 - (long int)(0xffffffffu - i2); }

	return i;
}

/*
** unpacku32() -- unpack a 32-bit unsigned from a char buffer (like ntohl())
*/ 
unsigned long int unpacku32(unsigned char *buf)
{
	return ((unsigned long int)buf[0]<<24) |
		((unsigned long int)buf[1]<<16) |
			((unsigned long int)buf[2]<<8)  |
				buf[3];
}

/*
** unpacki64() -- unpack a 64-bit int from a char buffer (like ntohl())
*/ 
long long int unpacki64(unsigned char *buf)
{
	unsigned long long int i2 = ((unsigned long long int)buf[0]<<56) |
		((unsigned long long int)buf[1]<<48) |
			((unsigned long long int)buf[2]<<40) |
				((unsigned long long int)buf[3]<<32) |
					((unsigned long long int)buf[4]<<24) |
						((unsigned long long int)buf[5]<<16) |
							((unsigned long long int)buf[6]<<8)  |
								buf[7];
	long long int i;

	// change unsigned numbers to signed
	if (i2 <= 0x7fffffffffffffffu) { i = i2; }
	else { i = -1 -(long long int)(0xffffffffffffffffu - i2); }

	return i;
}

/*
** unpacku64() -- unpack a 64-bit unsigned from a char buffer (like ntohl())
*/ 
unsigned long long int unpacku64(unsigned char *buf)
{
	return ((unsigned long long int)buf[0]<<56) |
		((unsigned long long int)buf[1]<<48) |
			((unsigned long long int)buf[2]<<40) |
				((unsigned long long int)buf[3]<<32) |
					((unsigned long long int)buf[4]<<24) |
						((unsigned long long int)buf[5]<<16) |
							((unsigned long long int)buf[6]<<8)  |
								buf[7];
}

/*
** pack() -- store data dictated by the format string in the buffer
**
**   bits |signed   unsigned   float   string
**   -----+----------------------------------
**      8 |   c        C         
**     16 |   h        H         f
**     32 |   l        L         d
**     64 |   q        Q         g
**      - |                               s
**
**  (16-bit unsigned length is automatically prepended to strings)
*/ 

unsigned int pack(unsigned char *buf, double d)
{
	unsigned long long int fhold;
	unsigned int size = 0;
		
	fhold = pack754_32(d); // convert to IEEE 754
	packi32(buf, fhold);
	buf += 4;	
	size += 4;	
	return size;	
}

template<typename Derived>
unsigned int pack_eigen_matrix(unsigned char *buf, int& current_buf_loc, Eigen::MatrixBase<Derived>& M)
{
	unsigned long long int fhold;
	unsigned int size = 0;
	int i,j;

	typedef typename Eigen::MatrixBase<Derived>::Scalar type;

	switch(*typeid(type).name())
	{
		case 'd': // double
		for(i=0;i<M.rows();i++)
		{
			for(j=0;j<M.cols();j++)
			{		
				fhold = pack754_32(M(i,j)); // convert to IEEE 754
				packi32(buf+current_buf_loc, fhold);
				current_buf_loc+=4;		
			}
		}
		break;
			
		case 'f': // float
		for(i=0;i<M.rows();i++)
		{
			for(j=0;j<M.cols();j++)
			{			
				fhold = pack754_16(M(i,j)); // convert to IEEE 754
				packi16(buf+current_buf_loc, fhold);
				current_buf_loc+=2;		
			}
		}
		break;
	}
	return size;
}

template<typename Derived>
void unpack_eigen_matrix(unsigned char *buf, int& current_buf_loc, Eigen::MatrixBase<Derived>& M)
{
	unsigned long long int fhold;
	double d;
	float f;
	int i,j;

	typedef typename Eigen::MatrixBase<Derived>::Scalar type;

	std::cout << typeid(type).name() << std::endl;
	switch(*typeid(type).name())
	{
			
		case 'd': // double-32
		for(i=0;i<M.rows();i++)
		{
			for(j=0;j<M.cols();j++)
			{			
				fhold = unpacku32(buf+current_buf_loc);
				d = unpack754_32(fhold);
				current_buf_loc+=4;
				M(i,j)=d;		
			}
			
		}
		break;

		case 'f': // float-16
		for(i=0;i<M.rows();i++)
		{
			for(j=0;j<M.cols();j++)
			{			
				fhold = unpacku16(buf+current_buf_loc);
				f = unpack754_16(fhold);
				current_buf_loc+=2;
				M(i,j)=f;		
			}
			
		}
		break;

	}
}

int main(void)
{
	
	unsigned char buf[1024];
	double d=3.141592;
	double d2;
	int current_buf_loc;
	Matrix3l J, K;
	Eigen::Matrix<double, 3, 1> vec;
	Eigen::Matrix<float, 3, 3> J_f;
		
	//	pack(buf,d);//	
	//	unpack(buf,&d2);
	//	printf("%f\n",d2);

	J << 1.123, 2.123, 3.,
	4., 5., 6.321,
	7., 8., 9.;
	J_f << 1.123, 2.123, 3.,
	4., 5., 6.321,
	7., 8., 9.;

	K << 3.123, 2.123, 3.,
	6., 5., 6.321,
	4., 1.123, 9.;
	vec << 0.123456789, 1.5, 183.;


	cout << setprecision(10);

	cout << vec << endl;
	current_buf_loc=0;
	pack_eigen_matrix(buf,current_buf_loc,J);
	pack_eigen_matrix(buf,current_buf_loc,vec);
	pack_eigen_matrix(buf,current_buf_loc,J_f);
	pack_eigen_matrix(buf,current_buf_loc,K);
	
	J.setZero();
	J_f.setZero();
	K.setZero();
	vec.setZero();

	current_buf_loc=0;
	unpack_eigen_matrix(buf,current_buf_loc,J);
	unpack_eigen_matrix(buf,current_buf_loc,vec);
	unpack_eigen_matrix(buf,current_buf_loc,J_f);
	unpack_eigen_matrix(buf,current_buf_loc,K);

	cout << setprecision(10);
	
	std::cout << J << std::endl;
	std::cout << vec << std::endl;
	std::cout << J_f << std::endl;
	std::cout << K << std::endl;

	std::cout << current_buf_loc << std::endl;
	std::cout << sizeof(buf) << std::endl;
	return 0;
}