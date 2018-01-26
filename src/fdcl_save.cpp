#include "fdcl_save.h"

using namespace std;

fdcl_save::fdcl_save()
{
	buf_line_full=false;
	buf_line_loc=0;
};
fdcl_save::fdcl_save(string filename)
{
	buf_line_loc=0;
	this->filename=filename;
	open(filename);	
};

fdcl_save::~fdcl_save()
{
	fd.close();
};

void fdcl_save::open(string filename)
{
	this->filename=filename;
	fd.open(filename.c_str());
	cout << "fdcl_save: '" << filename << "' opened" << std::endl;
}

void fdcl_save::write(double d)
{
	if(!buf_line_full)
		buf_line_loc+=snprintf(buf_line+buf_line_loc, MAX_BUF_LINE-buf_line_loc, "%e, ",d);		

	is_buf_line_full();
};

void fdcl_save::write(float f)
{
	if(!buf_line_full)
		buf_line_loc+=snprintf(buf_line+buf_line_loc, MAX_BUF_LINE-buf_line_loc, "%e, ",f);		

	is_buf_line_full();
};


void fdcl_save::write(int i)
{
	if(!buf_line_full)
		buf_line_loc+=snprintf(buf_line+buf_line_loc, MAX_BUF_LINE-buf_line_loc, "%d, ",i);		
	is_buf_line_full();
};

template<typename Derived>
void fdcl_save::write(Eigen::MatrixBase<Derived>& M)
{
	int i,j;
	for(i=0;i<M.rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{			
			if(!buf_line_full)
				buf_line_loc+=snprintf(buf_line+buf_line_loc, MAX_BUF_LINE-buf_line_loc, "%e, ",M(i,j));		

			is_buf_line_full();
		}		
	}
};

void fdcl_save::endl()
{
	if(!buf_line_full)
		buf_line_loc+=snprintf(buf_line+buf_line_loc, MAX_BUF_LINE-buf_line_loc, "\n");		

	is_buf_line_full();

	// save the line to the stringstream buffer and start a new line
	if(!buf_line_full)
		buf.write(buf_line,buf_line_loc);
	else
		buf.write(buf_line,MAX_BUF_LINE);
		
	buf_line_loc=0;
};


void fdcl_save::close()
{
	fd << buf.str();
	fd.close();	

	cout << "fdcl_save: saved to '" << filename << "'"<< std::endl;
	
}

void fdcl_save::is_buf_line_full()
{	
	if (buf_line_loc > MAX_BUF_LINE)
	{
		cout << "fdcl_save: buffer truncated, increase MAX_BUF_LINE at fdcl_save.h at least to " << buf_line_loc << std::endl;
		buf_line_full = true;
	}	
}

template void fdcl_save::write( Eigen::MatrixBase<Eigen::Matrix<double,3,1> > & M);
template void fdcl_save::write( Eigen::MatrixBase<Eigen::Matrix<double,3,3> > & M);
template void fdcl_save::write( Eigen::MatrixBase<Eigen::Matrix<double,4,1> > & M);



