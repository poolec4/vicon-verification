#include "fdcl_i2c.h"
#include "fdcl_param.h"
#include "misc_matrix_func.h"

using namespace std;

fdcl_i2c::fdcl_i2c()
{
	thr[0]=0;
	thr[1]=0;
	thr[2]=0;
	thr[3]=0;
	i_test=0;
}

fdcl_i2c::~fdcl_i2c()
{
	close(fhi2c);
}

void fdcl_i2c::write_throttle(bool MOTOR_ON){
	int i;
//	uint8_t  *msg = new uint8_t[24] ;

  // Execute motor output commands


/*	int length = 6;
	char buffer[6];
	uint8_t  *msg = new uint8_t[24] ;
	for(int i=0;i<4;i++)
	{
    	tcflush(fhi2c, TCIOFLUSH);

    	usleep(500);

    	if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
  	 	 	printf("ERROR: ioctl\n");

    	read(fhi2c,buffer,length);
    	//printf("Motor:%d ",i);
    	for(int k=0;k<length;k++)
    	{
      		msg[i*6+k] = ((uint8_t)buffer[k]);
      //printf("%d, ", buffer[k]);
    	}
    //printf("\n");
  	}
*/
  	for(i = 0; i < 4; i++)
  	{
    //printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
    	tcflush(fhi2c, TCIOFLUSH);
    	usleep(500);
    	// if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
    	// 	printf("I2C: ERROR: ioctl motor %d with errno %d\n", i, errno);

    	if(MOTOR_ON)
    	{
    		if(write(fhi2c, &thr[i], 1)!=1)
				    	printf("I2C: ERROR: write the throttle %d to the motor %i\n", thr[i], i);

    	}
  	}

//	return msg;
//	delete[] msg;
}

void fdcl_i2c::write_force(Vector4 f_motor, bool MOTOR_ON)
{
	int i;

	sat(f_motor,0.,14.);

  	for(i = 0; i < 4; i++)
  	{
		thr[i]=round(calib[0]*pow(f_motor(i),2)+calib[1]*f_motor(i)+calib[2]);
		if (thr[i] < 0)
			thr[i]=0;
		else if (thr[i] > 255)
			thr[i]=255;

    	tcflush(fhi2c, TCIOFLUSH);
    	usleep(500);
    	if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
    		printf("I2C: ERROR: ioctl motor %d with errno %d\n", i, errno);

    	if(MOTOR_ON)
    	{
    		if(write(fhi2c, &thr[i], 1)!=1)
				    	printf("I2C: ERROR: write the throttle %d to the motor %i\n", thr[i], i);

    	}
  	}

//	return msg;
//	delete[] msg;
}


void fdcl_i2c::load_config(fdcl_param& file_cfg)
{
	file_cfg.read("I2C.port",port);
	cout << "I2C: port " << port << endl;

	file_cfg.read("I2C.addr0",mtr_addr[0]);
	file_cfg.read("I2C.addr1",mtr_addr[1]);
	file_cfg.read("I2C.addr2",mtr_addr[2]);
	file_cfg.read("I2C.addr3",mtr_addr[3]);

	file_cfg.read("I2C.calib",calib);

	printf("I2C: addr %d, %d, %d, %d\n",mtr_addr[0],mtr_addr[1],mtr_addr[2],mtr_addr[3]);
	cout << "I2C: calib " << calib << endl;
}

void fdcl_i2c::open()
{
	bool motor_check;
	int thr0=0;

  // Open i2c:
	fhi2c = ::open(port.c_str(), O_RDWR);// Chris
	cout << "I2C: Opening i2c port " << port << " ..." << endl;

	if(fhi2c<0)
	{
		if(errno==13)
			printf("I2C: ERROR opening i2c port, errno=%d: permission denied\n",errno);
		else
			printf("I2C: ERROR opening i2c port, errno=%d\n",errno);
	}
	else
		cout << "I2C: The i2c port is open" << endl;

	usleep(100000);
	tcflush(fhi2c, TCIFLUSH);
 	usleep(100000);

  // Call and response from motors
	printf("I2C: Checking motors...\n");

	for(int i=0; i<4; i++)
	{
		motor_check=true;
		if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
		{
        	printf("I2C: ERROR: motor %i ioctl\n", i);
        	motor_check = false;
      	}
		usleep(100);
		if(write(fhi2c, &thr0, 1)!=1)
		{
        	printf("I2C: ERROR: motor %i write\n", i);
        	motor_check = false;
      	}
      	usleep(100);
      	if(motor_check==true)
        	printf("I2C: motor %i working\n", i);
	}

}

void fdcl_i2c::idle(bool MOTOR_ON)
{
	for(int i=0; i<4; i++)
		thr[i]=0;

	write_throttle(MOTOR_ON);

}

void fdcl_i2c::warmup(bool MOTOR_ON)
{
	for(int i=0; i<4; i++)
		thr[i]=30;

	write_throttle(MOTOR_ON);

}

void fdcl_i2c::test(bool MOTOR_ON, bool TEST_ON)
{
	struct timespec tspec_current, tspec_pre;
	double del_t=0.;

	cout << "MOTOR TEST" << endl;

	thr[i_test]=60;
	thr[(i_test+1) % 4]=0;
	thr[(i_test+2) % 4]=0;
	thr[(i_test+3) % 4]=0;

	clock_gettime(CLOCK_REALTIME, &tspec_pre);
	cout << "MOTOR TEST: Motor " << i_test << " :throttle 60 for 3 seconds" << endl;
	del_t=0.;
	while(del_t < 3. && TEST_ON)
	{
		clock_gettime(CLOCK_REALTIME, &tspec_current);

		del_t=(double) tspec_current.tv_sec+ ((double)tspec_current.tv_nsec)/1.e9;
		del_t-=(double) tspec_pre.tv_sec+ ((double)tspec_pre.tv_nsec)/1.e9;

		write_throttle(MOTOR_ON);
	}

	i_test+=1;
	if (i_test >= 4)
		i_test=0;

}


void fdcl_i2c::sat(Eigen::Matrix<double,4,1> &x, double x_min, double x_max)
{
	int i;
	for (i=0; i<4; i++)
	{
		if (x(i)>x_max)
			x(i)=x_max;
		else if (x(i)<x_min)
			x(i)=x_min;
	}
}
