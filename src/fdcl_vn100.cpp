#include "fdcl_vn100.h"

fdcl_vn100::fdcl_vn100()
{
}

fdcl_vn100::~fdcl_vn100()
{
}

void fdcl_vn100::load_config(fdcl_param& cfg)
{
	cfg.read("IMU.port",port);
	cfg.read("IMU.baud_rate",baud_rate);

	cout << "IMU: port " << port << endl;
	cout << "IMU: baud_rate " << baud_rate << endl;
}

void fdcl_vn100::open()
{
	open(port,baud_rate);
}

void fdcl_vn100::open(string port, const int baud_rate)
{
	this->port=port;
	this->baud_rate=baud_rate;


	errorCode = vn100_connect(&vn100,port.c_str(),baud_rate);
	if(errorCode == VNERR_NO_ERROR)
		printf("fdcl_vn100: connected\n");
	else
		printf("fdcl_vn100: ERROR: IMU cannot be connected (VN_ERROR_CODE: %d)\n",errorCode);

	// Disable ASCII asynchronous messages 
	errorCode = vn100_setAsynchronousDataOutputType(
        &vn100,
        VNASYNC_OFF,
        true);
	
	// Configure the binary messages output
	errorCode = vn100_setBinaryOutput1Configuration(
		&vn100,
		BINARY_ASYNC_MODE_SERIAL_2,	// This should be the one we are connected to now. 
		4,							// Outputting binary data at 200 Hz (800 Hz on-board filter / 4 = 200 Hz). 
		BG1_YPR | BG1_ANGULAR_RATE | BG1_ACCEL,
		BG3_NONE,
		BG5_NONE,//BG5_LINEAR_ACCEL_NED,
		true);
	if(errorCode == VNERR_NO_ERROR)
		printf("fdcl_vn100: IMU configured \n");
	else
		printf("fdcl_vn100: ERROR: IMU cannot be configured (VN_ERROR_CODE: %d)\n",errorCode);


	// Now register to receive notifications when a new asynchronous binary packet is received. 
	errorCode = vn100_registerAsyncDataReceivedListener(&vn100, &fdcl_vn100::callback);

}

void fdcl_vn100::close()
{
	errorCode = vn100_unregisterAsyncDataReceivedListener(&vn100, &fdcl_vn100::callback);	
	errorCode = vn100_disconnect(&vn100);
	printf("fdcl_vn100: disconnected\n");
}

void fdcl_vn100::callback(void* sender, VnDeviceCompositeData* data)
{
	Vector3 YPR, W_i, a_i;
	Matrix3 R_ni;

	YPR(0)=data->ypr.yaw*M_PI/180.;
	YPR(1)=data->ypr.pitch*M_PI/180.;
	YPR(2)=data->ypr.roll*M_PI/180.;

	R_ni(0,0)=cos(YPR(0))*cos(YPR(1));
	R_ni(0,1)=cos(YPR(0))*sin(YPR(2))*sin(YPR(1)) - cos(YPR(2))*sin(YPR(0));
	R_ni(0,2)=sin(YPR(0))*sin(YPR(2)) + cos(YPR(0))*cos(YPR(2))*sin(YPR(1));
	R_ni(1,0)=cos(YPR(1))*sin(YPR(0));
	R_ni(1,1)=cos(YPR(0))*cos(YPR(2)) + sin(YPR(0))*sin(YPR(2))*sin(YPR(1));
	R_ni(1,2)=cos(YPR(2))*sin(YPR(0))*sin(YPR(1)) - cos(YPR(0))*sin(YPR(2));
	R_ni(2,0)=-sin(YPR(1));
	R_ni(2,1)=cos(YPR(1))*sin(YPR(2));
	R_ni(2,2)=cos(YPR(2))*cos(YPR(1));	

	W_i(0)=data->angularRate.c0;
	W_i(1)=data->angularRate.c1;
	W_i(2)=data->angularRate.c2;
	
	a_i(0)=data->acceleration.c0;
	a_i(1)=data->acceleration.c1;
	a_i(2)=data->acceleration.c2;	

	UAV.callback_IMU(a_i, W_i, R_ni);
}   

