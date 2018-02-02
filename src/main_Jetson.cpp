// Uncomment fdcl_i2C motor error print lines
#define FileName "2018_02_02_zed_verification.txt"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <iostream>
#include <fstream>
#include <sl/Camera.hpp>

#include "Eigen/Dense"

#include "main_Jetson.h"
#include "fdcl_param.h"
#include "fdcl_control.h"
#include "fdcl_i2c.h"
#include "fdcl_wifi.h"
#include "misc_matrix_func.h"
#include "fdcl_EKF.h"
#include "fdcl_vn100.h"
#include "fdcl_vicon.h"

using Eigen::MatrixXd;
using namespace std;
using namespace sl;


bool SYSTEM_ON=true;
bool MOTOR_ON=false;
bool CALIBRATE_ON=false;
bool CALIBRATE_DONE=false;
int  COMMAND_MODE=0;

// ZED includes and definitions
sl::Camera zed;
//sl::Pose camera_pose;
//std::thread zed_callback;
bool quit = false;
// OpenGL window to display camera motion
//GLViewer viewer;

const int MAX_CHAR = 128;

#define NUM_THREADS 3

pthread_mutex_t UAV_data_mutex;

struct COMMAND_type COMM;

fdcl_param file_cfg;
fdcl_EKF UAV;
fdcl_wifi WIFI;
fdcl_vn100 IMU;
fdcl_vicon VICON;
fdcl_i2c MOTOR;
fdcl_control CTRL;

void *data_thread(void *thread_id);
void *vicon_thread(void *thread_id);
void *zed_thread(void *thread_id);

// ZED functions
void startZED();
void run();
void close();
void transformPose(sl::Transform &pose, float tx);

int main()
{
	pthread_t threads[NUM_THREADS];
	pthread_attr_t attr;
	struct sched_param  param;
	int fifo_max_prio, fifo_min_prio, fifo_mid_prio;

	// initialize state
	file_cfg.open("../jetson_SEH.cfg");
	UAV.load_config(file_cfg);
	WIFI.load_config(file_cfg);
	IMU.load_config(file_cfg);
	VICON.load_config(file_cfg);
	MOTOR.load_config(file_cfg);
	CTRL.load_config(file_cfg);

	// Set ZED parameters
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720; // Use HD720 video mode (default fps: 60)
	init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
	init_params.coordinate_units = UNIT_METER; // Set units in meters

	// Open ZED
	zed.open(init_params);
	
	// Enable positional tracking with default parameters
	sl::TrackingParameters tracking_parameters;
	zed.enableTracking(tracking_parameters);
	
	// Initialize mutex and condition variables
	pthread_mutex_init(&UAV_data_mutex, NULL);

	// Set thread attributes
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);

	printf("creating threads...\n");

    // Set configuration parameters for the ZED
    InitParameters initParameters;
    initParameters.camera_resolution = RESOLUTION_HD720;
    initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters.coordinate_units = UNIT_METER;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;


    // Set positional tracking parameters
    TrackingParameters trackingParameters;
    trackingParameters.initial_world_transform = sl::Transform::identity();
    trackingParameters.enable_spatial_memory = true;

    // Start motion tracking
    zed.enableTracking(trackingParameters);

    // Initialize OpenGL viewer
    //viewer.init(zed.getCameraInformation().camera_model);

    // Start ZED callback
    //startZED();

    // Set the display callback
    //glutCloseFunc(close);
    //glutMainLoop();

	param.sched_priority = fifo_max_prio;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[0], &attr, data_thread, (void *) 0);
	pthread_create(&threads[1], &attr, vicon_thread, (void *) 0);
	pthread_create(&threads[2], &attr, zed_thread, (void *) 0);

	pthread_join(threads[0], NULL);
	pthread_join(threads[1], NULL);
	pthread_join(threads[2], NULL);

	pthread_attr_destroy(&attr);
	pthread_mutex_destroy(&UAV_data_mutex);

	file_cfg.close();
	printf("threads closed\n");

	return 0;
}


void *zed_thread(void *thread_id)
{
	printf("ZED: thread initialized..\n");
	ofstream myfile;

	while(SYSTEM_ON == true)
	{
		sl::Pose zed_pose;
		if (zed.grab() == SUCCESS) 
		{
	        // Get the pose of the camera relative to the world frame
	        TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);
	        // Display translation and timestamp
	        printf("ZED   tx: %.3f, ty:  %.3f, tz:  %.3f, timestamp: %llu\n",
	        zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);
	        // Display orientation quaternion
	        printf("Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\n",
	        zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);

			//printf("Vicon tx: %.3f  ty: %.3f  tz: %.3f \n", UAV.x_v(0), UAV.x_v(1), UAV.x_v(2));
		
			myfile.open (FileName,fstream::app);
			// ZED translation
			myfile << std::fixed << std::setprecision(8) << zed_pose.timestamp <<",";
			myfile << std::fixed << std::setprecision(8) << zed_pose.getTranslation().tx <<",";
			myfile << std::fixed << std::setprecision(8) << zed_pose.getTranslation().ty <<",";
			myfile << std::fixed << std::setprecision(8) << zed_pose.getTranslation().tz <<"\n";
			// ZED orientation
			//myfile << std::fixed << std::setprecision(8) << zed_pose.getOrientation().ox <<",";
			//myfile << std::fixed << std::setprecision(8) << zed_pose.getOrientation().oy <<",";
			//myfile << std::fixed << std::setprecision(8) << zed_pose.getOrientation().oz <<",";
			//myfile << std::fixed << std::setprecision(8) << zed_pose.getOrientation().ow <<",";
			//// VICON translation
			//myfile << std::fixed << std::setprecision(8) << UAV.x_v(0) <<",";
			//myfile << std::fixed << std::setprecision(8) << UAV.x_v(1) <<",";
			//myfile << std::fixed << std::setprecision(8) << UAV.x_v(2) <<",";
			//// VICON orientation
			//myfile << std::fixed << std::setprecision(8) << UAV.q_v(0) <<",";
			//myfile << std::fixed << std::setprecision(8) << UAV.q_v(1) <<",";
			//myfile << std::fixed << std::setprecision(8) << UAV.q_v(2) <<",";
			//myfile << std::fixed << std::setprecision(8) << UAV.q_v(3) <<"\n";

			myfile.close();	
		}
	}

	printf("ZED: thread closing\n");
	pthread_exit(NULL);
}

void *data_thread(void *thread_id)
{
	printf("DATA: thread initialized..\n");


	printf("DATA: thread closing\n");

	pthread_exit(NULL);

}

void *vicon_thread(void *thread_id)
{
	VICON.open();
	printf("VICON: thread initialized..\n");

	while(SYSTEM_ON==true)
	{
		VICON.loop();
	}

	VICON.close();
	printf("VICON: thread closing\n");
	pthread_exit(NULL);

}
