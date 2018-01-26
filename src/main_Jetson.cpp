// Uncomment fdcl_i2C motor error print lines

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

bool SYSTEM_ON=true;
bool MOTOR_ON=false;
bool CALIBRATE_ON=false;
bool CALIBRATE_DONE=false;
int  COMMAND_MODE=0;

// ZED includes and definitions

using namespace std;
using namespace sl;

sl::Camera zed;
sl::Pose camera_pose;
std::thread zed_callback;
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

	// Initialize mutex and condition variables
	pthread_mutex_init(&UAV_data_mutex, NULL);

	// Set thread attributes
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
	fifo_mid_prio = (fifo_max_prio + fifo_min_prio) / 2;

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

     float tx = 0, ty = 0, tz = 0;
    float rx = 0, ry = 0, rz = 0;

    // Get the distance between the center of the camera and the left eye
    float translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;

    // Create text for GUI
    char text_rotation[MAX_CHAR];
    char text_translation[MAX_CHAR];

    // Create a CSV file to log motion tracking data
    std::ofstream outputFile;
    std::string csvName = "Motion_data";
    outputFile.open(csvName + ".csv");
    if (!outputFile.is_open())
        cout << "WARNING: Can't create CSV file. Run the application with administrator rights." << endl;
    else
        outputFile << "Timestamp(ns);Rotation_X(rad);Rotation_Y(rad);Rotation_Z(rad);Position_X(m);Position_Y(m);Position_Z(m);" << endl;


    while (!quit && zed.getSVOPosition() != zed.getSVONumberOfFrames() - 1) {
        if (zed.grab() == SUCCESS) {
            // Get the position of the camera in a fixed reference frame (the World Frame)
            TRACKING_STATE tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);

            if (tracking_state == TRACKING_STATE_OK) {
                // getPosition() outputs the position of the Camera Frame, which is located on the left eye of the camera.
                // To get the position of the center of the camera, we transform the pose data into a new frame located at the center of the camera.
                // The generic formula used here is: Pose(new reference frame) = M.inverse() * Pose (camera frame) * M, where M is the transform between two frames.
                transformPose(camera_pose.pose_data, translation_left_to_center); // Get the pose at the center of the camera (baseline/2 on X axis)

                // Update camera position in the viewing window
                //viewer.updateZEDPosition(camera_pose.pose_data);

                // Get quaternion, rotation and translation
                sl::float4 quaternion = camera_pose.getOrientation();
                sl::float3 rotation = camera_pose.getEulerAngles(); // Only use Euler angles to display absolute angle values. Use quaternions for transforms.
                sl::float3 translation = camera_pose.getTranslation();

                // Display translation and rotation (pitch, yaw, roll in OpenGL coordinate system)
                snprintf(text_rotation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", rotation.x, rotation.y, rotation.z);
                snprintf(text_translation, MAX_CHAR, "%3.2f; %3.2f; %3.2f", translation.x, translation.y, translation.z);

                printf("%3.2f; %3.2f; %3.2f", rotation.x, rotation.y, rotation.z);
                printf("%3.2f; %3.2f; %3.2f", translation.x, translation.y, translation.z);

                // Save the pose data in a csv file
                if (outputFile.is_open())
                    outputFile << zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE) << "; " << text_rotation << "; " << text_translation << ";" << endl;
            }

            // Update rotation, translation and tracking state values in the OpenGL window
            //viewer.updateText(string(text_translation), string(text_rotation), tracking_state);
        } else sl::sleep_ms(1);
    }

	printf("ZED: thread closing\n");
	pthread_exit(NULL);

}


void transformPose(sl::Transform &pose, float tx) {
    sl::Transform transform_;
    transform_.setIdentity();
    // Move the tracking frame by tx along the X axis
    transform_.tx = tx;
    // Apply the transformation
    pose = Transform::inverse(transform_) * pose * transform_;
}

void close() {
    quit = true;
    zed_callback.join();
    zed.disableTracking("./ZED_spatial_memory"); // Record an area file

    zed.close();
    //viewer.exit();
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
		VICON.loop();

	VICON.close();
	printf("VICON: thread closing\n");
	pthread_exit(NULL);

}
