using namespace std;

#include <iostream>
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <string.h>
#include <sys/time.h>
#include <string.h>
#include <netinet/in.h>
#include <vrpn_Tracker.h>
vrpn_TRACKERCB VData;


struct timeval tinit, tnow, tv;

double R_ev[3][3]={{1.0  ,  0.0  ,  0.0},
                   {0.0  , -1.0  ,  0.0},
                   {0.0  ,  0.0  , -1.0}};// Vicon frame (v) to inertial frame (e) (fixed)
double R_bm[3][3]={{1.0  ,  0.0  ,  0.0},
                   {0.0  , -1.0  ,  0.0},
                   {0.0  ,  0.0  , -1.0}};// Markers frame (m) to the body frame (b) (fixed)


static void VRPN_CALLBACK pose_callback(void* user_data, const vrpn_TRACKERCB tData)
{

    VData.pos[0]=tData.pos[0];
    VData.pos[1]=tData.pos[1];
    VData.pos[2]=tData.pos[2];
    VData.quat[0]=tData.quat[0];
    VData.quat[1]=tData.quat[1];
    VData.quat[2]=tData.quat[2];
    VData.quat[3]=tData.quat[3];
}
 

int main(int argc, char *argv[])
{

    struct timeval t_init, t_now;
    double t;

    vrpn_Tracker_Remote* tracker_pos = new vrpn_Tracker_Remote("base_link@192.168.10.1");

    tracker_pos->register_change_handler(0, pose_callback);

    bool shouldRun = true;

    int wr;
    int nsbuf;
    float data_send[7];

    tracker_pos->mainloop();
    sleep(1);

    double data_send_ = 0.0;// The v1 position from the last loop
    gettimeofday(&t_init,NULL);

    while(shouldRun)
    {
    
        double x_v[3], x_e[3], quat_vm[4], R_vm[3][3], R_em[3][3], R_eb[3][3];

		tracker_pos->mainloop();
        // Position in Vicon frame
        x_v[0] = VData.pos[0];
        x_v[1] = VData.pos[1];
        x_v[2] = VData.pos[2];

        // Quaternions from markers frame to Vicon frame
        quat_vm[0] = (double)VData.quat[0];
        quat_vm[1] = (double)VData.quat[1];
        quat_vm[2] = (double)VData.quat[2];
        quat_vm[3] = (double)VData.quat[3];

        // Rotation matrix from markers frame to Vicon frame
        R_vm[0][0] = 1-(2*(quat_vm[1])*(quat_vm[1]))-(2*(quat_vm[2])*(quat_vm[2]));
        R_vm[0][1] = (2*quat_vm[0]*quat_vm[1])-(2*quat_vm[3]*quat_vm[2]);
        R_vm[0][2] = (2*quat_vm[0]*quat_vm[2])+(2*quat_vm[3]*quat_vm[1]);
        R_vm[1][0] = (2*quat_vm[0]*quat_vm[1])+(2*quat_vm[3]*quat_vm[2]);
        R_vm[1][1] = 1-(2*(quat_vm[0])*(quat_vm[0]))-(2*(quat_vm[2])*(quat_vm[2]));
        R_vm[1][2] = (2*(quat_vm[1])*(quat_vm[2]))-(2*(quat_vm[3])*(quat_vm[0]));
        R_vm[2][0] = (2*quat_vm[0]*quat_vm[2])-(2*quat_vm[3]*quat_vm[1]);
        R_vm[2][1] = (2*quat_vm[0]*quat_vm[3])+(2*quat_vm[2]*quat_vm[1]);
        R_vm[2][2] = 1-(2*(quat_vm[0])*(quat_vm[0]))-(2*(quat_vm[1])*(quat_vm[1]));

        // R_eb = R_ev*R_vm*R_bm' (note that R_bm' = R_bm)
//        Matrix_multipication(R_ev, R_vm, R_em);
//        Matrix_multipication(R_em, R_bm, R_eb);// R_eb: body frame to inertial frame

        // Location of mass center (inertial frame)
 
        printf("Position in the inertial frame:\n %E     %E     %E \n\n\n",x_v[0], x_v[1], x_v[2]);
/*        printf("Rotation matrix from body-fixed frame to inertial frame:\n\n %E     %E     %E\n %E     %E     %E\n %E     %E     %E\n",
               R_eb[0][0], R_eb[0][1], R_eb[0][2],
               R_eb[1][0], R_eb[1][1], R_eb[1][2],
               R_eb[2][0], R_eb[2][1], R_eb[2][2]);
        printf("\n\n\n\n\n");
*/
		usleep(10000);
    }

    delete tracker_pos;

    return 0;//a.exec();
}
