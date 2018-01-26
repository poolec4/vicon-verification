#include <pthread.h>
#include <iostream>
#include <gtk/gtk.h>
#include <gtk/gtkmain.h>

#include "Eigen/Dense"
#include "main_Base.h"
#include "fdcl_serial.h"
#include "fdcl_wifi.h"
#include "fdcl_save.h"

//using Eigen::MatrixXd;
using namespace std;

bool SYSTEM_ON=true;
bool SAVE_TO_FILE=false;

bool UAV_SYSTEM_ON=true;
bool UAV_MOTOR_ON=false;
int  UAV_COMMAND_MODE=0;
bool UAV_CALIBRATE_ON=false;
bool UAV_CONTROL_ATT=true;

fdcl_save fd_save;
fdcl_wifi wifi;

struct g_update_type {
// gtk widgets that have to be updated by g_idle_add(update_gtk_labels,(gpointer) &gu);
	GtkWidget *label_t;
	GtkWidget *label_imu;
	GtkWidget *label_wifi;
	GtkWidget *label_vicon;
	GtkWidget *label_ctrl;
	GtkWidget *label_x[3];
	GtkWidget *label_v[3];
	GtkWidget *label_a[3];
	GtkWidget *label_ba[3];
	GtkWidget *label_bW[3];
	GtkWidget *label_W[3];
	GtkWidget *label_R[3][3];
	GtkWidget *label_Rd[3][3];
	GtkWidget *label_Wd[3];
	GtkWidget *label_eR[3];
	GtkWidget *label_eW[3];
	GtkWidget *label_eI[3];
	GtkWidget *label_M[3];
	GtkWidget *label_motor_f[4];
	GtkWidget *label_motor_thr[4];
	GtkWidget *toggle_calibrate;
};

#define NUM_THREADS 1

pthread_mutex_t uav_command_mutex;

struct UAV_state_type UAV;
struct COMMAND_type COMM;
struct CONTROL_type CTRL;
struct MOTOR_type motor;

int main()
{
    pthread_t threads[NUM_THREADS];
    pthread_attr_t attr;
    struct sched_param  param;
    int fifo_max_prio, fifo_min_prio;

    // Initialize mutex and condition variables
	pthread_mutex_init(&uav_command_mutex, NULL);

    // Set thread attributes
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);

	param.sched_priority = fifo_max_prio/NUM_THREADS;
	pthread_attr_setschedparam(&attr, &param);

	printf("Beginning threads...\n");
	pthread_create(&threads[0], &attr, WIFI_Base_Thread, (void *) 0);

	gtk_gui();

//	pthread_join(threads[0], NULL);

	pthread_attr_destroy(&attr);
	pthread_mutex_destroy(&uav_command_mutex);

	printf("Threads closed.\n");

	return 0;
}


void *WIFI_Base_Thread(void *thread_id)
{
	fdcl_serial buf_recv, buf_send;
	fdcl_param file_cfg;

	int bytes_sent, bytes_recv;

	file_cfg.open("../jetson_SEH.cfg");
	file_cfg.read("WIFI.port",wifi.port);
	wifi.open_server();

	while(SYSTEM_ON == true)
	{
		wifi.t_pre=wifi.t;
		wifi.t=wifi.gettime();
		wifi.dt=wifi.t-wifi.t_pre;

		bytes_recv=wifi.recv(buf_recv,749);

		pthread_mutex_lock(&uav_command_mutex);
		buf_recv.unpack(UAV_SYSTEM_ON);
		buf_recv.unpack(UAV_MOTOR_ON);
		buf_recv.unpack(UAV_COMMAND_MODE);
		buf_recv.unpack(UAV_CALIBRATE_ON);
		pthread_mutex_unlock(&uav_command_mutex);

		buf_recv.unpack(UAV.t);
		buf_recv.unpack(UAV.t_IMU);
		buf_recv.unpack(UAV.t_VICON);

		buf_recv.unpack(UAV.dt);
		buf_recv.unpack(UAV.dt_IMU);
		buf_recv.unpack(UAV.dt_VICON);
		buf_recv.unpack(CTRL.dt);

		buf_recv.unpack(UAV.x);
		buf_recv.unpack(UAV.v);
		buf_recv.unpack(UAV.a);
		buf_recv.unpack(UAV.W);
		buf_recv.unpack(UAV.R);
		buf_recv.unpack(UAV.b_a);
		buf_recv.unpack(UAV.b_W);
//		buf_recv.unpack(UAV.P);

//		cout << UAV.t << endl << UAV.b_W << endl << endl;;

		buf_recv.unpack(UAV.W_b_IMU);
		buf_recv.unpack(UAV.a_f_IMU);
		buf_recv.unpack(UAV.R_fb_IMU);
//		cout << UAV.t << endl << IMU.R_fb << endl << endl;;

		buf_recv.unpack(UAV.x_f_VICON);
		buf_recv.unpack(UAV.R_fb_VICON);

		buf_recv.unpack(COMM.Rd);
		buf_recv.unpack(COMM.Wd);
		buf_recv.unpack(COMM.Wd_dot);

		buf_recv.unpack(CTRL.eR);
		buf_recv.unpack(CTRL.eW);
		buf_recv.unpack(CTRL.eI);
		buf_recv.unpack(CTRL.M);
		buf_recv.unpack(CTRL.f_motor);
		buf_recv.unpack(motor.thr[0]);
		buf_recv.unpack(motor.thr[1]);
		buf_recv.unpack(motor.thr[2]);
		buf_recv.unpack(motor.thr[3]);

//		cout << UAV.t << endl << CTRL.M << endl << endl;;
		buf_recv.clear();

		usleep(10000);

		buf_send.clear();
		buf_send.pack(UAV_SYSTEM_ON);
		buf_send.pack(UAV_MOTOR_ON);
		buf_send.pack(UAV_COMMAND_MODE);
		buf_send.pack(UAV_CALIBRATE_ON);

		bytes_sent=wifi.send(buf_send);
		//cout << bytes_sent << "/" << buf_send.size() << " sent" << endl;

		//printf("%i: %i, %i, %i, %i\n",bytes_sent,UAV_SYSTEM_ON,UAV_MOTOR_ON,UAV_COMMAND_MODE,UAV_CALIBRATE_ON);


/*		if(UAV_CALIBRATE_ON==false)
		{
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(g_toggle_calibrate),FALSE);
		}*/
		if(SAVE_TO_FILE==true)
		{

			fd_save.write(UAV.t);
			fd_save.write(UAV.t_IMU);
			fd_save.write(UAV.t_VICON);

			fd_save.write(UAV.x);
			fd_save.write(UAV.v);
			fd_save.write(UAV.a);
			fd_save.write(UAV.W);
			fd_save.write(UAV.R);
			fd_save.write(UAV.b_a);
			fd_save.write(UAV.b_W);

			fd_save.write(UAV.W_b_IMU);
			fd_save.write(UAV.a_f_IMU);
			fd_save.write(UAV.R_fb_IMU);

			fd_save.write(UAV.x_f_VICON);
			fd_save.write(UAV.R_fb_VICON);

			fd_save.write(COMM.Rd);
			fd_save.write(COMM.Wd);
			fd_save.write(COMM.Wd_dot);

			fd_save.write(CTRL.eR);
			fd_save.write(CTRL.eW);
			fd_save.write(CTRL.eI);
			fd_save.write(CTRL.M);
			fd_save.write(CTRL.f_motor);

			fd_save.endl();
		}
	}

	printf("WIFI: closing...\n");

	pthread_exit(NULL);

}


void gtk_gui()
{
	GtkWidget *g_switch_motor;
	GtkWidget *g_toggle_motor;
	GtkWidget *g_toggle_filesave;
	GtkWidget *g_entry_filename;
	GtkWidget *g_toggle_system;
	g_update_type gu;

	printf("GTK: creating GUI..\n");

	GtkBuilder      *builder;
    GtkWidget       *window;

    gtk_init(NULL,NULL);

    builder = gtk_builder_new();
    gtk_builder_add_from_file (builder, "../window_main.glade", NULL);

    window = GTK_WIDGET(gtk_builder_get_object(builder, "window_main"));
    gtk_builder_connect_signals(builder, NULL);

    // get pointers to the objects
	gu.label_t = GTK_WIDGET(gtk_builder_get_object(builder, "label_t"));
    gu.label_imu = GTK_WIDGET(gtk_builder_get_object(builder, "label_imu"));
    gu.label_wifi = GTK_WIDGET(gtk_builder_get_object(builder, "label_wifi"));
    gu.label_vicon = GTK_WIDGET(gtk_builder_get_object(builder, "label_vicon"));
    gu.label_ctrl = GTK_WIDGET(gtk_builder_get_object(builder, "label_ctrl"));

    g_switch_motor = GTK_WIDGET(gtk_builder_get_object(builder, "switch_motor"));
    g_toggle_motor = GTK_WIDGET(gtk_builder_get_object(builder, "toggle_motor"));
    g_toggle_filesave = GTK_WIDGET(gtk_builder_get_object(builder, "toggle_filesave"));
    g_entry_filename = GTK_WIDGET(gtk_builder_get_object(builder, "entry_filename"));
    g_toggle_system = GTK_WIDGET(gtk_builder_get_object(builder, "toggle_system"));
    gu.toggle_calibrate = GTK_WIDGET(gtk_builder_get_object(builder, "toggle_calibrate"));

    gu.label_x[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_x0"));
    gu.label_x[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_x1"));
    gu.label_x[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_x2"));

    gu.label_v[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_v0"));
    gu.label_v[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_v1"));
    gu.label_v[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_v2"));

    gu.label_W[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_W0"));
    gu.label_W[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_W1"));
    gu.label_W[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_W2"));

    gu.label_a[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_a0"));
    gu.label_a[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_a1"));
    gu.label_a[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_a2"));

    gu.label_ba[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_ba0"));
    gu.label_ba[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_ba1"));
    gu.label_ba[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_ba2"));

    gu.label_bW[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_bW0"));
    gu.label_bW[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_bW1"));
    gu.label_bW[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_bW2"));

    gu.label_R[0][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R00"));
    gu.label_R[1][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R10"));
    gu.label_R[2][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R20"));
    gu.label_R[0][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R01"));
    gu.label_R[1][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R11"));
    gu.label_R[2][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R21"));
    gu.label_R[0][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R02"));
    gu.label_R[1][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R12"));
    gu.label_R[2][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_R22"));

    gu.label_Wd[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Wd0"));
    gu.label_Wd[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Wd1"));
    gu.label_Wd[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Wd2"));

    gu.label_Rd[0][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd00"));
    gu.label_Rd[1][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd10"));
    gu.label_Rd[2][0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd20"));
    gu.label_Rd[0][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd01"));
    gu.label_Rd[1][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd11"));
    gu.label_Rd[2][1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd21"));
    gu.label_Rd[0][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd02"));
    gu.label_Rd[1][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd12"));
    gu.label_Rd[2][2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_Rd22"));

    gu.label_eR[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eR0"));
    gu.label_eR[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eR1"));
    gu.label_eR[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eR2"));

    gu.label_eW[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eW0"));
    gu.label_eW[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eW1"));
    gu.label_eW[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eW2"));

    gu.label_eI[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eI0"));
    gu.label_eI[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eI1"));
    gu.label_eI[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_eI2"));

    gu.label_M[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_M0"));
    gu.label_M[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_M1"));
    gu.label_M[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_M2"));

    gu.label_motor_f[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_f0"));
    gu.label_motor_f[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_f1"));
    gu.label_motor_f[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_f2"));
    gu.label_motor_f[3] = GTK_WIDGET(gtk_builder_get_object(builder, "label_f3"));

    gu.label_motor_thr[0] = GTK_WIDGET(gtk_builder_get_object(builder, "label_thr0"));
    gu.label_motor_thr[1] = GTK_WIDGET(gtk_builder_get_object(builder, "label_thr1"));
    gu.label_motor_thr[2] = GTK_WIDGET(gtk_builder_get_object(builder, "label_thr2"));
    gu.label_motor_thr[3] = GTK_WIDGET(gtk_builder_get_object(builder, "label_thr3"));

	// set the default file name
	gtk_entry_set_text(GTK_ENTRY(g_entry_filename),"test0");
	// set the system_on togle on
	gtk_switch_set_active(GTK_SWITCH(g_switch_motor),FALSE);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(g_toggle_system),TRUE);
	gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gu.toggle_calibrate),FALSE);
	// set callback for key press
	g_signal_connect(G_OBJECT(window), "key-release-event", G_CALLBACK(key_event), (gpointer) g_switch_motor);

    g_object_unref(builder);

    gtk_widget_show(window);

	// update labels whenever available
	g_idle_add(update_gtk_labels,(gpointer) &gu);

    gtk_main();

}

gboolean key_event(GtkWidget *widget, GdkEventKey *event, gpointer user_data)
{
	gchar* key_pressed;

	key_pressed = gdk_keyval_name(event->keyval);
	cout << "KEY: pressed: " << key_pressed << endl;

	switch(event->keyval)
	{
		case GDK_KEY_m:
			pthread_mutex_lock(&uav_command_mutex);
			UAV_MOTOR_ON=false;
			pthread_mutex_unlock(&uav_command_mutex);

			printf("UAV_MOTOR_ON = false\n");
			gtk_switch_set_active(GTK_SWITCH(user_data),FALSE);

		break;

		case GDK_KEY_s:


		break;
	}

	return FALSE;
}


void switch_motor_state_set_cb(GtkSwitch *switch_motor, gboolean state, gpointer user_data)
{
	if (gtk_switch_get_active(switch_motor))
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_MOTOR_ON=true;
		pthread_mutex_unlock(&uav_command_mutex);
	}
	else
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_MOTOR_ON=false;
		pthread_mutex_unlock(&uav_command_mutex);
	}
}

void on_toggle_system_toggled(GtkToggleButton *toggle_system, gpointer user_data)
{
	if (gtk_toggle_button_get_active(toggle_system))
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_SYSTEM_ON=true;
		SYSTEM_ON=true;
		pthread_mutex_unlock(&uav_command_mutex);
	}
	else
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_SYSTEM_ON=false;
		SYSTEM_ON=false;
		gtk_main_quit();

		pthread_mutex_unlock(&uav_command_mutex);
	}
}


void on_radiobutton_pos_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
	{
		UAV_CONTROL_ATT=false;
		if (UAV_COMMAND_MODE < 0)
			UAV_COMMAND_MODE *= -1;

		cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;


	}
}


void on_radiobutton_att_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
	{
		UAV_CONTROL_ATT=true;
		if (UAV_COMMAND_MODE > 0)
			UAV_COMMAND_MODE *= -1;

		cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

	}
}

void on_rd_cmd_0_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
	{
		UAV_COMMAND_MODE=0;
		cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;
	}

}


void on_rd_cmd_1_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=1;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}

void on_rd_cmd_2_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=2;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}

void on_rd_cmd_3_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=3;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}

void on_rd_cmd_4_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=4;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}

void on_rd_cmd_5_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=5;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}

void on_rd_cmd_6_toggled(GtkToggleButton *togglebutton, gpointer user_data)
{
	if (gtk_toggle_button_get_active(togglebutton))
		UAV_COMMAND_MODE=6;

	if (UAV_CONTROL_ATT==true)
		UAV_COMMAND_MODE*=-1;

	cout << "UAV_COMMAND_MODE: " << UAV_COMMAND_MODE << endl;

}


void on_toggle_file_save_toggled(GtkToggleButton *toggle_filesave, gpointer user_data)
{
	const gchar *entry_text;
	char file_name[256];

	if (gtk_toggle_button_get_active(toggle_filesave))
	{
//		entry_text=gtk_entry_get_text(GTK_ENTRY(g_entry_filename));
		entry_text=gtk_entry_get_text(GTK_ENTRY(user_data));
		strncpy(file_name,entry_text,strlen(entry_text));
		sprintf(file_name+strlen(file_name),".txt");
		printf("FILE: started saving at buffer for the file, : %s\n",file_name);

		SAVE_TO_FILE=true;
		fd_save.open(file_name);

	}
	else
	{
		SAVE_TO_FILE=false;
		fd_save.close();
		printf("FILE: saved and closed\n");

	}


}

void on_toggle_calibrate_toggled(GtkToggleButton *toggle_system, gpointer user_data)
{
	if (gtk_toggle_button_get_active(toggle_system))
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_CALIBRATE_ON=true;
		pthread_mutex_unlock(&uav_command_mutex);
	}
	else
	{
		pthread_mutex_lock(&uav_command_mutex);
		UAV_CALIBRATE_ON=false;
		pthread_mutex_unlock(&uav_command_mutex);
	}
}


// called when window is closed
void on_window_main_destroy()
{
    gtk_main_quit();
    SYSTEM_ON = false;

}



static gboolean update_gtk_labels(void* user_data)
{
	char label_text[50] = {0};
	int i,j;

	g_update_type* gu = (g_update_type*) user_data;

	sprintf(label_text, "t: %5.2f sec",UAV.t);
    gtk_label_set_text(GTK_LABEL(gu->label_t), label_text);

	sprintf(label_text, "IMU: %5.2f kHz",1/UAV.dt_IMU/1000.);
    gtk_label_set_text(GTK_LABEL(gu->label_imu), label_text);

	sprintf(label_text, "WIFI: %5.2f kHz",1/wifi.dt/1000.);
    gtk_label_set_text(GTK_LABEL(gu->label_wifi), label_text);

	sprintf(label_text, "VICON: %5.2f kHz",1/UAV.dt_VICON/1000.);
    gtk_label_set_text(GTK_LABEL(gu->label_vicon), label_text);

	sprintf(label_text, "Control: %5.2f kHz",1/CTRL.dt/1000.);
    gtk_label_set_text(GTK_LABEL(gu->label_ctrl), label_text);


	for (i=0; i< 3; i++)
	{
		sprintf(label_text, "%6.4f",UAV.x(i));
    	gtk_label_set_text(GTK_LABEL(gu->label_x[i]), label_text);
		sprintf(label_text, "%6.4f",UAV.v(i));
    	gtk_label_set_text(GTK_LABEL(gu->label_v[i]), label_text);
		sprintf(label_text, "%6.4f",UAV.a(i));
    	gtk_label_set_text(GTK_LABEL(gu->label_a[i]), label_text);
		sprintf(label_text, "%6.4f",UAV.b_a(i));
   		gtk_label_set_text(GTK_LABEL(gu->label_ba[i]), label_text);

		sprintf(label_text, "%6.4f",UAV.W(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_W[i]), label_text);
		sprintf(label_text, "%6.4f",UAV.b_W(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_bW[i]), label_text);

		sprintf(label_text, "%6.4f",COMM.Wd(i));
   		gtk_label_set_text(GTK_LABEL(gu->label_Wd[i]), label_text);

		sprintf(label_text, "%6.4f",CTRL.eR(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_eR[i]), label_text);
		sprintf(label_text, "%6.4f",CTRL.eW(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_eW[i]), label_text);
		sprintf(label_text, "%6.4f",CTRL.eI(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_eI[i]), label_text);
		sprintf(label_text, "%6.4f",CTRL.M(i));
	    gtk_label_set_text(GTK_LABEL(gu->label_M[i]), label_text);

		for (j=0; j<3; j++)
		{
			sprintf(label_text, "%6.4f",UAV.R(i,j));
		    gtk_label_set_text(GTK_LABEL(gu->label_R[i][j]), label_text);

			sprintf(label_text, "%6.4f",COMM.Rd(i,j));
		    gtk_label_set_text(GTK_LABEL(gu->label_Rd[i][j]), label_text);
		}

	}

	for (i=0; i< 4; i++)
	{
		sprintf(label_text, "%6.4f",CTRL.f_motor(i));
    	gtk_label_set_text(GTK_LABEL(gu->label_motor_f[i]), label_text);
		sprintf(label_text, "%d",motor.thr[i]);
    	gtk_label_set_text(GTK_LABEL(gu->label_motor_thr[i]), label_text);
	}

	if(UAV_CALIBRATE_ON==false)
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(gu->toggle_calibrate),FALSE);

	return true;

}
