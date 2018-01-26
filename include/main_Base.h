#include <time.h>
#include <stdbool.h>
#include "Eigen/Dense"
#include "fdcl_common_type.h"

typedef Eigen::Matrix<double, 4, 1> Vector4;

void task_health_check(struct time_stack* t, const char *task);

void *WIFI_Base_Thread(void *thread_id);
void gtk_gui();

extern "C" void on_btn_hello_clicked();
extern "C" void on_window_main_destroy();
extern "C" void on_toggle_file_save_toggled(GtkToggleButton *toggle_filesave, gpointer user_data);
extern "C" void on_toggle_system_toggled(GtkToggleButton *toggle_filesave, gpointer user_data);
extern "C" void on_toggle_calibrate_toggled(GtkToggleButton *toggle_filesave, gpointer user_data);
static gboolean update_gtk_labels(void* );
extern "C" void switch_motor_state_set_cb(GtkSwitch *switch_motor, gboolean state, gpointer user_data);
extern "C" void on_radiobutton_pos_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_radiobutton_att_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_0_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_1_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_2_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_3_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_4_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_5_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" void on_rd_cmd_6_toggled(GtkToggleButton *togglebutton, gpointer user_data);
extern "C" gboolean key_event(GtkWidget *widget, GdkEventKey *event, gpointer);

struct CONTROL_type {
	double dt;
	Vector3 eR, eW, M, eI;
	Vector4 f_motor;
};

struct MOTOR_type {
	int thr[4];
};


