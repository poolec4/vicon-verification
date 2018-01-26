INCLUDE_PATH=./vectornav/include ./eigen
CFLAGS=$(foreach d, $(INCLUDE_PATH), -I$d) -Wall -std=c++11
VRPN_LIBS = -lvrpn -lquat -pthread
OPTI_FLAG = -O3
GTK_LIBS = -rdynamic `pkg-config --cflags gtk+-3.0`  `pkg-config --libs gtk+-3.0`

jetson: fdcl_vicon.o fdcl_vn100.o fdcl_EKF.o fdcl_wifi.o fdcl_i2c.o fdcl_control.o fdcl_param.o misc_matrix_func.o main_Jetson.o vn100.o vncp_services.o vndevice.o fdcl_serial.o
	g++ -pthread  -o jetson fdcl_vicon.o fdcl_vn100.o fdcl_EKF.o fdcl_wifi.o fdcl_serial.o fdcl_i2c.o fdcl_control.o fdcl_param.o misc_matrix_func.o main_Jetson.o vn100.o vncp_services.o vndevice.o -lrt -lm $(VRPN_LIBS) $(OPTI_FLAG)

base: main_Base.o fdcl_save.o fdcl_serial.o fdcl_wifi.o fdcl_param.o
	g++ -lm -o base main_Base.o fdcl_serial.o fdcl_wifi.o fdcl_param.o fdcl_save.o $(GTK_LIBS) $(CFLAGS) $(OPTI_FLAG)

main_Base.o: main_Base.cpp main_Base.h fdcl_common_type.h
	g++ -pthread -lm -c main_Base.cpp $(GTK_LIBS) $(CFLAGS) $(OPTI_FLAG)

fdcl_control.o: fdcl_control.cpp fdcl_control.h
	g++ -pthread -c fdcl_control.cpp $(CFLAGS) $(OPTI_FLAG)

main_Jetson.o: fdcl_vn100.h  main_Jetson.cpp main_Jetson.h fdcl_common_type.h
	g++ -pthread -c main_Jetson.cpp $(CFLAGS) $(OPTI_FLAG)

misc_matrix_func.o: misc_matrix_func.c misc_matrix_func.h
	g++ -c -O3 -Wall misc_matrix_func.c $(CFLAGS)

fdcl_save.o: fdcl_save.h fdcl_save.cpp
	g++ -c -O3 -Wall fdcl_save.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_param.o: fdcl_param.h fdcl_param.cpp
	g++ -c -O3 -Wall fdcl_param.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_i2c.o: fdcl_i2c.h fdcl_i2c.cpp
	g++ -c -O3 -Wall fdcl_i2c.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_wifi.o: fdcl_wifi.h fdcl_wifi.cpp
	g++ -c -O3 -Wall fdcl_wifi.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_serial.o: fdcl_serial.h fdcl_serial.cpp
	g++ -c -O3 -Wall fdcl_serial.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_EKF.o: fdcl_EKF.h fdcl_EKF.cpp misc_matrix_func.o
	g++ -c -O3 -Wall fdcl_EKF.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_vn100.o: fdcl_vn100.h fdcl_vn100.cpp vn100.o vndevice.o vncp_services.o
	g++ -c -O3 -Wall fdcl_vn100.cpp $(CFLAGS) $(OPTI_FLAG)

fdcl_vicon.o: fdcl_vicon.h fdcl_vicon.cpp
	g++ -c -O3 -Wall fdcl_vicon.cpp $(CFLAGS) $(OPTI_FLAG) $(VRPN_LIBS)

vn100.o: ./vectornav/src/vn100.c
	gcc -c  ./vectornav/src/vn100.c $(CFLAGS) $(OPTI_FLAG)

vndevice.o: ./vectornav/src/vndevice.c
	gcc -c  ./vectornav/src/vndevice.c $(CFLAGS) $(OPTI_FLAG)

vncp_services.o: ./vectornav/src/arch/linux/vncp_services.c
	gcc -c  ./vectornav/src/arch/linux/vncp_services.c $(CFLAGS) $(OPTI_FLAG)

clean:
	rm -f *.o
