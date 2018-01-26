cmake_minimum_required (VERSION 3.1)
project(FDCL-UAV)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package (PkgConfig REQUIRED)
pkg_check_modules (GTK3 REQUIRED gtk+-3.0)

include_directories (${GTK3_INCLUDE_DIRS})
link_directories (${GTK3_LIBRARY_DIRS})
add_definitions (${GTK3_CFLAGS_OTHER})

set (CMAKE_CXX_STANDARD 11)

pkg_check_modules(LIBSERIALPORT libserialport)
link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

pkg_check_modules(LIBSERIALPORT libserialport)
link_directories(${LIBSERIALPORT_LIBRARY_DIRS})
include_directories(${LIBSERIALPORT_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries/vectornav/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries/eigen)
include_directories(${GTK3_INCLUDE_DIRS})

# set(CMAKE_CXX_FLAGS "-Wall -c -O3 -lvrpn -lquat -pthread -std=c++11 ${CMAKE_C_FLAGS}")


################################################################################
#### Make UAV
################################################################################
set(jetson_src
    src/main_Jetson.cpp
)
add_executable(jetson ${jetson_src})
target_link_libraries(jetson
    uav
    host
    vn100
    vrpn
    Threads::Threads
)
target_compile_options(jetson PRIVATE -pthread -Wall -o -lrt -lm -lvrpn -lquat -pthread -lserialport -lsbp -O3 -std=c++11)


################################################################################
#### Make Host
################################################################################
set(base_src
    src/main_Base.cpp
)
add_executable(base ${base_src})
target_link_libraries(base
    host
    vn100
    vrpn
    Threads::Threads
    ${GTK3_LIBRARIES}
)
target_compile_options(base PRIVATE -pthread -o -lrt -lm -lvrpn -lquat ${GTK3_CFLAGS_OTHER} -O3 -std=c++11)


################################################################################
#### Compile host libraries
################################################################################

set(host_src
    ${PROJECT_SOURCE_DIR}/src/fdcl_serial.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_wifi.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_param.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_save.cpp
)
add_library(host STATIC ${host_src})


################################################################################
#### Compile UAV libraries
################################################################################

set(uav_src
    ${PROJECT_SOURCE_DIR}/src/fdcl_vicon.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_vn100.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_i2c.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_EKF.cpp
    ${PROJECT_SOURCE_DIR}/src/misc_matrix_func.cpp
    ${PROJECT_SOURCE_DIR}/src/fdcl_control.cpp
)
add_library(uav STATIC ${uav_src})
target_link_libraries(uav
    PRIVATE vn100 sbp serialport
)


################################################################################
#### Compile VectorNav libraries
################################################################################

set(vn100_src
    ${PROJECT_SOURCE_DIR}/libraries/vectornav/src/vn100.c
    ${PROJECT_SOURCE_DIR}/libraries/vectornav/src/vndevice.c
    ${PROJECT_SOURCE_DIR}/libraries/vectornav/src/arch/linux/vncp_services.c
)
add_library(vn100 STATIC ${vn100_src})
