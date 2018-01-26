#ifndef _FDCL_WIFI_H
#define _FDCL_WIFI_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <strings.h>

#include "fdcl_param.h"
#include "fdcl_serial.h"

// do not request if MSG_NOSIGNAL is not defined 
// it is usually defined for linux, but not on mac
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0x0
#endif

#define MAX_BUFFER_RECV_SIZE 8192

class fdcl_wifi{
public:
	fdcl_wifi();
	~fdcl_wifi() {};
	
	int sockfd;
	int port;
	string server_ip_addr;
	unsigned char buf_recv[MAX_BUFFER_RECV_SIZE];
	double t, t_pre, dt;
	
	void load_config(fdcl_param& );
	int open_server(int port);
	int open_server( );
	int open_client();
	int open_client(string server_ip_addr, int port);
	int send(const void *buf, size_t len);	
	int send(fdcl_serial& buf_send);
	int recv(void *buf, size_t len);
	int recv(size_t len);
	int recv(fdcl_serial& buf_recv, size_t len);
	double gettime();	
private:
	struct timespec tspec_INIT, tspec_curr;
	
};

#endif
