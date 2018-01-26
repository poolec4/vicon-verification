#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>

#include "tcpip_Lee.h"
#define TCP_PORT 5000
int fhtcp;
char TCP_str[256];
int size_TCP_str;

int main()
{
	fhtcp=init_server(5000);
	
	char *msg = "Beej was here!";
	int len, bytes_sent;
	len = strlen(msg);
	bytes_sent = send(fhtcp, msg, len, 0);
	
	double d= 3.134;
	send(fhtcp, &d, sizeof(d),0);
	
	close(fhtcp);
	
	
}