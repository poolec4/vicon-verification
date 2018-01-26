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

#define MAXDATASIZE 100 // max number of bytes we can get at once 

int main()
{
	int sockfd, numbytes;  
	char buf[MAXDATASIZE];
		
	fhtcp=init_client("127.0.0.1",5000);
	
	if ((numbytes = recv(fhtcp, buf, MAXDATASIZE-1, 0)) == -1) {
		perror("recv");
		exit(1);
	}

	buf[numbytes] = '\0';

	printf("client: received '%s'\n",buf);


	double d;
	recv(fhtcp, &d, sizeof(d),0);
	printf("client: received %f\n",d);
	
	close(fhtcp);
		
}