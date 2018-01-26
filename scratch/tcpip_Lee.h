//Update 8/9/10 5:55PM: allow to re-use a port number

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <strings.h>


//functions

//this function setsup a server specfied port, then listens for a connection and returns a new socket id for the established connection 
int init_server(int port)
{
	struct sockaddr_in serv_addr, cli_addr;
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
        return -1;
	
	// reuse a port number
	int on=1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
	{
		printf("setsockopt(SO_REUSEADDR) failed");
	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);
	if (bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
		return -1;
	
	listen(sockfd,5);
	
	int clilen = sizeof(cli_addr);
	int newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	
	if (newsockfd < 0) 
		return -1;
	
	return newsockfd;
	
}


//this function resloves and setsup connection with a server at the specfied address and port number.
//the output of the function is an integer that identifies the connection
int init_client(char *address, int port)
{
    struct sockaddr_in serv_addr;
    struct hostent *server;
	
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        return -1;
	
    server = gethostbyname(address);
    if (server == NULL) {
        return -1;
    }
	
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
	
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(port);
	
    if (connect(sockfd,&serv_addr,sizeof(serv_addr)) < 0) 
        return -1;
	
	
	return sockfd;
	
}
