#include "fdcl_wifi.h"

fdcl_wifi::fdcl_wifi()
{
	clock_gettime(CLOCK_REALTIME, &tspec_INIT);
}
void fdcl_wifi::load_config(fdcl_param& cfg)
{
	cfg.read("WIFI.server_ip_addr",server_ip_addr);
	cfg.read("WIFI.port",port);	
}

int fdcl_wifi::open_server()
{
	return open_server(port);
}

int fdcl_wifi::open_server(int port)
{
	this->port=port;
	struct sockaddr_in serv_addr, cli_addr;
	printf("WIFI: opening socket...\n");
	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
	{
     	printf("WIFI: ERROR: opening socket...\n");
	    return -1;
	}
	// reuse a port number
	int on=1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)
	{
		printf("setsockopt(SO_REUSEADDR) failed\n");
	}
	
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);
	if ( ::bind(sockfd, (struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
	{
		printf("WIFI: binding failed\n");
		return -1;
	}
	listen(sockfd,5);
	
	unsigned int clilen = sizeof(cli_addr);
	printf("WIFI: accepting...\n");
	int newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	
	if (newsockfd < 0) 
	{
		printf("WIFI: accepting failed\n");
		return -1;
	}
	
	this->sockfd=newsockfd;
	return newsockfd;
	
}

int fdcl_wifi::open_client()
{
	return open_client(server_ip_addr,port);
}

int fdcl_wifi::open_client(string server_ip_addr, int port)
{
	this->server_ip_addr=server_ip_addr;
	this->port=port;
	
    struct sockaddr_in serv_addr;
    struct hostent *server;

	printf("WIFI: opening socket...\n");	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
	{
		printf("WIFI: ERROR initializing socket...\n");	
        return -1;
	}
    server = gethostbyname(server_ip_addr.c_str());
    if (server == NULL) {
		printf("WIFI: ERROR getting host by name...\n");	
        return -1;
    }
	
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
	
    bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
    serv_addr.sin_port = htons(port);
	
	printf("WIFI: client connecting...\n");	
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
	{
		printf("WIFI: ERROR client connecting...\n");	
	
        return -1;
	}
	
	return sockfd;		
}

int fdcl_wifi::send(const void *buf, size_t len)
{
	int bytes_sent;
	bytes_sent=::send(sockfd,buf,len,MSG_NOSIGNAL);	
	if (bytes_sent < len)
		cout << "ERROR: fdcl_wifi.send: bytes_sent " << bytes_sent << " <  bytes_requested  " << len << endl;
	return bytes_sent;
}

int fdcl_wifi::send(fdcl_serial& buf)
{
	return send(buf.data(),buf.size());	
}

int fdcl_wifi::recv(void *buf, size_t len)
{
	int bytes_recv;
	bytes_recv=::recv(sockfd,buf,len,0);
	if (bytes_recv < len)
		cout << "ERROR: fdcl_wifi.recv: bytes_recv " << bytes_recv << " <  bytes_requested  " << len << endl;
	return bytes_recv;
}

int fdcl_wifi::recv(size_t len)
{
	if (len > MAX_BUFFER_RECV_SIZE)
		cout << "ERROR: fdcl_wifi.recv: the requested receceive data size is greater than MAX_BUFFER_SIZE_RECV, increase MAX_BUFFER_SIZE_RECV in fdcl_wifi.h" << endl;
	return recv(buf_recv,len);
}

int fdcl_wifi::recv(fdcl_serial& fdcl_serial_recv_buf, size_t len)
{
	int bytes_recv;
	bytes_recv=recv(len);
	fdcl_serial_recv_buf.clear();
	fdcl_serial_recv_buf.init(buf_recv,len);		
	
	return bytes_recv;
}


double fdcl_wifi::gettime()
{
	double t;
	clock_gettime(CLOCK_REALTIME, &tspec_curr);
	t=(double) tspec_curr.tv_sec+ ((double)tspec_curr.tv_nsec)/1.e9;	
	t-=(double) tspec_INIT.tv_sec+ ((double)tspec_INIT.tv_nsec)/1.e9;
	return t;
}



