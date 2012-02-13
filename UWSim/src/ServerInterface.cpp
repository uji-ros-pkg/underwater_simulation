/*
 * ServerInterface.cpp
 *
 *  Created on: 03/05/2010
 *      Author: mprats
 */

#include "ServerInterface.h"

#include <stdio.h>
#include <sys/types.h>
#ifdef UNIX
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#ifdef WIN32
#include <winsock.h>
#endif
#include <string.h>

#include <iostream>

void ServerInterface::run () {
	int client_sock, clilen;

#ifdef WIN32
	WSADATA wsaData;
	WORD wVersionRequested;
	int err;

	wVersionRequested = MAKEWORD( 2, 0 ); // 2.0 and above version of WinSock
	err = WSAStartup( wVersionRequested, &wsaData );
    if ( err != 0 ) {
	   std::cerr << "Couldn't not find a usable WinSock DLL" << std::endl;
	   exit(0);
	}
#endif

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	if (sockfd < 0) std::cerr << "ERROR opening socket" << std::endl;

	struct sockaddr_in serv_addr, cli_addr;
	memset((char *) &serv_addr, 0, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(port);

	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
			 std::cerr << "ERROR on binding" << std::endl;

	listen(sockfd,5);

	std::cerr << "Server ready, accepting clients..." << std::endl;
	clilen = sizeof(cli_addr);
	while (1) {
		std::cerr << "Waiting for client..." << std::endl;
#ifdef WIN32
		client_sock = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
#else
		client_sock = accept(sockfd, (struct sockaddr *) &cli_addr, (socklen_t*)&clilen);
#endif
		std::cerr << "Client accepted. Read and Callback to process data..." << std::endl;

		while(/*int readres=*/processData(client_sock));

		std::cerr << "Closing Client.." << std::endl;
#ifdef WIN32
		closesocket(client_sock);
		WSACleanup();
#else
		close(client_sock);
#endif
	}
}

ServerInterface::ServerInterface(int port): OpenThreads::Thread() {
	 this->port=port;
	 startThread();
	 std::cerr << "Thread created" << std::endl;
}
