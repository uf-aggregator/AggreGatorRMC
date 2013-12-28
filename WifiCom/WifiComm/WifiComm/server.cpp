#undef UNICODE

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#define DEFAULT_PORT "27015"
#define DEFAULT_BUFLEN 512

#pragma comment(lib, "Ws2_32.lib")

int main() {
	WSADATA wsaData;
	int iResult;

	SOCKET ListenSocket = INVALID_SOCKET;
	SOCKET ClientSocket = INVALID_SOCKET;//temp socket

	int iSendResult;
	int recvbuflen = DEFAULT_BUFLEN;
	char recvbuf[DEFAULT_BUFLEN];

	struct addrinfo *result = NULL, hints;

	// Initialize Winsock===============================================
	//makeword specifies highest version number, 2.2 for ws2_32
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}//endif

	//default port for socket operations

	ZeroMemory(&hints, sizeof (hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	//GET IPADDR INFO=================================================
	// Resolve the local address and port to be used by the server
	//will take first ip address
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return 1;
	}//endif

	//CREATE LISTENER SOCKET==========================================
	//create listener socket for clients
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("Error at socket(): %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 1;
	}//endif

	//BIND SOCKET TO EXISTING IP ADDRESS ON MACHINE
	//setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult==SOCKET_ERROR){
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);//the address info from getaddrinfo earlier
							//isn't needed so this call frees it up
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}//endif

	//BEGIN LISTENING ON THE SOCKET==================================
	if (listen(ListenSocket, SOMAXCONN) == SOCKET_ERROR) {
		printf("Listen failed with error: %ld\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);

	//ACCEPTING A CONNECTION=========================================
	//accept a client socket, single connection
	//to accept multiple connections, use a loop
	ClientSocket = accept(ListenSocket,NULL,NULL);
	if (ClientSocket==INVALID_SOCKET){
		printf("accept failed: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}//endif


	//RECEIVING AND SENDING DATA====================================
	
	//receive until peer closes connection
	do{
		iResult = recv(ClientSocket, recvbuf, recvbuflen,0);
		if (iResult > 0){
			printf("Bytes received: %d\n", iResult);

			// Echo the buffer back to the sender
			iSendResult = send(ClientSocket, recvbuf, iResult, 0);
			if (iSendResult == SOCKET_ERROR) {
				printf("send failed: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
				return 1;
			}
			printf("Bytes sent: %d\n", iSendResult);
		}//endif
		else if (iResult ==0){
			printf("Connection closing...\n");
		}//end elseif
		else{
			printf("recv failed: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			return 1;
		}//end else
	} while (iResult > 0);

	//SHUT DOWN CONNECTION AFTER SENDING DATA===================================
	iResult = shutdown(ClientSocket, SD_SEND);
	if (iResult == SOCKET_ERROR){
		printf("shutdown failed: %d\n", WSAGetLastError());
		closesocket(ClientSocket);
		WSACleanup();
		return 1;
	}//endif

	//cleanup
	closesocket(ClientSocket);
	WSACleanup();


	return 0;
}