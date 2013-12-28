#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

//DEFINE DEFAULT VALUES HERE
#define DEFAULT_PORT "27015" //CHANGE PORT USED FOR ENTIRE CLIENT
#define DEFAULT_BUFLEN 512 //MESSAGE LENGTH

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")

int main(int argc, char **argv) {
	//DATA MEMBERS=============================================
	WSADATA wsaData;
	int iResult;

	//make addrinfo object that contains a sockaddr structure
	//and initializes these values
	struct addrinfo *result = NULL, *ptr = NULL, hints;

	//CREATE REQUEST SOCKET
	//address family is unspecified as to not be ipv6 or ipv4 sensitive
	SOCKET ConnectSocket = INVALID_SOCKET;
	int recvbuflen = DEFAULT_BUFLEN;
	char *sendbuf = "Testing";
	char recvbuf[DEFAULT_BUFLEN];

	//validate parameters
	//NOTICE takes command line argument
	//as shown, it'll take the name for the server
	if (argc != 2) {
		printf("usage: %s server-name\n", argv[0]);
		return 1;
	}//endif

	// Initialize Winsock
	//makeword specifies highest version number, 2.2 for ws2_32
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		//check wsa went through normally
		printf("WSAStartup failed: %d\n", iResult);
		return 1;
	}
	//request socket type to be stream type TCP
	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;


	//REQUEST IP ADDRESS===========================================
	//note it takes in a command line argument
	//this argument is the ip address of the server
	iResult - getaddrinfo(argv[1], DEFAULT_PORT, &hints, &result);
	if (iResult!=0){
		printf("getaddrinfo failed: %d\n", iResult);
		WSACleanup();
		return 1;
	}//endif

	//CONNECT TO ADDRESS RETURNED EARLIER=========================
	// Attempt to connect to an address until one succeeds
	//this for loop is read as *ptr is the result from earlier
	//the loop condition is until ptr is not NULL
	//and if that result doesn't work it goes onto the next one
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		//this is just preparing a socket to be passed as an argument
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);

		//just an error check
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		//CONNECT TO THE SERVER
		//run connect command to get server socket info
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR){
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}//endif
		break;
	}//end for

	//SEND AND RECEIVE DATA FROM SERVER==============================
	//send an initial buffer
	iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf),0);
	if (iResult == SOCKET_ERROR) {
		printf("send failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		return 1;
	}//endif

	//status update
	printf("Bytes Sent: %ld\n", iResult);

	//SHUTDOWN CONNECTION FOR SENDING
	//CLIENT CAN STILL USE CONNECTSOCKET FOR RECEIVING==============
	iResult = shutdown(ConnectSocket, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("shutdown failed: %d\n", WSAGetLastError());
		closesocket(ConnectSocket);
		WSACleanup();
		return 1;
	}//endif

	//RECEIVE DATA UNTIL SERVER CLOSES THE CONNECTION===============
	do{
		iResult = recv(ConnectSocket, recvbuf, recvbuflen,0);
		if (iResult>0){
			printf("Bytes received: %d\n", iResult);
		}//endif
		else if (iResult==0){
			printf("Connection closed\n");
		}
		else{
			printf("recv failed: %d\n", WSAGetLastError());
		}
	} while (iResult > 0);

	//cleanup
	closesocket(ConnectSocket);
	WSACleanup();

	system("PAUSE");
	return 0;
}