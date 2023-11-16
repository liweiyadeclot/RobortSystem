#include <iostream>
#include <winsock2.h>
#include <WS2tcpip.h>
// Windows specific pragma, that sucks
#pragma comment(lib, "ws2_32.lib")

int main(int argc, char **argv)
{
	WSADATA _;
	memset(&_, 0, sizeof(_));
	std::cout << WSAStartup(MAKEWORD(2, 2), &_) << std::endl;// Windows specific socket initialization

	SOCKET client_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);//client socket, using IPv4 & TCP & IP
	std::cout << client_socket << std::endl;
	struct sockaddr_in server_addr;//structure for passing parameters in and out
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;  //server uses IPv4
	inet_pton(AF_INET, "192.168.1.13", &server_addr.sin_addr);  //set IP address of server
	server_addr.sin_port = htons(9000);  //set port of server

	std::cout << connect(client_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) << std::endl; // connect to server

	char buffer[256] = { 0 };
	std::cout << "send:";
	std::cin >> std::noskipws >> buffer;
	return -1 == send(client_socket, buffer, strlen(buffer), 0);
}