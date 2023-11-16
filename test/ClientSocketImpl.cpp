#include "ClientSocketImpl.h"

ClientSocketImpl::ClientSocketImpl(const std::string& ipAddress, int port) 
{
#ifdef _WIN32
	WSADATA wsaData;
	memset(&wsaData, 0, sizeof(wsaData));
	int result = -1;
	result = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (result != 0)
	{
		;
	}
	m_Socket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

	struct sockaddr_in serverAddr;//structure for passing parameters in and out
	memset(&serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;  //server uses IPv4
	inet_pton(AF_INET, ipAddress.c_str(), &serverAddr.sin_addr);  //set IP address of server
	serverAddr.sin_port = htons(port);  //set port of server

	connect(m_Socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
#endif // _WIN32

}

ClientSocketImpl::~ClientSocketImpl()
{
#ifdef _WIN32
	closesocket(m_Socket);
#endif // _WIN32

}

int ClientSocketImpl::Send(const std::string& buffer, size_t len)
{
	int sendResult = -1;
#ifdef _WIN32
	sendResult = send(m_Socket, buffer.c_str(), len, 0);
#endif // _WIN32
	return sendResult;
}

int ClientSocketImpl::Recv(std::string& buffer, size_t len)
{
	char charBuffer[1025] = { 0 };
	size_t recvLen = 0;
	while (recvLen < len) 
	{
		int recvResult = 0;
#ifdef _WIN32
		recvResult = recv(m_Socket, charBuffer, sizeof(charBuffer) - 1, 0);
#endif // _WIN32
		if (recvResult <= 0)
		{
			break;
		}
		else
		{
			recvLen += recvResult;
			buffer.append(charBuffer);
		}
	}
	return recvLen;
}
