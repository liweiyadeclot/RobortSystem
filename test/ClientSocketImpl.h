#pragma once
#include <string>

#ifdef _WIN32
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

class ClientSocketImpl
{
public:
	ClientSocketImpl() = delete;
	ClientSocketImpl(const std::string& ipAddress, int port);
	~ClientSocketImpl();

	int Send(const std::string& buffer, size_t len);
	int Recv(std::string& buffer, size_t len);
private:

#ifdef _WIN32
	SOCKET m_Socket;
#endif // _WIN32

};