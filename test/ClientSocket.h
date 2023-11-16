#pragma once
#include <string>
#include <memory>
#include "ClientSocketImpl.h"

class ClientSocket
{
public:
	ClientSocket() = delete;
	ClientSocket(const std::string& ipAddress, int port);
	~ClientSocket() = default;
	
	int Send(const char* buffer, size_t len);
	int Recv(char* buffer, size_t len);

private:
	std::unique_ptr<ClientSocketImpl> CSocketImplPtr;
};