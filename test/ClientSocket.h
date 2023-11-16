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
	
	int Send(const std::string buffer, size_t len);
	int Recv(std::string buffer, size_t len);

private:
	std::unique_ptr<ClientSocketImpl> CSocketImplPtr;
};