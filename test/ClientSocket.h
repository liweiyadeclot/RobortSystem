#pragma once
#include <string>
#include <memory>
class ClientSocketImpl;

class ClientSocket
{
public:
	ClientSocket() = delete;
	ClientSocket(const std::string& ipAddress, int port);
	ClientSocket(ClientSocket&& source) noexcept;
	~ClientSocket();
	
	int Send(const std::string buffer, size_t len);
	int Recv(std::string buffer, size_t len);

	ClientSocket& operator=(ClientSocket&&) noexcept;
private:
	std::unique_ptr<ClientSocketImpl> CSocketImplPtr;
};