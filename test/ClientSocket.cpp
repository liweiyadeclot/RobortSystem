#include "ClientSocket.h"
#include "ClientSocketImpl.h"

ClientSocket::ClientSocket(const std::string& ipAddress, int port)
{
	CSocketImplPtr = std::make_unique<ClientSocketImpl>(ipAddress, port);
}

ClientSocket& ClientSocket::operator=(ClientSocket&& source) noexcept
{
	if (this != &source)
	{
		CSocketImplPtr = std::move(source.CSocketImplPtr);
	}
	return *this;
}

ClientSocket::ClientSocket(ClientSocket&& source) noexcept
{
	CSocketImplPtr = std::move(source.CSocketImplPtr);
}

ClientSocket::~ClientSocket() = default;

int ClientSocket::Send(const std::string buffer, size_t len)
{
	return CSocketImplPtr->Send(buffer, len);
}

int ClientSocket::Recv(std::string buffer, size_t len)
{
	return CSocketImplPtr->Recv(buffer, len);
}
