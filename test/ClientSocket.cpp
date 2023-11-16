#include "ClientSocket.h"

ClientSocket::ClientSocket(const std::string& ipAddress, int port)
{
	CSocketImplPtr = std::make_unique<ClientSocketImpl>(ipAddress, port);
}


int ClientSocket::Send(const std::string buffer, size_t len)
{
	return CSocketImplPtr->Send(buffer, len);
}

int ClientSocket::Recv(std::string buffer, size_t len)
{
	return CSocketImplPtr->Recv(buffer, len);
}
