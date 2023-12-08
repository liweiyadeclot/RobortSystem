#include <iostream>
#include "ClientSocket.h"

int main(int argc, char** argv)
{
	const std::string serverAddr("192.168.1.12");
	ClientSocket client(serverAddr, 800);

	std::string buffer;
	std::cout << "send:";
	while (buffer != std::string("q"))
	{
		std::getline(std::cin, buffer);
		std::cout << buffer.size();
		client.Send(buffer, buffer.size());
	}

	return 0;
}