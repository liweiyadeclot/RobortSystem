#include <iostream>
#include "ClientSocket.h"

int main(int argc, char** argv)
{
	const std::string serverAddr("192.168.1.13");
	ClientSocket client(serverAddr, 9000);

	std::string buffer;
	std::cout << "send:";
	while (buffer != std::string("q"))
	{
		std::getline(std::cin, buffer);
		client.Send(buffer, buffer.size());
	}

	return 0;
}