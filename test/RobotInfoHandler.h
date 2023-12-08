#pragma once
#include <iostream>
#include <string>
#include "ClientSocket.h"
#include "RobotInfo.h"

class RobotInfoHandler
{
public:
	RobotInfoHandler() = default;

private:
	bool SendPos(const RobotInfo::RobotPos& pos);
	std::string RobotPosToString(const RobotInfo::RobotPos& pos);
private:
	static const int TCP_PORT = 9000;
	static const std::string TCP_IP_ADDR;
	ClientSocket m_Socket = ClientSocket(TCP_IP_ADDR, TCP_PORT);
	RobotInfo m_RInfo;
};