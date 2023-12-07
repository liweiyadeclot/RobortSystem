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

	ClientSocket m_Socket;
	RobotInfo m_RInfo;
};