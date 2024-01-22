#pragma once
#include <iostream>
#include <string>
#include "ClientSocket.h"
#include "RobotInfo.h"

class RobotInfoHandler
{
public:
	RobotInfoHandler() = default;

	bool SendPos(const RobotInfo::RobotPos& pos);
	bool SendJoint(const RobotInfo::RobotJoint& joint);
	std::string RobotPosToString(const RobotInfo::RobotPos& pos);
	std::string RobotJointToString(const RobotInfo::RobotJoint& joint);
private:
	static const int TCP_PORT = 800;
	static const std::string TCP_IP_ADDR;
	ClientSocket m_Socket = ClientSocket(TCP_IP_ADDR, TCP_PORT);
	RobotInfo m_RInfo;
};