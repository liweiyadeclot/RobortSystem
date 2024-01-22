#include "RobotInfoHandler.h"

const std::string RobotInfoHandler::TCP_IP_ADDR = "192.168.1.12";

bool RobotInfoHandler::SendPos(const RobotInfo::RobotPos& pos)
{
    std::string buffer = RobotPosToString(pos);
    std::string recv{};
    if (m_Socket.Send(buffer, buffer.length()) != -1)
        return m_Socket.Recv(buffer, 1) == 1 && recv == "o";
    else
        return false;
}

bool RobotInfoHandler::SendJoint(const RobotInfo::RobotJoint& joint)
{
    std::string buffer = RobotJointToString(joint);
    std::string recv{};
    if (m_Socket.Send(buffer, buffer.length()) != -1)
        return m_Socket.Recv(buffer, 1) == 1 && recv == "o";
    else
        return false;
}

std::string RobotInfoHandler::RobotPosToString(const RobotInfo::RobotPos& pos)
{
    std::string buffer("p,");

    buffer += std::to_string(pos.x);
    buffer += ",";
    buffer += std::to_string(pos.y);
    buffer += ",";
    buffer += std::to_string(pos.z);
    buffer += ",";
    buffer += std::to_string(pos.u);
    buffer += ",";
    buffer += std::to_string(pos.v);
    buffer += ",";
    buffer += std::to_string(pos.w);
    
    return buffer;

}

std::string RobotInfoHandler::RobotJointToString(const RobotInfo::RobotJoint& joint)
{
    std::string buffer("j,");

    buffer += std::to_string(joint.j0);
    buffer += ",";
    buffer += std::to_string(joint.j1);
    buffer += ",";
    buffer += std::to_string(joint.j2);
    buffer += ",";
    buffer += std::to_string(joint.j3);
    buffer += ",";
    buffer += std::to_string(joint.j4);
    buffer += ",";
    buffer += std::to_string(joint.j5);

    return buffer;

}