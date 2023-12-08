#include "RobotInfoHandler.h"

const std::string RobotInfoHandler::TCP_IP_ADDR = "192.168.1.12";

bool RobotInfoHandler::SendPos(const RobotInfo::RobotPos& pos)
{
    std::string buffer = RobotPosToString(pos);
    if (m_Socket.Send(buffer, buffer.length()) != -1)
        return true;
    else
        return false;
}

std::string RobotInfoHandler::RobotPosToString(const RobotInfo::RobotPos& pos)
{
    std::string buffer;

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
