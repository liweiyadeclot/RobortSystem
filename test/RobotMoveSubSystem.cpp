#include "RobotMoveSubSystem.h"
#include "RobotInfoHandler.h"

RobotMoveSubSystem::RobotMoveSubSystem()
{
	m_MovepImpl = std::make_unique<RobotInfoHandler>();
}

RobotMoveSubSystem::~RobotMoveSubSystem() = default;

void RobotMoveSubSystem::MoveToPos(double x, double y, double z, double u, double v, double w)
{
    m_MovepImpl->SendPos(RobotInfo::RobotPos(x,y,z,u,v,w));
}

void RobotMoveSubSystem::MoveToJoint(double x, double y, double z, double u, double v, double w)
{
    m_MovepImpl->SendJoint(RobotInfo::RobotJoint(x, y, z, u, v, w));
}

void RobotMoveSubSystem::MoveX(double distance) {
    // MoveX函数的实现
}

void RobotMoveSubSystem::MoveY(double distance) {
    // MoveY函数的实现
}

void RobotMoveSubSystem::MoveZ(double distance) {
    // MoveZ函数的实现
}

void RobotMoveSubSystem::MoveU(double distance) {
    // MoveU函数的实现
}

void RobotMoveSubSystem::MoveV(double distance) {
    // MoveV函数的实现
}

void RobotMoveSubSystem::MoveW(double distance) {
    // MoveW函数的实现
}

void RobotMoveSubSystem::MoveJ0(double angle) {
    // MoveJ0函数的实现
}

void RobotMoveSubSystem::MoveJ1(double angle) {
    // MoveJ1函数的实现
}

void RobotMoveSubSystem::MoveJ2(double angle) {
    // MoveJ2函数的实现
}

void RobotMoveSubSystem::MoveJ3(double angle) {
    // MoveJ3函数的实现
}

void RobotMoveSubSystem::MoveJ4(double angle) {
    // MoveJ4函数的实现
}

void RobotMoveSubSystem::MoveJ5(double angle) {
    // MoveJ5函数的实现
}