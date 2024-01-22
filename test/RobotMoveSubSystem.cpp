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
    // MoveX������ʵ��
}

void RobotMoveSubSystem::MoveY(double distance) {
    // MoveY������ʵ��
}

void RobotMoveSubSystem::MoveZ(double distance) {
    // MoveZ������ʵ��
}

void RobotMoveSubSystem::MoveU(double distance) {
    // MoveU������ʵ��
}

void RobotMoveSubSystem::MoveV(double distance) {
    // MoveV������ʵ��
}

void RobotMoveSubSystem::MoveW(double distance) {
    // MoveW������ʵ��
}

void RobotMoveSubSystem::MoveJ0(double angle) {
    // MoveJ0������ʵ��
}

void RobotMoveSubSystem::MoveJ1(double angle) {
    // MoveJ1������ʵ��
}

void RobotMoveSubSystem::MoveJ2(double angle) {
    // MoveJ2������ʵ��
}

void RobotMoveSubSystem::MoveJ3(double angle) {
    // MoveJ3������ʵ��
}

void RobotMoveSubSystem::MoveJ4(double angle) {
    // MoveJ4������ʵ��
}

void RobotMoveSubSystem::MoveJ5(double angle) {
    // MoveJ5������ʵ��
}