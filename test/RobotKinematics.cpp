#include "RobotKinematics.h"

using namespace Computing;

RobotKinematics::RobotKinematics(double dh1, double dh2, double dh3, double dh4, double dh5, double dh6)
{
	this->m_Implement = std::make_unique<RobotKinematicsImpl>(dh1, dh2, dh3, dh4, dh5, dh6);
}

RobotKinematics::JointAngles8d RobotKinematics::InverseResolve(double x, double y, double z, double rX, double rY, double rZ)
{
	return this->m_Implement->InverseResolve(x, y, z, rX, rY, rZ);
}