#ifndef _ROBOT_KINEMATICS_IMPL_H_
#define _ROBOT_KINEMATICS_IMPL_H_
#include "Computing.h"

class RobotKinematicsImpl {
public:
	using JointAngles8d = Computing::Matrix<double, 8, 6>;
	RobotKinematicsImpl() = delete;
	RobotKinematicsImpl(double dh1, double dh2, double dh3, double dh4, double dh5, double dh6);
	JointAngles8d InverseResolve(double x, double y, double z, double rX, double rY, double rZ);

private:
	using DHParams = Computing::Matrix<double, 1, 6>;
	DHParams m_DH;
};

#endif // !_ROBOT_KINEMATICS_IMPL_H_

