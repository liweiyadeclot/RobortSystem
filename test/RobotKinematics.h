#ifndef _ROBOTKINEMATICS_H_
#define _ROBOTKINEMATICS_H_
#include <memory>
#include "Computing.h"
class RobotKinematicsImpl;

class RobotKinematics {
public:
	using JointAngles8d = Computing::Matrix<double, 8, 6>;
	RobotKinematics() = delete;
	RobotKinematics(double dh1, double dh2, double dh3, double dh4, double dh5, double dh6);
	JointAngles8d InverseResolve(double x, double y, double z, double rX, double rY, double rZ);

private:
	std::unique_ptr<RobotKinematicsImpl> m_Implement;
};

#endif // !_ROBOTKINEMATICS_H_
