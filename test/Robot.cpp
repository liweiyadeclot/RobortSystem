#include "Robot.h"

Robot::Robot()
{
	m_Robot = std::make_unique<RobotImpl>();
	if (m_Robot->Reset() != 0)
	{
		//TODO error log;
	}
	for (int i = 0; i < 6; i++)
	{
		m_JointAngles.push_back(0);
	}
}