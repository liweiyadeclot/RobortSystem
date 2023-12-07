#include "RobotInfo.h"

void RobotInfo::UpdateRobotInfo(RobotPos deltaPos, RobotJoint deltaJoint)
{
	m_CurrPos += deltaPos;
	m_CurrJoint += deltaJoint;
}

RobotInfo::RobotPos RobotInfo::GetCurrPos()
{
	return m_CurrPos;
}

RobotInfo::RobotJoint RobotInfo::GetCurrJoint()
{
	return m_CurrJoint;
}
