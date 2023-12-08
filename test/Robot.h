#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <cstdint>

#include <vector>
#include <memory>

#include "RobotImpl.h"

class Robot {
public:
	Robot();
	~Robot() = default;
	int Reset();
	int MoveTo(int angle1, int angle2, int angle3, int angle4, int angle5, int angle6);
	int GetJointAngle(uint8_t jointNum);
	std::vector<int> GetJointAngles();.
private:
	std::vector<int> m_JointAngles;
	std::unique_ptr<RobotImpl> m_Robot;
};

#endif // !_ROBOT_H_

