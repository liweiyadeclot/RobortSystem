#ifndef _ROBOTIMPL_H_
#define _ROBOTIMPL_H_

class RobotImpl {
public:
	RobotImpl();
	~RobotImpl() = default;
	int Reset();
	int MoveTo(int angle1, int angle2, int angle3, int angle4, int angle5, int angle6);
};

#endif // !_ROBOTIMPL_H_

