#pragma once
#include <string>

class RobotInfo
{
public:
	struct RobotPos
	{
		RobotPos(double x, double y, double z,
			double u, double v, double w) :
			x(x), y(y), z(z), u(u), v(v), w(w) {}

		RobotPos operator+(const RobotPos& rhs)
		{
			return RobotPos(x + rhs.x, y + rhs.y, z + rhs.z,
				u + rhs.u, v + rhs.v, w + rhs.w);
		}

		RobotPos& operator=(const RobotPos& rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			u = rhs.u;
			v = rhs.v;
			w = rhs.w;
			return *this;
		}

		void operator+=(const RobotPos& rhs)
		{
			*this = rhs;
		}

		double x, y, z, u, v, w;
	};

	struct RobotJoint
	{
		RobotJoint(double x, double y, double z,
			double u, double v, double w) :
			j0(x), j1(y), j2(z), j3(u), j4(v), j5(w) {}

		double j0, j1, j2, j3, j4, j5;

		RobotJoint operator+(const RobotJoint& rhs)
		{
			return RobotJoint(j0 + rhs.j0, j1 + rhs.j1, j2 + rhs.j2,
				j3 + rhs.j3, j4 + rhs.j4, j5 + rhs.j5);
		}

		RobotJoint& operator=(const RobotJoint& rhs)
		{
			j0 = rhs.j0;
			j1 = rhs.j1;
			j2 = rhs.j2;
			j3 = rhs.j3;
			j4 = rhs.j4;
			j5 = rhs.j5;
			return *this;
		}

		void operator+=(const RobotJoint& rhs)
		{
			*this = rhs;
		}
	};

	RobotInfo() : m_CurrPos(0, 0, 0, 0, 0, 0),
		m_CurrJoint(0, 0, 0, 0, 0, 0){}

	void UpdateRobotInfo(RobotPos deltaPos, RobotJoint deltaJoint);
	RobotPos GetCurrPos();
	RobotJoint GetCurrJoint();
private:
	RobotPos m_CurrPos;
	RobotJoint m_CurrJoint;
	// TO-DO: Original point
};