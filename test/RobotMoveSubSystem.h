#pragma once
#include <iostream>
#include <memory>

class RobotInfoHandler;

class RobotMoveSubSystem
{
public:
	RobotMoveSubSystem();

	void MoveX(double distance);
	void MoveY(double distance);
	void MoveZ(double distance);
	void MoveU(double distance);
	void MoveV(double distance);
	void MoveW(double distance);

	void MoveJ0(double angle);
	void MoveJ1(double angle);
	void MoveJ2(double angle);
	void MoveJ3(double angle);
	void MoveJ4(double angle);
	void MoveJ5(double angle);
private:
	std::unique_ptr<RobotInfoHandler> m_MovepImpl;
};