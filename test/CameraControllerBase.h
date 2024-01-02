#pragma once


/// <summary>
/// CameraControlerBase is the base class of all CameraControler
/// provides basic function for upper user.
/// </summary>
class CameraControllerBase
{
public:
	CameraControllerBase() = default;

	virtual ~CameraControllerBase() = default;
public:
	// Print CamerasInfo of a kind of Camera
	virtual void PrintCamerasInfo() = 0;
private:
	// No data.
};