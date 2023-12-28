#pragma once


/// <summary>
/// CameraControlerBase is the base class of all CameraControler
/// provides basic function for upper user.
/// </summary>
class CameraControlerBase
{
public:
	CameraControlerBase() = default;

	virtual ~CameraControlerBase() = 0;
public:
	// Print CamerasInfo of a kind of Camera
	virtual void PrintCamerasInfo() = 0;
private:
	// No data.
};