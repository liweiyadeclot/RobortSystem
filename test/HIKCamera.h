#pragma once
#ifndef HIKCAMERA_H
#define HIKCAMERA_H
#include "Camera.h"
class HIKCamera : public Camera
{
public:
	HIKCamera();
	~HIKCamera();

public:
	bool IsOpen();
	CameraFrame GetFrame();
};

#endif // !HIKCAMERA_H
