#pragma once
#ifndef HIKCAMERA_H
#define HIKCAMERA_H
#include "Camera.h"
#include <cstdint>
#include "MvCameraControl.h"

class HIKCamera : Camera
{
public:
	~HIKCamera();

	bool Open();
	bool Close();
	bool IsOpen();
	CameraFrame GetFrame();
private:
	HIKCamera(MV_CC_DEVICE_INFO* pDeviceInfo);
	void* m_handle;
	MV_CC_DEVICE_INFO m_info;
	bool m_isOpen;
};

#endif // !HIKCAMERA_H
