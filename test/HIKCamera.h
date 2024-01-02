#pragma once
#ifndef HIKCAMERA_H
#define HIKCAMERA_H
#include "Camera.h"
#include <cstdint>
#include "MvCameraControl.h"

class HIKCamera : public Camera
{
public:
	~HIKCamera();
	HIKCamera() = delete;
	HIKCamera(MV_CC_DEVICE_INFO* pDeviceInfo);

	bool Open();
	bool Close();
	bool IsOpen();
	CameraFrame GetFrame();
private:
	void* m_handle;
	MV_CC_DEVICE_INFO m_info;
	bool m_isOpen;
};

#endif // !HIKCAMERA_H
