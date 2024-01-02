#pragma once

#include <vector>
#include "CameraControllerBase.h"
#include "Camera.h"
#include "MvCameraControl.h"

class HIKCamerasController : public CameraControllerBase
{
public:
	HIKCamerasController() = delete;

	// Push HIK Camera into static camera
	HIKCamerasController(std::vector<std::shared_ptr<Camera>>& cameras);

public:
	// Print HIK Camera info with HIK SDK
	void PrintCamerasInfo() override;

private:
	int m_NumHIKCameras;
	std::vector<MV_CC_DEVICE_INFO*> m_pDeviceInfos;
private:
	void PrintDeviceInfo(MV_CC_DEVICE_INFO* pDeviceInfo);
};