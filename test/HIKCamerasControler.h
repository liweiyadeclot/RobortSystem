#pragma once

#include "CameraControlerBase.h"
#include "Camera.h"
#include <vector>

class HIKCamerasControler : public CameraControlerBase
{
public:
	HIKCamerasControler() = delete;

	// Push HIK Camera into static camera
	HIKCamerasControler(std::vector<Camera>& cameras);

public:
	// Print HIK Camera info with HIK SDK
	void PrintCamerasInfo() override;

private:
	int m_NumCameras;
};