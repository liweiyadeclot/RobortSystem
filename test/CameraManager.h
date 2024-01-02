#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#include "Camera.h"

class CameraControllerBase;

class CameraManager
{
public:
	CameraManager(const CameraManager& rhs) = delete;
	const CameraManager& operator=(const CameraManager& rhs) = delete;
public:
	using CameraIndex = uint32_t;

	static CameraManager* GetInstance();

	// Get a useable Camera, if the camera is not opened, open it.
	static std::shared_ptr<Camera> GetOrOpenCamera(CameraIndex index = 0);

	// Print all Camera infomation to screen
	static void PrintCamerasInfo();
	
private:
	// Init CameraControllers
	CameraManager();

	~CameraManager();
private:
	static std::vector<std::shared_ptr<Camera>> m_Cameras;

	static std::vector<std::unique_ptr<CameraControllerBase>> m_CameraControllers;

	static CameraManager* s_Instance;
};

