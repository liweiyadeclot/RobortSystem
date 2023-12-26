#pragma once
#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H
#include <cstdint>
#include <vector>
#include <memory>

#include "Camera.h"

class CameraManager 
{
public:
	using CameraIndex = uint32_t;


	static std::shared_ptr<Camera> GetOrOpenCamera(CameraIndex index = 0);
private:
	static std::vector<std::shared_ptr<Camera>> m_Cameras;
	static CameraManager* Instance;
};

#endif // !CAMERA_MANAGER_H
