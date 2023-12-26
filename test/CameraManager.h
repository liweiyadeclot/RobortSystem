#pragma once
#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H
#include <cstdint>
#include <vector>

#include "Camera.h"

class CameraManager {
public:
	using CameraIndex = uint32_t;
	struct CameraInfo
	{
		CameraIndex index;
	};

	static const std::vector<CameraInfo> EnumCameras();
	static Camera Open();
	static Camera Open(CameraIndex index);
private:
	static CameraManager Instance;
};

#endif // !CAMERA_MANAGER_H
