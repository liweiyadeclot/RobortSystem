#include <iostream>
#include "CameraManager.h"
#include "CameraControlerBase.h"

CameraManager::~CameraManager()
{
}

CameraManager* CameraManager::GetInstance()
{
	if (s_Instance == nullptr)
	{
		s_Instance = new CameraManager();
	}
	return s_Instance;

}

std::shared_ptr<Camera> CameraManager::GetOrOpenCamera(CameraIndex index)
{
	// index valid?
	if (index >= m_Cameras.size())
	{
		// Throw exception
		std::cout << "ERROR: Camera Index " << index << " invalid!" << std::endl;
		return nullptr;
	}
	else
	{
		bool OpenCameraSuccess = m_Cameras[index]->Open();
		if (OpenCameraSuccess)
		{
			return m_Cameras[index];
		}
		else
		{
			return nullptr;
		}
	}
}

void CameraManager::PrintCamerasInfo()
{
	// Print each kind of cameras.
	for (const auto& cameraControler : m_CameraControlers)
	{
		cameraControler->PrintCamerasInfo();
	}
}

