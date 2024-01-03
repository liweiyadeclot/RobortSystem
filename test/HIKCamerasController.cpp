#include <memory>
#include <iostream>
#include "HIKCamerasController.h"
#include "MvCameraControl.h"
#include "HIKCamera.h"


HIKCamerasController::HIKCamerasController(std::vector<std::shared_ptr<Camera>>& cameras)
{
	PrintCamerasInfo();
	// Push HIKCamera into cameras
	for (size_t i = 0; i < m_NumHIKCameras; ++i)
	{
		cameras.push_back(std::make_unique<HIKCamera>(m_pDeviceInfos[i]));
	}

}

void HIKCamerasController::PrintCamerasInfo()
{
	MV_CC_DEVICE_INFO_LIST deviceList;
	memset(&deviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

	// Enum every HIK device
	int ResultEnumDecives = MV_CC_EnumDevices(MV_USB_DEVICE, &deviceList);
	if (ResultEnumDecives != MV_OK)
	{
		std::cout << "Enum Devices fail! ERROR " << ResultEnumDecives << std::endl;
	}
	
	// Refresh device numbers.
	m_NumHIKCameras = deviceList.nDeviceNum;
	// Refresh devices infos.
	m_pDeviceInfos.resize(0);
	for (size_t i = 0; i < m_NumHIKCameras; ++i)
	{
		m_pDeviceInfos.push_back(deviceList.pDeviceInfo[i]);
	}

	if (m_NumHIKCameras > 0)
	{
		for (unsigned int i = 0; i < deviceList.nDeviceNum; i++)
		{
			std::cout << "[device " << i << "]:" << std::endl;
			MV_CC_DEVICE_INFO* pDeviceInfo = deviceList.pDeviceInfo[i];
			if (nullptr == pDeviceInfo)
			{
				break;
			}
			HIKCamerasController::PrintDeviceInfo(pDeviceInfo);
		}
	}
	else
	{
		// Find no device
		std::cout << "No device!" << std::endl;
	}
}

void HIKCamerasController::PrintDeviceInfo(MV_CC_DEVICE_INFO* pDeviceInfo)
{
	if (nullptr == pDeviceInfo)
	{
		// throw exception
		std::cout << "The Pointer of pstMVDevInfo is NULL!" << std::endl;
	}
	if (pDeviceInfo->nTLayerType == MV_USB_DEVICE)
	{
		std::cout << "chPortID: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chPortID << "]" << std::endl;
		std::cout << "chModelName: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chModelName << "]" << std::endl;
		std::cout << "chFamilyName: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chFamilyName << "]" << std::endl;
		std::cout << "chDeviceVersion: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chDeviceVersion << "]" << std::endl;
		std::cout << "chManufacturerName: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chManufacturerName << "]" << std::endl;
		std::cout << "Serial Number:: [" << pDeviceInfo->SpecialInfo.stCamLInfo.chSerialNumber << "]" << std::endl;
	}
	else
	{
		std::cout << "Not support." << std::endl;
	}


}
