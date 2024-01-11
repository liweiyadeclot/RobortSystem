#include "HIKCamera.h"
#include "CameraFrame.h"
#include <memory>
#include <opencv2/opencv.hpp>

HIKCamera::HIKCamera(MV_CC_DEVICE_INFO* pDeviceInfo)
	:m_handle(nullptr),
	m_info(*pDeviceInfo),
	m_isOpen(false)
{
	int32_t nRet = MV_CC_CreateHandle(&this->m_handle, &this->m_info);
	if (MV_OK != nRet)
	{
		printf("Create Handle fail! nRet [0x%x]\n", nRet);
		return;
	}
}

HIKCamera::~HIKCamera() {
	if (this->IsOpen())
	{
		this->Close();
	}

	// Destroy handle
	int32_t nRet = MV_CC_DestroyHandle(this->m_handle);
	if (MV_OK != nRet)
	{
		printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
		return;
	}
}

bool HIKCamera::Open()
{
	if (IsOpen())
	{
		return true;
	}

	// Open the specified device
	int32_t nRet = MV_CC_OpenDevice(this->m_handle);
	if (MV_OK != nRet)
	{
		printf("Open Device fail! nRet [0x%x]\n", nRet);
		return false;
	}

	//Set ExposureTime to make image brighter
	nRet = MV_CC_SetFloatValue(m_handle, "ExposureTime", 80000);
	if (MV_OK != nRet)
	{
		printf("Set ExposureTime fail! nRet [0x%x]\n", nRet);
	}

	// Start grab image
	nRet = MV_CC_StartGrabbing(this->m_handle);
	if (MV_OK != nRet)
	{
		printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
		return false;
	}
	this->m_isOpen = true;
	return true;
}

bool HIKCamera::Close()
{
	// Stop grab image
	int32_t nRet = MV_CC_StopGrabbing(this->m_handle);
	if (MV_OK != nRet)
	{
		printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
		return false;
	}

	// Close device
	nRet = MV_CC_CloseDevice(this->m_handle);
	if (MV_OK != nRet)
	{
		printf("ClosDevice fail! nRet [0x%x]\n", nRet);
		return false;
	}

	this->m_isOpen = false;
	return true;
}

bool HIKCamera::IsOpen() {
	return this->m_isOpen;
}

CameraFrame HIKCamera::GetFrame() {
	CameraFrame ret = CameraFrame();
	MV_FRAME_OUT rawFrame{ 0 };
	int32_t nRet = MV_CC_GetImageBuffer(this->m_handle, &rawFrame, 1000);
	if (nRet != MV_OK)
	{
		printf("GetFrame fail! nRet [0x%x]\n", nRet);
		return ret;
	}

	ret = CameraFrame(rawFrame.stFrameInfo.nHeight, rawFrame.stFrameInfo.nWidth, CV_8UC1, rawFrame.pBufAddr);

	nRet = MV_CC_FreeImageBuffer(this->m_handle, &rawFrame);
	if (nRet != MV_OK)
	{
		printf("Free Frame fail! nRet [0x%x]\n", nRet);
		return ret;
	}

	cv::cvtColor(ret.clone(), ret, cv::COLOR_BayerRG2BGR);
	return ret;
}