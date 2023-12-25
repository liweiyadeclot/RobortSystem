#include <cstdio>
#include <cstring>
#include <fstream>
#include "MvCameraControl.h"
#include "Camera.h"

#define USE_OPENCV 1

#if USE_OPENCV
#include "opencv2/opencv.hpp"
#endif

using std::ofstream;

int main(int argc, char** argv)
{
	/*初始化运行环境*/
	void* handle;
	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
	int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("Enum Devices fail! nRet [0x%x]\n", nRet);
		return -1;
	}

	if (stDeviceList.nDeviceNum > 0)
	{
		for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				return -1;
			}
		}
	}
	else
	{
		printf("Find No Devices!\n");
		return -1;
	}

	//printf("Please Input camera index:");
	unsigned int nIndex = 0;
	//scanf_s("%d", &nIndex);

	if (nIndex >= stDeviceList.nDeviceNum)
	{
		printf("Input error!\n");
		return -1;
	}

	// Select device and create handle
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
	if (MV_OK != nRet)
	{
		printf("Create Handle fail! nRet [0x%x]\n", nRet);
		return -1;
	}

	// Open the specified device
	nRet = MV_CC_OpenDevice(handle);
	if (MV_OK != nRet)
	{
		printf("Open Device fail! nRet [0x%x]\n", nRet);
		return -1;
	}

	// Set camera attribute
	nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 80000);
	if (MV_OK != nRet)
	{
		printf("Set camera attribute fail! nRet [0x%x]\n", nRet);
		return -1;
	}

	// Start grabbing images
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("Start grabbing fail! nRet [0x%x]\n", nRet);
	}

	// Get image frame
	MV_FRAME_OUT pstFrame;
	nRet = MV_CC_GetImageBuffer(handle, &pstFrame, 100);
	nRet = MV_CC_GetImageBuffer(handle, &pstFrame, 100);
	if (MV_OK != nRet)
	{
		printf("Get image fail! nRet [0x%x]\n", nRet);
	}
	else
	{
		// Save raw data
		printf("Frame size:%d\n", pstFrame.stFrameInfo.nFrameLen);
		char str[16] = "";
		snprintf(str, sizeof(str), "%dx%d.raw", pstFrame.stFrameInfo.nWidth, pstFrame.stFrameInfo.nHeight);
		ofstream ofs(str, std::ios::out | std::ios::binary);
		ofs.write((const char*)(pstFrame.pBufAddr), pstFrame.stFrameInfo.nFrameLen);
		ofs.close();
	}

	if (pstFrame.stFrameInfo.nFrameLen < 1)
	{
		printf("Warning: frame too small:%d\n", pstFrame.stFrameInfo.nFrameLen);
	}

	// Convert raw data to bmp image
#if USE_OPENCV
	cv::Mat BayerFrame(pstFrame.stFrameInfo.nHeight, pstFrame.stFrameInfo.nWidth, CV_8UC1, (void*)(pstFrame.pBufAddr));
	cv::Mat BGRFrame(pstFrame.stFrameInfo.nHeight, pstFrame.stFrameInfo.nWidth, CV_8UC3);
	// Convert pixel type from BayerRG8 to BGR
	cv::cvtColor(BayerFrame, BGRFrame, cv::COLOR_BayerRG2BGR);
	//cv::imshow("capture", BGRFrame);
	//cv::waitKey();
	// Write image frame into file
	cv::imwrite("output.bmp", BGRFrame);
#else
	MV_SAVE_IMAGE_PARAM_EX3 pstSaveParam{ 0 };
	unsigned char* imageFileBuffer = (unsigned char*)malloc(4 * pstFrame.stFrameInfo.nFrameLen);
	if (imageFileBuffer == nullptr)
	{
		printf("malloc fail!\n");
	}
	// Attrbute of input
	pstSaveParam.enPixelType = MvGvspPixelType::PixelType_Gvsp_BayerBG8;
	pstSaveParam.pData = pstFrame.pBufAddr;
	pstSaveParam.nHeight = pstFrame.stFrameInfo.nHeight;
	pstSaveParam.nWidth = pstFrame.stFrameInfo.nWidth;
	pstSaveParam.nDataLen = pstFrame.stFrameInfo.nFrameLen;
	//pstSaveParam.nJpgQuality = 99;
	// Attribute of output
	pstSaveParam.iMethodValue = 0;
	pstSaveParam.nBufferSize = 4 * pstFrame.stFrameInfo.nFrameLen;
	pstSaveParam.pImageBuffer = imageFileBuffer;
	pstSaveParam.enImageType = MV_SAVE_IAMGE_TYPE::MV_Image_Bmp;
	nRet = MV_CC_SaveImageEx3(handle, &pstSaveParam);

	if (nRet == MV_OK)
	{
		// Write image frame into file
		ofstream ofs("output.bmp", std::ios::out | std::ios::binary);
		ofs.write((const char*)(imageFileBuffer), pstSaveParam.nImageLen);
		ofs.close();
	}
	else
	{
		printf("Convert fail! nRet [0x%x]\n", nRet);
	}
#endif // USE_OPENCV

	// Free image frame
	nRet = MV_CC_FreeImageBuffer(handle, &pstFrame);
	if (MV_OK != nRet)
	{
		printf("Free image fail! nRet [0x%x]\n", nRet);
	}

	// Stop grabbing images
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("Stop grabbing fail! nRet [0x%x]\n", nRet);
	}

	// Close device
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("ClosDevice fail! nRet [0x%x]\n", nRet);
		return -1;
	}

	// Destroy handle
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
		return -1;
	}
	printf("Device successfully closed.\n");

	return 0;
}