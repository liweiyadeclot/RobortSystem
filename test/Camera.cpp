#include "Camera.h"
#include "MvCameraControl.h"

class Camera::Implement {
public:
	void* handle;
	bool isOpen;
	bool isGrabbing;

	Implement(uint32_t index)
		:handle(nullptr),
		isOpen(false),
		isGrabbing(false)
	{
		MV_CC_DEVICE_INFO_LIST stDeviceList;
		memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
		int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet)
		{
			printf("Error:Enum Devices fail! nRet [0x%x]\n", nRet);
			return;
		}

		if (stDeviceList.nDeviceNum == 0)
		{
			printf("Error:No devices found!");
			return;
		}

		nRet = MV_CC_CreateHandle(&this->handle, stDeviceList.pDeviceInfo[index]);
		if (MV_OK != nRet)
		{
			printf("Error:Create Handle fail! nRet [0x%x]\n", nRet);
			return;
		}

		nRet = MV_CC_OpenDevice(this->handle);
		if (MV_OK != nRet)
		{
			printf("Error:Open Device fail! nRet [0x%x]\n", nRet);
			return;
		}

		isOpen = true;

		nRet = MV_CC_SetFloatValue(this->handle, "ExposureTime", 80000);
		if (MV_OK != nRet)
		{
			printf("Warning:Set camera attribute fail. nRet [0x%x]\n", nRet);
		}
	}

	~Implement() 
	{
		if (isGrabbing)
			this->StopGrabbing();
		int32_t nRet = MV_CC_CloseDevice(this->handle);
		if (MV_OK != nRet)
		{
			printf("Error:ClosDevice fail! nRet [0x%x]\n", nRet);
			return;
		}

		// Destroy handle
		nRet = MV_CC_DestroyHandle(this->handle);
		if (MV_OK != nRet)
		{
			printf("Error:Destroy Handle fail! nRet [0x%x]\n", nRet);
			return;
		}
		printf("Device successfully closed.\n");
	}

	static const std::vector<Camera::CameraInfo> ListCameras() 
	{
		std::vector<Camera::CameraInfo> ret{};
		MV_CC_DEVICE_INFO_LIST stDeviceList;
		memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
		int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet)
		{
			printf("Error:Enum Devices fail! nRet [0x%x]\n", nRet);
		}
		for(uint8_t i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			CameraInfo info{ 0 };
			info.index = i;
			ret.push_back(info);
		}
		return ret;
	}

	int32_t StartGrabbing() 
	{
		int32_t nRet = MV_CC_StartGrabbing(this->handle);
		if (MV_OK != nRet)
		{
			printf("Start grabbing fail! nRet [0x%x]\n", nRet);
		}
		return nRet;
	}

	int32_t StopGrabbing() 
	{
		int32_t nRet = MV_CC_StopGrabbing(this->handle);
		if (MV_OK != nRet)
		{
			printf("Stop grabbing fail! nRet [0x%x]\n", nRet);
		}
		return nRet;
	}

	CameraFrame GetFrame(Camera& cam) 
	{
		return CameraFrame(cam);
	}
};

Camera::Camera() 
	: Camera(0)
{
}

Camera::Camera(uint32_t index) 
	: pImpl(std::make_unique<Camera::Implement>(index))
{
}

Camera::~Camera() = default;

const std::vector<Camera::CameraInfo> Camera::ListCameras()
{
	return Camera::Implement::ListCameras();
}


bool Camera::IsOpen()
{
	return this->pImpl->isOpen;
}

int32_t Camera::StartGrabbing()
{
	return this->pImpl->StartGrabbing();
}

int32_t Camera::StopGrabbing()
{
	return this->pImpl->StopGrabbing();
}

CameraFrame Camera::GetFrame()
{
	return this->pImpl->GetFrame(*this);
}

class CameraFrame::Implement {
public:
	Camera& source;
	MV_FRAME_OUT pstFrame;
	bool isValid;

	Implement(Camera& source) :
		source(source),
		pstFrame(),
		isValid(false)
	{
		int32_t nRet = MV_CC_GetImageBuffer(source.pImpl->handle, &this->pstFrame, 100);
		if (MV_OK != nRet)
		{
			printf("Get image fail! nRet [0x%x]\n", nRet);
			return;
		}
		this->isValid = true;
	}

	~Implement()
	{
		if (this->isValid)
		{
			this->Free();
		}
	}

	const uint8_t* GetData() const
	{
		return this->pstFrame.pBufAddr;
	}

	uint64_t GetDataLength() const
	{
		return this->pstFrame.stFrameInfo.nFrameLen;
	}

	uint32_t GetImageHeight() const
	{
		return this->pstFrame.stFrameInfo.nHeight;
	}

	uint32_t GetImageWidth() const
	{
		return this->pstFrame.stFrameInfo.nWidth;
	}

	void Free()
	{
		int32_t nRet = MV_CC_FreeImageBuffer(this->source.pImpl->handle, &this->pstFrame);
		if (MV_OK != nRet)
		{
			printf("Free image fail! nRet [0x%x]\n", nRet);
			return;
		}
		this->isValid = false;
	}
};

CameraFrame::CameraFrame(Camera& source) : pImpl(std::make_unique<Implement>(source))
{
}

CameraFrame::~CameraFrame() = default;

CameraFrame::CameraFrame(const CameraFrame& other)
{
}

const uint8_t* CameraFrame::GetData() const
{
	return this->pImpl->GetData();
}

uint64_t CameraFrame::GetDataLength() const
{
	return this->pImpl->GetDataLength();
}

bool CameraFrame::IsValid() const
{
	return this->pImpl->isValid;
}

uint32_t CameraFrame::GetImageHeight() const
{
	return this->pImpl->GetImageHeight();
}

uint32_t CameraFrame::GetImageWidth() const
{
	return this->pImpl->GetImageWidth();
}

void CameraFrame::Free()
{
	this->pImpl->Free();
}