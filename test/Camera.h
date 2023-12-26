#ifndef CAMERA_H
#define CAMERA_H

#include <cstdint>
#include <memory>
#include <vector>

#include "CameraFrame.h"

class Camera {
public:
	struct CameraInfo
	{
		uint32_t index;
	};

	Camera();
	Camera(uint32_t index);
	~Camera();

	static const std::vector<CameraInfo> ListCameras();
	bool IsOpen();
	int32_t StartGrabbing();
	int32_t StopGrabbing();
	CameraFrame GetFrame();
private:
	class Implement;
	std::unique_ptr<Implement> pImpl;
};


#endif // !CAMERA_H

