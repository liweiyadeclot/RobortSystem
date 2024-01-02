#ifndef CAMERA_H
#define CAMERA_H

#include "CameraFrame.h"

class Camera {
public:
	Camera() = default;
	virtual ~Camera() =  default;

	virtual bool Open() = 0;
	virtual bool Close() = 0;
	virtual bool IsOpen() = 0;
	virtual CameraFrame GetFrame() = 0;
protected:
	// Camera() = default;
};


#endif // !CAMERA_H

