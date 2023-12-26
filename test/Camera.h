#ifndef CAMERA_H
#define CAMERA_H

#include "CameraFrame.h"

class Camera {
public:
	Camera();
	~Camera();

public:
	virtual bool IsOpen() = 0;
	virtual CameraFrame GetFrame() = 0;
private:


};


#endif // !CAMERA_H

