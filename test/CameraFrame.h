#ifndef CAMERA_FRAME_H
#define CAMERA_FRAME_H
#include <cstdint>
#include <memory>

class Camera;

class CameraFrame {
public:
	~CameraFrame();
	const uint8_t* GetData() const;
	uint64_t GetDataLength() const;
	bool IsValid() const;
	void Free();

private:
	class Implement;
	std::unique_ptr<Implement> pImpl;

	friend class Camera;
	CameraFrame(Camera& source);
};

#endif // !CAMERA_FRAME_H

