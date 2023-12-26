#include "CameraManager.h"
#include "Camera.h"
#include "HIKCamera.h"

#include <vector>

const std::vector<CameraManager::CameraInfo> CameraManager::EnumCameras() {
    // 枚举已连接的相机设备
    // 实现逻辑可能涉及遍历已连接的相机列表，并构建相应的CameraInfo结构体，存储相机的索引信息
    // 将构建的CameraInfo结构体放入一个std::vector中并返回
}

Camera CameraManager::Open() {
    HIKCamera cam = HIKCamera();
    return cam;
}

Camera CameraManager::Open(CameraIndex index) {
    // 打开指定索引的相机设备
    // 实现逻辑可能通过索引找到相应的相机设备，并返回一个打开了该设备的Camera实例
    // 如果指定索引的相机设备不存在，可以选择返回一个空的Camera实例或者抛出一个异常
}

CameraManager CameraManager::Instance;