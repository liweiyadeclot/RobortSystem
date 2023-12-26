#include "CameraManager.h"
#include "Camera.h"
#include "HIKCamera.h"

#include <vector>

const std::vector<CameraManager::CameraInfo> CameraManager::EnumCameras() {
    // ö�������ӵ�����豸
    // ʵ���߼������漰���������ӵ�����б���������Ӧ��CameraInfo�ṹ�壬�洢�����������Ϣ
    // ��������CameraInfo�ṹ�����һ��std::vector�в�����
}

Camera CameraManager::Open() {
    HIKCamera cam = HIKCamera();
    return cam;
}

Camera CameraManager::Open(CameraIndex index) {
    // ��ָ������������豸
    // ʵ���߼�����ͨ�������ҵ���Ӧ������豸��������һ�����˸��豸��Cameraʵ��
    // ���ָ������������豸�����ڣ�����ѡ�񷵻�һ���յ�Cameraʵ�������׳�һ���쳣
}

CameraManager CameraManager::Instance;