#include <cstdio>
#include <cstring>
#include <fstream>
#include "Camera.h"

#include "opencv2/opencv.hpp"

using std::ofstream;

int main(int argc, char** argv)
{
	Camera cam;


	// ѭ����ȡ����ͷ��׽����֡����ʾ
	while (true)
	{
		// ��ȡһ֡ͼ��
		CameraFrame frame{ cam.GetFrame() };

		// ��ʾͼ��
		cv::imshow("Camera", frame);

		// ����q���˳�ѭ��
		if (cv::waitKey(1) == 'q')
			break;
	}

	// �ͷ����д���
	cv::destroyAllWindows();

	return 0;
}