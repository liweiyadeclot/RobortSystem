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
		cv::Mat BayerFrame(frame.GetImageHeight(), frame.GetImageWidth(), CV_8UC1, (void*)(frame.GetData()));
		cv::Mat BGRFrame(frame.GetImageHeight(), frame.GetImageWidth(), CV_8UC3);
		// Convert pixel type from BayerRG8 to BGR
		cv::cvtColor(BayerFrame, BGRFrame, cv::COLOR_BayerRG2BGR);

		// ��ʾͼ��
		cv::imshow("Camera", BGRFrame);

		// ����q���˳�ѭ��
		if (cv::waitKey(1) == 'q')
			break;
	}

	// �ͷ����д���
	cv::destroyAllWindows();

	return 0;
}