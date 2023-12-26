#include <cstdio>
#include <cstring>
#include <fstream>
#include "Camera.h"

#include "opencv2/opencv.hpp"

using std::ofstream;

int main(int argc, char** argv)
{
	Camera cam;


	// 循环读取摄像头捕捉到的帧并显示
	while (true)
	{
		// 读取一帧图像
		CameraFrame frame{ cam.GetFrame() };
		cv::Mat BayerFrame(frame.GetImageHeight(), frame.GetImageWidth(), CV_8UC1, (void*)(frame.GetData()));
		cv::Mat BGRFrame(frame.GetImageHeight(), frame.GetImageWidth(), CV_8UC3);
		// Convert pixel type from BayerRG8 to BGR
		cv::cvtColor(BayerFrame, BGRFrame, cv::COLOR_BayerRG2BGR);

		// 显示图像
		cv::imshow("Camera", BGRFrame);

		// 按下q键退出循环
		if (cv::waitKey(1) == 'q')
			break;
	}

	// 释放所有窗口
	cv::destroyAllWindows();

	return 0;
}