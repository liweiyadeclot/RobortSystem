#include <cstdio>
#include <cstring>
#include <fstream>
#include "CameraManager.h"
#include "opencv2/opencv.hpp"

using std::ofstream;

bool FindChessboradCorners(cv::InputArray image, cv::Size patternSize, std::vector<cv::Point2f> outCorners,
	int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE) 
{
	cv::Mat input = image.getMat();
	if (input.channels() > 1) 
	{
		input = cv::Mat();
		cv::cvtColor(image, input, cv::COLOR_BGR2GRAY);
	}
	bool found = false;
	found = cv::findChessboardCorners(input, patternSize, outCorners);
	if (found)
	{
		cv::cornerSubPix(input, outCorners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
	}
	return found;
}

int main(int argc, char** argv)
{
	CameraManager* s_CameraManager = CameraManager::GetInstance();
	std::shared_ptr<Camera> cam = s_CameraManager->GetOrOpenCamera();
	std::vector<std::vector<cv::Point2f> > imagePoints;


	// 循环读取摄像头捕捉到的帧并查找棋盘格点位
	while (true)
	{
		// 读取一帧图像
		CameraFrame frame{ cam->GetFrame() };

		cv::Mat view;
		// 缩小，不然我电脑显示不完全
		cv::resize(frame, view, cv::Size(640, 480));
		// 显示图像
		cv::imshow("Camera", view);
		// 等待回车键按下则采用，其他键按下则跳过，按下q键退出循环开始标定
		int key = cv::waitKey(1);
		if (key == 'q')
		{
			break;
		}
		else if (key != '\r')
		{
			continue;
		}


		std::vector<cv::Point2f> corners;
		if (true == FindChessboradCorners(frame, cv::Size(9, 6), corners))
		{
			imagePoints.push_back(corners);
			cv::drawChessboardCorners(frame, cv::Size(9, 6), corners, true);
		}
		else
		{
			cv::putText(frame, "Corners not found", cv::Point(0,0), 1, 1, cv::Scalar::all(255));
		}
		// 缩小，不然我电脑显示不完全
		cv::resize(frame.clone(), frame, cv::Size(640, 480));
		// 显示找到的角点
		cv::imshow("Camera", frame);

		// 等待任意键按下，按下q键退出循环开始标定
		if (cv::waitKey() == 'q')
			break;
	}

	// 释放所有窗口
	cv::destroyAllWindows();

	return 0;
}