#pragma once
#include <memory>
#include <string>

#include "opencv2/opencv.hpp"

#include "RobotMoveSubSystem.h"
#include "CameraManager.h"

void ShowPos(const std::string description, const std::vector<double>& pos)
{
	std::cout << description
		<< "x == " << pos[0] << ", "
		<< "y == " << pos[1] << ", "
		<< "z == " << pos[2] << ", "
		<< "a == " << pos[3] << ", "
		<< "b == " << pos[4] << ", "
		<< "c == " << pos[5] << ", "
		<< std::endl;
}

void CalibDataCollector()
{
	RobotMoveSubSystem robot;
	std::shared_ptr<Camera> camera{ CameraManager::GetInstance()->GetOrOpenCamera() };
	std::vector<double> currentPos{ -200, 500, 750, 135, 30, 160 };
	while (true)
	{
		robot.MoveToPos(currentPos[0], currentPos[1], currentPos[2], currentPos[3], currentPos[4], currentPos[5]);
		ShowPos("Current robot position/pose:", currentPos);
		cv::Mat frame{ camera->GetFrame() }, view;
		cv::resize(frame, view, cv::Size(640, 480));
		cv::imshow("Camera", view);
		cv::waitKey(1);
		std::string command;
		std::cin >> command;
		bool quit{ false };
		std::string imageFileName{ ".\\calib_data\\" };
		switch (command[0])
		{
		case 'x':
			currentPos[0] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'y':
			currentPos[1] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'z':
			currentPos[2] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'a':
			currentPos[3] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'b':
			currentPos[4] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'c':
			currentPos[5] = atof(command.substr(2, command.size() - 2).c_str());
			break;
		case 'p':
			for (double value : currentPos)
			{
				imageFileName.append(std::to_string(value));
				imageFileName.append(",");
			}
			imageFileName.append(".jpg");
			cv::imwrite(imageFileName, frame);
			break;
		case 'q':
			quit = true;
			break;
		default:
			break;
		}
		cv::destroyAllWindows();
		if (quit)
		{
			break;
		}
	}
}