#pragma once
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>

#include "CustomSystemMoveDemo.h"

void PoseToR(const std::vector<double> pose, cv::Mat& R_end2robot)
{
	double rz{ pose[0] * CV_PI / 180.0f }, ry{ pose[1] * CV_PI / 180.0f }, rx{ pose[2] * CV_PI / 180.0f };
	double sinRz{ sin(rz) }, cosRz{ cos(rz) }, sinRy{ sin(ry) }, cosRy{ cos(ry) }, sinRx{ sin(rx) }, cosRx{ cos(rx) };

	// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cosRx, -sinRx,
		0, sinRx, cosRx
		);
	// Calculate rotation about y axis
	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
		cosRy, 0, sinRy,
		0, 1, 0,
		-sinRy, 0, cosRy
		);
	// Calculate rotation about z axis
	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
		cosRz, -sinRz, 0,
		sinRz, cosRz, 0,
		0, 0, 1
		);
	// Combined rotation matrix
	R_end2robot = R_z * R_y * R_x;
}

void GenCircleOnFixedZ(const std::vector<double>& center3d, double radius, double z, uint16_t positionCount, std::vector<std::vector<double>>& positionVec)
{
	double circleRadius{ sqrt(radius * radius - (z - center3d[2]) * (z - center3d[2])) };
	for (uint16_t i = 0; i < positionCount; i++)
	{
		double theta{ (2 * (double)i / (double)positionCount) * CV_PI };
		double x{ cos(theta) * circleRadius + center3d[0] }, y{ sin(theta) * circleRadius + center3d[1] };
		positionVec.push_back({ x, y, z });
	}
}

void AimAtLowerPoint(const std::vector<double>& selfPosition3d, const std::vector<double>& target3d, std::vector<double>& pose)
{
	double x{ target3d[0] - selfPosition3d[0] },
		y{ target3d[1] - selfPosition3d[1] },
		z{ target3d[2] - selfPosition3d[2] };
	double ry{ CV_PI - atan(-z / sqrt(x * x + y * y)) }, rz{ atan2(y,x) };
	pose.resize(3, 0);
	pose[1] = ry * 180 / CV_PI;
	pose[0] = rz * 180 / CV_PI;
}

void TurnToVerticalDirection(cv::Mat& aimedR)
{
	double rz{ 0 };
	if (aimedR.at<double>(2, 0) > 0)
	{
		rz = atan(aimedR.at<double>(2, 1) / aimedR.at<double>(2, 0));
	}
	else if (aimedR.at<double>(2, 1) > 0)
	{
		rz = CV_PI / 2 - atan(aimedR.at<double>(2, 0) / aimedR.at<double>(2, 1));
	}
	aimedR = aimedR * (cv::Mat_<double>(3, 3) <<
		cos(rz), -sin(rz), 0,
		sin(rz), cos(rz), 0,
		0, 0, 1);
}

void AimAtLowerPointVertically(const std::vector<double>& selfPosition3d, const std::vector<double>& target3d, cv::Mat& aimedR)
{
	std::vector<double> pose;
	AimAtLowerPoint(selfPosition3d, target3d, pose);
	PoseToR(pose, aimedR);
	TurnToVerticalDirection(aimedR);
}

void AimOnUpperHemisphereVertically(const std::vector<double>&& center3d, double radius, uint16_t layerCount, uint16_t posCountEachLayer, std::vector<std::vector<double>>& posVec)
{
	for (int i = 1; i <= layerCount; i++)
	{
		double z{ sin(i * CV_PI / 2 / (layerCount + 1)) * radius + center3d[2] };
		std::vector<std::vector<double>> positionVec;
		GenCircleOnFixedZ(center3d, radius, z, posCountEachLayer, positionVec);
		size_t originalSize{ posVec.size() };
		posVec.resize(originalSize + posCountEachLayer);
		for (int j = 0; j < posCountEachLayer; j++)
		{
			cv::Mat R_camera2custom;
			AimAtLowerPointVertically(positionVec[j], center3d, R_camera2custom);
			cv::Mat t{ cv::Mat(positionVec[j], false) };
			RTToPos(R_camera2custom, t, posVec[originalSize + j]);
		}
	}
}

void AimOnUpperHemisphere(const std::vector<double>&& center3d, double radius, uint16_t layerCount, uint16_t posCountEachLayer, std::vector<std::vector<double>>& posVec)
{
	for (int i = 1; i <= layerCount; i++)
	{
		double z{ sin(i * CV_PI / 2 / (layerCount + 1)) * radius + center3d[2] };
		std::vector<std::vector<double>> positionVec;
		GenCircleOnFixedZ(center3d, radius, z, posCountEachLayer, positionVec);
		size_t originalSize{ posVec.size() };
		posVec.resize(originalSize + posCountEachLayer);
		for (int j = 0; j < posCountEachLayer; j++)
		{
			std::vector<double> pose;
			AimAtLowerPoint(positionVec[j], center3d, pose);
			posVec[originalSize + j].resize(6);
			for (int k = 0; k < 3; k++)
			{
				posVec[originalSize + j][k] = positionVec[j][k];
				posVec[originalSize + j][3 + k] = pose[k];
			}
		}
	}
}

void TestAimOnUpperHemisphereVertically(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 6);
	const uint32_t SQUARE_SIZE{ 30 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};
	std::vector<std::vector<double>> posVec{};
	AimOnUpperHemisphereVertically({ 75,120,0 }, 400, 1, 15, posVec);
	for (int i = 0; i < posVec.size(); i++)
	{
		std::vector<double> pos{ posVec[i] };
		std::cout << "Pos:"
			<< pos[0] << ","
			<< pos[1] << ","
			<< pos[2] << ","
			<< pos[3] << ","
			<< pos[4] << ","
			<< pos[5] << std::endl;
	}

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 0, 400, 750, 135, 0, 180 });
		robotPosVec.emplace_back<std::vector<double>>({ 300, 400, 750, 135, -30, 200 });
		robotPosVec.emplace_back<std::vector<double>>({ -200, 300, 750, 135, 0, 160 });
		robotPosVec.emplace_back<std::vector<double>>({ 20, 200, 600, 135, -30, 160 });
		RobotMoveSubSystem robotMovement;
		std::shared_ptr<Camera> cam = CameraManager::GetInstance()->GetOrOpenCamera();
		for (std::vector<double> pos : robotPosVec)
		{
			robotMovement.MoveToPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
			while (cv::waitKey(1) == -1)
			{
				cv::Mat view;
				cv::resize(cam->GetFrame(), view, cv::Size(640, 480));
				cv::imshow("Camera", view);
			}
			images.push_back(cam->GetFrame());
		}
		cv::destroyAllWindows();
		cv::Mat cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
		int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
		std::vector<double> robotPos;
		for (auto pos : posVec)
		{
			CustomSystemMove(pos, R_obj2base, t_obj2base, R_gripper2cam, t_gripper2cam, robotPos);
			robotMovement.MoveToPos(robotPos[0], robotPos[1], robotPos[2], robotPos[3], robotPos[4], robotPos[5]);
			cv::Mat view;
			cv::resize(CameraManager::GetInstance()->GetOrOpenCamera()->GetFrame(), view, cv::Size(640, 480));
			cv::imshow("Camera", view);
			cv::waitKey();

		}
		cv::waitKey();
		cv::destroyAllWindows();
	}
}

void TestAimOnUpperHemisphere(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 6);
	const uint32_t SQUARE_SIZE{ 30 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};
	std::vector<std::vector<double>> posVec{ {0,0,300, 0, 0, 180} ,
	 {0,30,300, 0, 0, 180} ,
	 {0,60,300, 0, 0, 180} ,
	 {0,90,300, 0, 0, 180} ,
	 {0,120,300, 0, 0, 180} ,
	 {0,150,300, 0, 0, 180} };
	AimOnUpperHemisphere({ 75,120,0 }, 350, 1, 15, posVec);
	for (int i = 0; i < posVec.size(); i++)
	{
		std::vector<double> pos{ posVec[i] };
		std::cout << "Pos:"
			<< pos[0] << ","
			<< pos[1] << ","
			<< pos[2] << ","
			<< pos[3] << ","
			<< pos[4] << ","
			<< pos[5] << std::endl;
	}

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 0, 400, 750, 135, 0, 180 });
		robotPosVec.emplace_back<std::vector<double>>({ 300, 400, 750, 135, -30, 200 });
		robotPosVec.emplace_back<std::vector<double>>({ -200, 300, 750, 135, 0, 160 });
		robotPosVec.emplace_back<std::vector<double>>({ 20, 200, 600, 135, -30, 160 });
		RobotMoveSubSystem robotMovement;
		std::shared_ptr<Camera> cam = CameraManager::GetInstance()->GetOrOpenCamera();
		for (std::vector<double> pos : robotPosVec)
		{
			robotMovement.MoveToPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
			while (cv::waitKey(1) == -1)
			{
				cv::Mat view;
				cv::resize(cam->GetFrame(), view, cv::Size(640, 480));
				cv::imshow("Camera", view);
			}
			images.push_back(cam->GetFrame());
		}
		cv::destroyAllWindows();
		cv::Mat cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
		int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
		std::vector<double> robotPos;
		for (auto pos : posVec)
		{
			CustomSystemMove(pos, R_obj2base, t_obj2base, R_gripper2cam, t_gripper2cam, robotPos);
			robotMovement.MoveToPos(robotPos[0], robotPos[1], robotPos[2], robotPos[3], robotPos[4], robotPos[5]);
			cv::Mat view;
			cv::resize(CameraManager::GetInstance()->GetOrOpenCamera()->GetFrame(), view, cv::Size(640, 480));
			cv::imshow("Camera", view);
			cv::waitKey();

		}
		cv::waitKey();
		cv::destroyAllWindows();
	}
}

void TestGenCircleOnFixedZ(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 6);
	const uint32_t SQUARE_SIZE{ 30 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};
	std::vector<std::vector<double>> posVec{ {0,0,300, 0, 0, 180} ,
	 {0,30,300, 0, 0, 180} ,
	 {0,60,300, 0, 0, 180} ,
	 {0,90,300, 0, 0, 180} ,
	 {0,120,300, 0, 0, 180} ,
	 {0,150,300, 0, 0, 180} };
	size_t originalSize{ posVec.size() };
	std::vector<std::vector<double>> positionVec{};
	GenCircleOnFixedZ({ 75,120,0 }, 350, 300, 15, positionVec);
	posVec.resize(originalSize + positionVec.size(), { 0,0,0,0,0,180 });
	for (int i = 0; i < positionVec.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			posVec[originalSize + i][j] = positionVec[i][j];
		}
	}

	for (int i = 0; i < posVec.size(); i++)
	{
		std::vector<double> pos{ posVec[i] };
		std::cout << "Pos:"
			<< pos[0] << ","
			<< pos[1] << ","
			<< pos[2] << ","
			<< pos[3] << ","
			<< pos[4] << ","
			<< pos[5] << std::endl;
	}

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 0, 400, 750, 135, 0, 180 });
		robotPosVec.emplace_back<std::vector<double>>({ 300, 400, 750, 135, -30, 200 });
		robotPosVec.emplace_back<std::vector<double>>({ -200, 300, 750, 135, 0, 160 });
		robotPosVec.emplace_back<std::vector<double>>({ 20, 200, 600, 135, -30, 160 });
		RobotMoveSubSystem robotMovement;
		std::shared_ptr<Camera> cam = CameraManager::GetInstance()->GetOrOpenCamera();
		for (std::vector<double> pos : robotPosVec)
		{
			robotMovement.MoveToPos(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
			while (cv::waitKey(1) == -1)
			{
				cv::Mat view;
				cv::resize(cam->GetFrame(), view, cv::Size(640, 480));
				cv::imshow("Camera", view);
			}
			images.push_back(cam->GetFrame());
		}
		cv::destroyAllWindows();
		cv::Mat cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
		int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
		std::vector<double> robotPos;
		for (auto pos : posVec)
		{
			CustomSystemMove(pos, R_obj2base, t_obj2base, R_gripper2cam, t_gripper2cam, robotPos);
			robotMovement.MoveToPos(robotPos[0], robotPos[1], robotPos[2], robotPos[3], robotPos[4], robotPos[5]);
			cv::Mat view;
			cv::resize(CameraManager::GetInstance()->GetOrOpenCamera()->GetFrame(), view, cv::Size(640, 480));
			cv::imshow("Camera", view);
			cv::waitKey();

		}
		cv::waitKey();
		cv::destroyAllWindows();
	}
}