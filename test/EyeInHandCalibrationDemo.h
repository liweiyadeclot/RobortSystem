#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

#include "opencv2/opencv.hpp"

#include "CameraManager.h"
#include "RobotMoveSubSystem.h"
#include "CameraCalibrationDemo.h"

void RobotPosToRT(std::vector<double> pos, cv::Mat& R_gripper2base, cv::Mat& t_gripper2base)
{
	double rz{ pos[3] * CV_PI / 180 }, ry{ pos[4] * CV_PI / 180 }, rx{ pos[5] * CV_PI / 180 };
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
	R_gripper2base = R_x * R_y * R_z;

	t_gripper2base = (cv::Mat_<double>(3, 1) <<
		-pos[0],
		-pos[1],
		-pos[2]
		);
}

cv::Mat Rt2T(cv::Mat R, cv::Mat tvec)
{
	// 创建齐次变换矩阵
	cv::Mat transformationMat = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(transformationMat(cv::Rect(0, 0, 3, 3))); // 将旋转矩阵复制到变换矩阵的前3x3区域
	tvec.copyTo(transformationMat(cv::Rect(3, 0, 1, 3))); // 将平移向量复制到变换矩阵的第4列
	transformationMat.at<double>(3, 3) = 1.0; // 添加常量1
	return transformationMat;
}

cv::Mat RtVec2T(cv::Mat rvec, cv::Mat tvec)
{
	// 提取 R_gripper2base 的旋转矩阵和平移向量
	cv::Mat R_gripper2base;
	cv::Rodrigues(rvec, R_gripper2base);

	return Rt2T(R_gripper2base, tvec);
}


int EyeInHandCalibration(const std::vector<cv::Mat> images, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, const std::vector<std::vector<double>> robotPosVec,
	cv::Mat& R_cam2gripper, cv::Mat& t_cam2gripper)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	std::cout << "cameraMatrix:" << cameraMatrix << std::endl;
	std::cout << "distCoeff:" << distCoeff << std::endl;

	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector< std::vector<cv::Point2f>> imagePoints{};
	for (cv::Mat origin : images)
	{
		//cv::Mat origin = cv::imread(fileName);
		std::vector<cv::Point2f> corners{};
		if (FindChessboradCorners(origin, BOARD_SIZE, corners))
		{
			//drawChessboardCorners(origin, BOARD_SIZE, corners, true);
			imagePoints.push_back(corners);
		}
		//cv::Mat view;
		// //缩小，不然我电脑显示不完全
		//resize(origin, view, cv::Size(640, 480));
		//cv::imshow("result", view);
		//cv::waitKey();

	}

	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	std::vector<cv::Mat> R_gripper2baseVec, t_gripper2baseVec, rvec_obj2camVec, t_obj2camVec;
	for (int i = 0; i < objPoints.size(); i++) {
		/*for (int j = 0; j < objPoints[i].size(); j++)
		{
			std::cout << "imagePoint" << i << "," << j << ":" << imagePoints[i][j] << std::endl;
			std::cout << "objPoint" << i << "," << j << ":" << objPoints[i][j] << std::endl;
		}*/
		cv::Mat rvec, tvec;
		if (!cv::solvePnP(objPoints[i], imagePoints[i], cameraMatrix, distCoeff, rvec, tvec))
		{
			std::cout << "Falied to solvePnP" << i << std::endl;
		}
		//std::cout << "rvec" << i << ":" << rvec << std::endl;
		//std::cout << "tvec" << i << ":" << tvec << std::endl;
		rvec_obj2camVec.push_back(rvec);
		t_obj2camVec.push_back(tvec);

		cv::Mat R_gripper2base, t_gripper2base;

		RobotPosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		R_gripper2baseVec.push_back(R_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);
	}

	cv::calibrateHandEye(R_gripper2baseVec, t_gripper2baseVec, rvec_obj2camVec, t_obj2camVec, R_cam2gripper, t_cam2gripper);

	std::cout << "R_cam2gripper:" << R_cam2gripper << std::endl;
	std::cout << "t_cam2gripper:" << t_cam2gripper << std::endl;//相机坐标单位即像素尺寸为2.5微米

	cv::Mat obj{ cv::Mat(3,1,CV_64F) };
	cv::Mat R_obj2cam{};
	cv::Mat estimateP_camera{ };
	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j += 15)
		{
			obj.at<double>(0, 0) = objPoints[i][j].x;
			obj.at<double>(1, 0) = objPoints[i][j].y;
			obj.at<double>(2, 0) = objPoints[i][j].z;
			cv::Rodrigues(rvec_obj2camVec[i], R_obj2cam);
			estimateP_camera = R_obj2cam * obj + t_obj2camVec[i];
			std::cout << "imagePoints[" << i << "][" << j << "]:" << imagePoints[i][j] << ", reprojection:" << (cameraMatrix * estimateP_camera) / estimateP_camera.at<double>(2, 0) << std::endl;
		}
	}
	//cv::Mat R_obj2cam;
	//cv::Rodrigues(rvecVec[0], R_obj2cam);

	//std::cout << R_obj2cam * R_cam2gripper * R_gripper2baseVec[0];

	//cv::Rodrigues(rvecVec[1], R_obj2cam);
	//std::cout << "\n?==\n" << R_obj2cam * R_cam2gripper * R_gripper2baseVec[1] << std::endl;

	return 0;
	
}

int TestEyeInHandCalib(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 190 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 756.21, -154.61, 815.67, 81.08, -69.36, 89.34 });
		robotPosVec.emplace_back<std::vector<double>>({ 668.95, -384.08, 806.34, -125.44, -89.27, -83.59 });
		robotPosVec.emplace_back<std::vector<double>>({ 770.24, -52.12, 818.95, 88.75, -59.50, 89.83 });
		RobotMoveSubSystem robotMovement;
		std::shared_ptr<Camera> cam = CameraManager::GetInstance()->GetOrOpenCamera();
		robotMovement.MoveToPos(756.31, -155.40, 807.96, 73.65, -69.21, 96.99);
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
		robotMovement.MoveToPos(756.31, -155.40, 807.96, 73.65, -69.21, 96.99);
	}
	else
	{
		robotPosVec.emplace_back<std::vector<double>>({ 501.59, 205.26, 724.08, 159.99, -52.94, 40.79 });
		//robotPosVec.emplace_back<std::vector<double>>({ 583.17, 54.59, 671.00, -171.09, -75.18, -19.67 });
		robotPosVec.emplace_back<std::vector<double>>({ 719.01, 294.18, 798.64, 113.76, -63.60, 88.46 });
		//robotPosVec.emplace_back<std::vector<double>>({ 742.58, 225.29, 807.35, 114.94,-78.37, 81.74 });
		robotPosVec.emplace_back<std::vector<double>>({ 767.45, 120.58, 798.61, 103.99, -82.84, 84.64 });
		std::vector<std::string> fileNames{ "C:\\Users\\zsh\\MVS\\Data\\501.59-205.26-724.08-159.99--52.94-40.79.jpg",
			//"C:\\Users\\zsh\\MVS\\Data\\583.17-54.59-671.00--171.09--75.18--19.67.jpg",
			"C:\\Users\\zsh\\MVS\\Data\\719.01-294.18-798.64-113.76--63.60-88.46.jpg",
			//"C:\\Users\\zsh\\MVS\\Data\\742.58-225.29-807.35-114.94--78.37-81.74.jpg",
			"C:\\Users\\zsh\\MVS\\Data\\767.45-120.58-798.61-103.99--82.84-84.64.jpg" };
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_cam2gripper, t_cam2gripper;
	return EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_cam2gripper, t_cam2gripper);
}