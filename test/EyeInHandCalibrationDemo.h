#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

#include "opencv2/opencv.hpp"

#include "CameraManager.h"
#include "RobotMoveSubSystem.h"
#include "CameraCalibrationDemo.h"

void PosToRT(const std::vector<double> pos, cv::Mat& R, cv::Mat& t)
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
	R = R_z * R_y * R_x;

	t = (cv::Mat_<double>(3, 1) <<
		pos[0],
		pos[1],
		pos[2]
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
		cv::Mat rvec, tvec;
		if (!cv::solvePnP(objPoints[i], imagePoints[i], cameraMatrix, distCoeff, rvec, tvec))
		{
			std::cout << "Falied to solvePnP" << i << std::endl;
		}
		rvec_obj2camVec.push_back(rvec);
		t_obj2camVec.push_back(tvec);

		cv::Mat R_base2gripper, t_base2gripper;
		PosToRT(robotPosVec[i], R_base2gripper, t_base2gripper);
		cv::Mat R_gripper2base, t_gripper2base;
		cv::transpose(R_base2gripper,R_gripper2base);
		t_gripper2base = -R_gripper2base * t_base2gripper;
		R_gripper2baseVec.push_back(R_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);
	}

	cv::calibrateHandEye(R_gripper2baseVec, t_gripper2baseVec, rvec_obj2camVec, t_obj2camVec, R_cam2gripper, t_cam2gripper);// , cv::CALIB_HAND_EYE_PARK);


	std::cout << "R_cam2gripper:" << R_cam2gripper << std::endl;
	std::cout << "t_cam2gripper:" << t_cam2gripper << std::endl;

	return 0;
}

int TestEyeInHandCalib(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 20 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 492.00, 16.15, 693.90, 46.32, 171.64, 8.66 });
		robotPosVec.emplace_back<std::vector<double>>({ 475.72, -86.14, 709.64, 24.89, 160.99, -7.40 });
		robotPosVec.emplace_back<std::vector<double>>({ 470.69, 92.61, 697.99, 60.61, -174.38, 17.21 });
		
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
	}
	else
	{
		robotPosVec.emplace_back<std::vector<double>>({ 492.00, 16.15, 693.90, 46.32, 171.64, 8.66 });
		robotPosVec.emplace_back<std::vector<double>>({ 475.72, -86.14, 709.64, 24.89, 160.99, -7.40 });
		robotPosVec.emplace_back<std::vector<double>>({ 470.69, 92.61, 697.99, 60.61, -174.38, 17.21 });
		std::vector<std::string> fileNames{
			".\\calib_data\\3\\492.000000,16.150000,693.900000,46.320000,171.640000,8.660000,.jpg",
			".\\calib_data\\3\\475.720000,-86.140000,709.640000,24.890000,160.990000,-7.400000,.jpg",
			".\\calib_data\\3\\470.690000,92.610000,697.990000,60.610000,-174.380000,17.210000,.jpg"
		};
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_gripper2cam, t_gripper2cam;
	int ret = EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_gripper2cam, t_gripper2cam);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}