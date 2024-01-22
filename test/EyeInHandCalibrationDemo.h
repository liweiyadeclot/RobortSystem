#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>
#include "CameraManager.h"
#include "opencv2/opencv.hpp"
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

int EyeInHandCalibration()
{
	//std::vector<std::string> fileNames{ "C:\\Users\\zsh\\MVS\\Data\\501.59-205.26-724.08-159.99--52.94-40.79.jpg",
	//	"C:\\Users\\zsh\\MVS\\Data\\583.17-54.59-671.00--171.09--75.18--19.67.jpg",
	//	"C:\\Users\\zsh\\MVS\\Data\\719.01-294.18-798.64-113.76--63.60-88.46.jpg" };
	std::vector<std::vector<double>> robotPosVec{};

	//robotPosVec.push_back(std::vector<double>({ 501.59, 205.26, 724.08, 159.99, -52.94, 40.79 }));
	//robotPosVec.push_back(std::vector<double>({ 583.17, 54.59, 671.00, -171.09, -75.18, -19.67 }));
	//robotPosVec.push_back(std::vector<double>({ 719.01, 294.18, 798.64, 113.76, -63.60, 88.46 }));

	robotPosVec.push_back(std::vector<double>({ 756.21, -154.61, 815.67, 81.08, -69.36, 89.34 }));
	robotPosVec.push_back(std::vector<double>({ 668.95, -384.08, 806.34, -125.44, -89.27, -83.59 }));
	robotPosVec.push_back(std::vector<double>({ 770.24, -52.12, 818.95, 88.75, -59.50, 89.83 }));

	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 190 };
	RobotMoveSubSystem robotMovement;
	std::shared_ptr<Camera> cam = CameraManager::GetInstance()->GetOrOpenCamera();
	robotMovement.MoveToPos(756.31, -155.40, 807.96, 73.65, -69.21, 96.99);
	std::vector<cv::Mat> images{};
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

	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	std::cout << "cameraMatrix:" << cameraMatrix << std::endl;
	std::cout << "distCoeff:" << distCoeff << std::endl;

	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector< std::vector<cv::Point2f>> imagePoints{};
	/*for (std::string fileName : fileNames)
	{*/
	for (cv::Mat origin : images)
	{
		//cv::Mat origin = cv::imread(fileName);
		std::vector<cv::Point2f> corners;
		if (FindChessboradCorners(origin, BOARD_SIZE, corners))
		{
			//drawChessboardCorners(origin, BOARD_SIZE, corners, true);
		}
		//cv::Mat view;
		// //缩小，不然我电脑显示不完全
		//resize(origin, view, cv::Size(640, 480));
		//cv::imshow("result", view);
		//cv::waitKey();

		imagePoints.push_back(corners);
	}

	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	std::vector<cv::Mat> R_gripper2baseVec, t_gripper2baseVec, rvecVec, tvecVec;
	cv::Mat R_cam2gripper, t_cam2gripper;
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
		std::cout << "rvec" << i << ":" << rvec << std::endl;
		std::cout << "tvec" << i << ":" << tvec << std::endl;
		rvecVec.push_back(rvec);
		tvecVec.push_back(tvec);

		cv::Mat R_gripper2base, t_gripper2base;

		RobotPosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		R_gripper2baseVec.push_back(R_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);
	}
	cv::calibrateHandEye(R_gripper2baseVec, t_gripper2baseVec, rvecVec, tvecVec, R_cam2gripper, t_cam2gripper);

	std::cout << "R_cam2gripper:" << R_cam2gripper << std::endl;
	std::cout << "t_cam2gripper:" << t_cam2gripper << std::endl;//相机坐标单位即像素尺寸为2.5微米
	robotMovement.MoveToPos(756.31, -155.40, 807.96, 73.65, -69.21, 96.99);

	cv::Mat R_obj2cam;
	cv::Rodrigues(rvecVec[0], R_obj2cam);

	std::cout << R_obj2cam * R_cam2gripper * R_gripper2baseVec[0];

	cv::Rodrigues(rvecVec[1], R_obj2cam);
	std::cout << "==" << R_obj2cam * R_cam2gripper * R_gripper2baseVec[1] << std::endl;
	return 0;
}

//bool TestEyeInHandCalibResult(cv::Mat R_cam2gripper, cv::Mat t_cam2gripper)
//{
//	cv::Mat boardPointInCam = (cv::Mat_<double>(3, 1) << 
//		0, 400000 * 2.5, 0);
//	cv::Mat robotPos;
//}