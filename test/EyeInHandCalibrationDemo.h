#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

#include "opencv2/opencv.hpp"

#include "CameraManager.h"
#include "RobotMoveSubSystem.h"
#include "CameraCalibrationDemo.h"

void RobotPosToRT(std::vector<double> pos, cv::Mat& R_base2gripper, cv::Mat& t_base2gripper)
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
	R_base2gripper = R_z * R_y * R_x;

	t_base2gripper = (cv::Mat_<double>(3, 1) <<
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
	cv::Mat& R_gripper2cam, cv::Mat& t_gripper2cam, cv::Mat& R_obj2base, cv::Mat& t_obj2base)
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
	std::vector<cv::Mat> R_base2gripperVec, t_base2gripperVec, rvec_obj2camVec, t_obj2camVec;
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

		cv::Mat R_base2gripper, t_base2gripper;
		RobotPosToRT(robotPosVec[i], R_base2gripper, t_base2gripper);
		R_base2gripperVec.push_back(R_base2gripper);
		t_base2gripperVec.push_back(t_base2gripper);
	}

	cv::Mat R_base2obj, t_base2obj;
	cv::calibrateRobotWorldHandEye(rvec_obj2camVec, t_obj2camVec, R_base2gripperVec, t_base2gripperVec, R_base2obj, t_base2obj, R_gripper2cam, t_gripper2cam, cv::CALIB_ROBOT_WORLD_HAND_EYE_SHAH);

	std::cout << "R_gripper2cam:" << R_gripper2cam << std::endl;
	std::cout << "t_gripper2cam:" << t_gripper2cam << std::endl;

	cv::Mat obj{ cv::Mat(3,1,CV_64F) };

	cv::transpose(R_base2obj, R_obj2base);
	t_obj2base = -R_obj2base * t_base2obj;

	std::cout << "R_obj2base:" << R_obj2base << std::endl;
	std::cout << "t_obj2base:" << t_obj2base << std::endl;

	cv::Mat estimateP_base{ }, estimateP_gripper{ }, estimateP_camera{ };
	for (int i = 0; i < imagePoints.size(); i++)
	{
		std::vector<cv::Point2f> reprojPoints{};
		std::vector<cv::Point3f> estimateP_cameraVec{};
		for (int j = 0; j < imagePoints[i].size(); j += 1)
		{
			obj.at<double>(0, 0) = objPoints[i][j].x;
			obj.at<double>(1, 0) = objPoints[i][j].y;
			obj.at<double>(2, 0) = objPoints[i][j].z;
			estimateP_base = R_obj2base * obj + t_obj2base;
			estimateP_gripper = R_base2gripperVec[i] * estimateP_base + t_base2gripperVec[i];
			estimateP_camera = R_gripper2cam * estimateP_gripper + t_gripper2cam;
			estimateP_cameraVec.emplace_back(estimateP_camera);
		}
		cv::projectPoints(estimateP_cameraVec, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeff, reprojPoints);
		cv::Mat view{ images[i].clone() };
		drawChessboardCorners(view, BOARD_SIZE, imagePoints[i], true);
		drawChessboardCorners(view, BOARD_SIZE, reprojPoints, true);
		cv::resize(view.clone(), view, cv::Size(640, 480));
		cv::imshow("reprojection", view);
		cv::waitKey();
	}
	cv::destroyAllWindows();
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
	const uint32_t SQUARE_SIZE{ 20 };

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
		robotPosVec.emplace_back<std::vector<double>>({ 629.11,-254.28,920.70,106.05,-82.14,52.11 });
		robotPosVec.emplace_back<std::vector<double>>({ 664.75,-136.22,920.71,93.02,-72.07,74.15 });
		robotPosVec.emplace_back<std::vector<double>>({ 676.66,49.76,921.16,101.06,-58.81,80.09 });
		robotPosVec.emplace_back<std::vector<double>>({ 547.73,49.47,905.58,122.46,-54.92,62.04 });
		robotPosVec.emplace_back<std::vector<double>>({ 806.85,-131.20,855.97,69.33,-74.51,101.29 });
		std::vector<std::string> fileNames{ "D:\\Works\\semester_7\\毕设\\RobortSystem\\test\\calib_data\\2\\629.11,-254.28,920.70,106.05,-82.14,52.11.jpg",
			"D:\\Works\\semester_7\\毕设\\RobortSystem\\test\\calib_data\\2\\664.75,-136.22,920.71,93.02,-72.07,74.15.jpg",
			"D:\\Works\\semester_7\\毕设\\RobortSystem\\test\\calib_data\\2\\676.66,49.76,921.16,101.06,-58.81,80.09.jpg",
			"D:\\Works\\semester_7\\毕设\\RobortSystem\\test\\calib_data\\2\\547.73,49.47,905.58,122.46,-54.92,62.04.jpg",
			"D:\\Works\\semester_7\\毕设\\RobortSystem\\test\\calib_data\\2\\806.85,-131.20,855.97,69.33,-74.51,101.29.jpg" };
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
	return EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
}