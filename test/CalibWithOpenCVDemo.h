#pragma once
#include <vector>

#include "opencv2/opencv.hpp"

#include "RobotMoveSubSystem.h"
#include "CameraManager.h"

bool FindChessboradCorners(const cv::InputArray image, const cv::Size patternSize, cv::InputOutputArray outCorners,
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

void genChessBoardObjectPoints(const cv::Size BOARD_SIZE, const uint32_t SQUARE_SIZE, const std::vector<std::vector<cv::Point2f> >& imagePoints, std::vector<std::vector<cv::Point3f>>& objPoints)
{
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < BOARD_SIZE.height; ++i) {
		for (int j = 0; j < BOARD_SIZE.width; ++j) {
			corners.emplace_back(i * SQUARE_SIZE, j * SQUARE_SIZE, 0);
		}
	}
	objPoints.resize(imagePoints.size(), corners);
}

void PosToRT(const std::vector<double> pos, cv::Mat& R_end2robot, cv::Mat& t_end2robot)
{
	double rz{ pos[3] * CV_PI / 180.0f }, ry{ pos[4] * CV_PI / 180.0f }, rx{ pos[5] * CV_PI / 180.0f };
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

	t_end2robot = (cv::Mat_<double>(3, 1) <<
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
	return transformationMat;
}

cv::Mat RtVec2T(cv::Mat rvec, cv::Mat tvec)
{
	// 提取 R_gripper2base 的旋转矩阵和平移向量
	cv::Mat R_gripper2base;
	cv::Rodrigues(rvec, R_gripper2base);

	return Rt2T(R_gripper2base, tvec);
}

int CalibrateAll(std::vector<cv::Mat> inImageVec, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, std::vector<std::vector<double>> inRobotPosVec,
	cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& R_camera2end, cv::Mat& t_camera2end, cv::Mat& R_custom2robot, cv::Mat& t_custom2robot)
{

	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<cv::Mat> imageVec;
	std::vector<std::vector<double>> robotPosVec;
	for (int i = 0; i < inImageVec.size(); i++)
	{
		std::vector<cv::Point2f> corners;
		if (true == FindChessboradCorners(inImageVec[i], BOARD_SIZE, corners))
		{
			imagePoints.push_back(corners);
			imageVec.push_back(inImageVec[i]);
			robotPosVec.push_back(inRobotPosVec[i]);
		}
	}
	std::vector<std::vector<cv::Point3f>> objPoints{};
	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	// start camera calibration
	std::vector<cv::Mat> rvec_custom2cameraVecs, tvec_custom2cameraVecs;
	cv::calibrateCamera(objPoints, imagePoints, imageVec[0].size(), cameraMatrix, distCoeffs, rvec_custom2cameraVecs, tvec_custom2cameraVecs);
	// end of camera calibration
	//start eye-in-hand calibration
	//计算和收集所需数据
	std::vector<cv::Mat> Hg, Hc, R_robot2endVec, t_robot2endVec, R_base2gripperVec, t_base2gripperVec;
	for (int i = 0; i < imagePoints.size(); i++) {
		cv::Mat rvec{ rvec_custom2cameraVecs[i] }, tvec{ tvec_custom2cameraVecs[i] };
		Hc.push_back(RtVec2T(rvec, tvec));

		cv::Mat R_gripper2base, t_gripper2base;
		PosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		Hg.push_back(Rt2T(R_gripper2base, t_gripper2base));
		cv::Mat t_base2gripper{-R_gripper2base.t() * t_gripper2base};

		R_base2gripperVec.push_back(R_gripper2base.t());
		t_base2gripperVec.push_back(t_base2gripper);

		cv::Mat R_robot2end;
		cv::transpose(R_gripper2base, R_robot2end);
		R_robot2endVec.push_back(R_robot2end);
		t_robot2endVec.push_back(-R_robot2end * t_gripper2base);
	}
	cv::Mat R_robot2custom, t_robot2custom, R_end2camera, t_end2camera;
	cv::calibrateRobotWorldHandEye(rvec_custom2cameraVecs, tvec_custom2cameraVecs, R_base2gripperVec, t_base2gripperVec, R_robot2custom, t_robot2custom, R_end2camera, t_end2camera);
	R_camera2end = R_end2camera.t();
	t_camera2end = -R_camera2end * t_end2camera;
	R_custom2robot = R_robot2custom.t();
	t_custom2robot = -R_custom2robot * t_robot2custom;
	
	std::cout
		<< "R_camera2end:" << R_camera2end << std::endl
		<< "t_camera2end:" << t_camera2end << std::endl
		<< "R_custom2robot:" << R_custom2robot << std::endl
		<< "t_custom2robot:" << t_custom2robot << std::endl;
	// end of custom system calibration
	// start reprojection validation
	//cv::Mat obj{ cv::Mat(3,1,CV_64F) };
	//for (int i = 0; i < imagePoints.size(); i++)
	//{
	//	std::vector<cv::Point2f> reprojPoints{};
	//	std::vector<cv::Point3f> estimateP_cameraVec{};
	//	for (int j = 0; j < imagePoints[i].size(); j += 1)
	//	{
	//		cv::Mat estimateP_base{ }, estimateP_gripper{ }, estimateP_camera{ };
	//		obj.at<double>(0, 0) = objPoints[i][j].x;
	//		obj.at<double>(1, 0) = objPoints[i][j].y;
	//		obj.at<double>(2, 0) = objPoints[i][j].z;
	//		estimateP_base = R_custom2robot * obj + t_custom2robot;
	//		estimateP_gripper = R_robot2endVec[i] * estimateP_base + t_robot2endVec[i];
	//		estimateP_camera = R_end2camera * estimateP_gripper + t_end2camera;
	//		estimateP_cameraVec.emplace_back(estimateP_camera);
	//		/*cv::Mat estimateHomoP_image = cameraMatrix * estimateP_camera / estimateP_camera.at<double>(2, 0);
	//		reprojPoints.emplace_back(estimateHomoP_image.at<double>(0, 0), estimateHomoP_image.at<double>(1, 0));*/
	//	}
	//	cv::projectPoints(estimateP_cameraVec, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, reprojPoints);
	//	cv::Mat view;
	//	cv::undistort(imageVec[i].clone(), view, cameraMatrix, distCoeffs);
	//	drawChessboardCorners(view, BOARD_SIZE, reprojPoints, true);
	//	cv::resize(view.clone(), view, cv::Size(640, 480));
	//	std::string windowName{ std::string("reprojection0") };
	//	windowName[windowName.length() - 1] += i;
	//	cv::imshow(windowName, view);
	//}
	return 0;
}

int TestCalibrateAllDemo(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 6);
	const uint32_t SQUARE_SIZE{ 30 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ -49.38, 671.00, 759.28, 143.24, -177.42, 4.39 });
		robotPosVec.emplace_back<std::vector<double>>({ 146.64, 686.72, 710.61, 124.32, 169.89, -12.94 });
		robotPosVec.emplace_back<std::vector<double>>({ -267.57, 553.50, 833.55, 159.16, -160.31, 20.04 });
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
			std::string imageFileName{ ".\\calib_data\\" };
			for (double value : pos)
			{
				imageFileName.append(std::to_string(value));
				imageFileName.append(",");
			}
			imageFileName.append(".jpg");
			cv::imwrite(imageFileName, cam->GetFrame());
			images.push_back(cam->GetFrame());
		}
		cv::destroyAllWindows();
	}
	else
	{
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,50.000000,750.000000,0.000000,0.000000,180.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,300.000000,650.000000,0.000000,0.000000,150.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,-300.000000,700.000000,0.000000,0.000000,210.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 350.000000,0.000000,700.000000,0.000000,-30.000000,180.000000 });
		std::vector<std::string> fileNames{
			".\\calib_data\\650.000000,50.000000,750.000000,0.000000,0.000000,180.000000,.jpg",
			".\\calib_data\\650.000000,300.000000,650.000000,0.000000,0.000000,150.000000,.jpg",
			".\\calib_data\\650.000000,-300.000000,700.000000,0.000000,0.000000,210.000000,.jpg",
			".\\calib_data\\350.000000,0.000000,700.000000,0.000000,-30.000000,180.000000,.jpg"
		};
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat cameraMatrix, distCoeffs, R_camera2end, t_camera2end, R_custom2camera, t_custom2camera;
	int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_camera2end, t_camera2end, R_custom2camera, t_custom2camera);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}
