#pragma once
#include <cstdio>
#include <cstring>
#include <string>
#include "CameraManager.h"
#include "opencv2/opencv.hpp"

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
			corners.emplace_back(j * SQUARE_SIZE, i * SQUARE_SIZE, 0);
		}
	}
	objPoints.resize(imagePoints.size(), corners);
}

int CameraCalibrationDemoMain(const std::vector<cv::Mat>& frameVec, uint8_t boardWidth, uint8_t boardHeight, double squareSize, cv::InputOutputArray cameraMatrix, cv::InputOutputArray distCoeffs)
{
	const cv::Size BOARD_SIZE(boardWidth, boardHeight);
	const double SQUARE_SIZE{ squareSize };
	cv::Size imageSize;
	CameraManager* s_CameraManager = CameraManager::GetInstance();
	std::shared_ptr<Camera> cam = s_CameraManager->GetOrOpenCamera();


	std::vector<std::vector<cv::Point2f> > imagePoints;
	// ѭ����ȡ����ͷ��׽����֡���������̸��λ
	for (cv::Mat frame : frameVec)
	{
		// ��ȡһ֡ͼ��
		//CameraFrame frame{ cam->GetFrame() };
		//cv::Mat frame{ cv::imread(imagePath) };

		imageSize = frame.size();
		//cv::Mat view;
		// ��С����Ȼ�ҵ�����ʾ����ȫ
		//cv::resize(frame, view, cv::Size(640, 480));
		//// ��ʾͼ��
		//cv::imshow("Camera", view);
		//// �ȴ��س�����������ã�����������������������q���˳�ѭ����ʼ�궨
		//int key = cv::waitKey(1);
		//if (key == 'q')
		//{
			//break;
		/*}
		else if (key != '\r')
		{
			continue;
		}*/


		std::vector<cv::Point2f> corners;
		if (true == FindChessboradCorners(frame, BOARD_SIZE, corners))
		{
			imagePoints.push_back(corners);
			//cv::drawChessboardCorners(frame, BOARD_SIZE, corners, true);
			// ��С����Ȼ�ҵ�����ʾ����ȫ
			//cv::resize(frame, view, cv::Size(640, 480));
		}
		else
		{
			//int baseLine = 0;
			//std::string msg = "Corners not found";
			//cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
			//cv::Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);
			//cv::putText(view, msg, textOrigin, 1, 1, cv::Scalar(0, 0, 255));
		}

		//// ��ʾ�ҵ��Ľǵ�
		//cv::imshow("Camera", view);

		//// �ȴ���������£�����q���˳�ѭ����ʼ�궨
		//if (cv::waitKey() == 'q')
			//break;
	}
	std::vector<std::vector<cv::Point3f>> objPoints{};
	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	// �ͷ����д���
	cv::destroyAllWindows();

	if (imagePoints.size() < 1)
	{
		return 1;
	}

	std::vector<cv::Mat> rvecs, tvecs;
	// ��ʼ�궨
	cv::calibrateCamera(objPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	//std::cout << cameraMatrix.at<double>(0, 0)//fx
	//	<< "," << cameraMatrix.at<double>(1, 1)//fy
	//	<< "," << cameraMatrix.at<double>(0, 2)//cx
	//	<< "," << cameraMatrix.at<double>(1, 2)//cy
	//	<< "," << distCoeffs.at<double>(0)//k1
	//	<< "," << distCoeffs.at<double>(1)//k2
	//	<< "," << distCoeffs.at<double>(2)//p1
	//	<< "," << distCoeffs.at<double>(3)//p2
	//	<< std::endl;
	return 0;
}