#include <cstdio>
#include <cstring>
#include <string>
#include "CameraManager.h"
#include "opencv2/opencv.hpp"
#include "RobotMoveSubSystem.h"
#include "CameraCalibrationDemo.h"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	Mat cameraMatrix;
	Mat distCoeff;
	cout << "**************************Camera Calibration Start" << endl;
	CameraCalibrationDemoMain("C:\\Users\\zsh\\MVS\\Data\\501.59-205.26-724.08-159.99--52.94-40.79.jpg", 9, 7, 20, cameraMatrix, distCoeff);
	cout << "**************************Eye In Hand Calibration Start" << endl;
	return EyeInHandCalibration(cameraMatrix, distCoeff);
}


int EyeInHandCalibration(Mat cameraMatrix, Mat distCoeff)
{
	//RobotMoveSubSystem robotMovement;
	//robotMovement.MoveToPos(742.58, 225.29, 807.35, 114.94, -78.37, 81.74);
	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 20 };
	Mat origin = imread("C:\\Users\\zsh\\MVS\\Data\\501.59-205.26-724.08-159.99--52.94-40.79.jpg");
	vector<Point2f> corners;
	if (FindChessboradCorners(origin, BOARD_SIZE, corners))
	{
		drawChessboardCorners(origin, BOARD_SIZE, corners, true);
	}
	Mat view;
	// 缩小，不然我电脑显示不完全
	resize(origin, view, cv::Size(640, 480));
	imshow("result", view);
	waitKey();



	return 0;
}