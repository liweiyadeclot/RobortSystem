#include "CustomSystemCalibrationDemo.h"

void RTToPos(const cv::Mat R, const cv::Mat t, std::vector<double>& pos)
{
	if (pos.size() < 6)
		pos.resize(6, 0);
	for (int i = 0; i < 3; i++)
	{
		pos[i] = t.at<double>(i, 0);
	}

	double rx{ 0 }, ry{ 0 }, rz{ 0 };
	ry = atan2(-R.at<double>(2, 0), sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0)));
	if (abs(2 * ry - CV_PI) < 0.01)
	{
		rx = atan2(R.at<double>(0, 1), R.at<double>(1, 1));
	}
	else if (abs(2 * ry + CV_PI) < 0.01)
	{
		rx = -atan2(R.at<double>(0, 1), R.at<double>(1, 1));
	}
	else
	{
		rx = atan2(R.at<double>(2, 1) / cos(ry), R.at<double>(2, 2) / cos(ry));
		rz = atan2(R.at<double>(1, 0) / cos(ry), R.at<double>(0, 0) / cos(ry));
	}
	pos[3] = rx * 180 / CV_PI;
	pos[4] = ry * 180 / CV_PI;
	pos[5] = rz * 180 / CV_PI;
}

void CustomSystemMove(const std::vector<double> cameraPos_custom, const cv::Mat R_custom2robot, const cv::Mat t_custom2robot, const cv::Mat R_end2camera, const cv::Mat t_end2camera,
	std::vector<double>& endPos_robot)
{
	cv::Mat R_custom2camera, t_custom2camera;
	PosToRT(cameraPos_custom, R_custom2camera, t_custom2camera);

	cv::Mat R_robot2end;
	cv::Mat R_robot2custom;
	cv::transpose(R_custom2robot, R_robot2custom);
	cv::Mat R_camera2end;
	cv::transpose(R_end2camera, R_camera2end);
	R_robot2end = R_camera2end * R_custom2camera * R_robot2custom;


	cv::Mat t_robot2end;
	cv::Mat t_camera2end;
	t_camera2end = -R_camera2end * t_end2camera;
	cv::Mat t_robot2camera;
	cv::Mat t_robot2custom;
	t_robot2custom = -R_robot2custom * t_custom2robot;
	t_robot2camera = R_robot2custom * t_custom2camera + t_robot2custom;
	cv::Mat R_robot2camera;
	R_robot2camera = R_custom2camera * R_robot2custom;
	t_robot2end = R_robot2camera * t_camera2end + t_robot2camera;

	RTToPos(R_robot2end, t_robot2end, endPos_robot);
	std::cout << "robotPos:"
		<< endPos_robot[0] << ","
		<< endPos_robot[1] << ","
		<< endPos_robot[2] << ","
		<< endPos_robot[3] << ","
		<< endPos_robot[4] << ","
		<< endPos_robot[5] << std::endl;
}

int TestCustomSystemMove(bool useRobot = false)
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
		robotPosVec.emplace_back<std::vector<double>>({ -202.13, 448.99, 673.82, 174.91, -158.68, 26.84 });
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
		robotPosVec.emplace_back<std::vector<double>>({ -49.38, 671.00, 759.28, 143.24, -177.42, 4.39 });
		robotPosVec.emplace_back<std::vector<double>>({ 146.64, 686.72, 710.61, 124.32, 169.89, -12.94 });
		robotPosVec.emplace_back<std::vector<double>>({ -267.57, 553.50, 833.55, 159.16, -160.31, 20.04 });
		robotPosVec.emplace_back<std::vector<double>>({ -202.13, 448.99, 673.82, 174.91, -158.68, 26.84 });
		std::vector<std::string> fileNames{
			".\\calib_data\\-49.380000,671.000000,759.280000,143.240000,-177.420000,4.390000,.jpg",
			".\\calib_data\\146.640000,686.720000,710.610000,124.320000,169.890000,-12.940000,.jpg",
			".\\calib_data\\-267.570000,553.500000,833.550000,159.160000,-160.310000,20.040000,.jpg",
			".\\calib_data\\-202.130000,448.990000,673.820000,174.910000,-158.680000,26.840000,.jpg"
		};
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
	int ret = CustomSystemCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
	std::vector<double> robotPos;
	for (double z = 200; z <= 500; z += 100)
	{
		CustomSystemMove(std::vector<double>({ 0,0,z,180,0,0 }), R_obj2base, t_obj2base, R_gripper2cam, t_gripper2cam, robotPos);
		if (useRobot)
		{
			RobotMoveSubSystem robotMovement;
			robotMovement.MoveToPos(robotPos[0], robotPos[1], robotPos[2], robotPos[3], robotPos[4], robotPos[5]);
			cv::Mat view;
			cv::resize(CameraManager::GetInstance()->GetOrOpenCamera()->GetFrame(), view, cv::Size(640, 480));
			cv::imshow("Camera", view);
			cv::waitKey();
		}
	}
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}