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
	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 20 };

	std::vector<std::vector<double>> robotPosVec{};
	std::vector<cv::Mat> images{};

	if (useRobot)
	{
		robotPosVec.emplace_back<std::vector<double>>({ 492.00, 16.15, 693.90, 46.32, 171.64, 8.66 });
		robotPosVec.emplace_back<std::vector<double>>({ 475.72, -86.14, 709.64, 24.89, 160.99, -7.40 });
		robotPosVec.emplace_back<std::vector<double>>({ 470.69, 92.61, 697.99, 60.61, -174.38, 17.21 });
		robotPosVec.emplace_back<std::vector<double>>({ 473.37, 195.01, 585.63, 80.25, -155.65, 22.65 });
		robotPosVec.emplace_back<std::vector<double>>({ 326.58, 34.31, 556.53, 57.80, 158.15, 30.94 });
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
		robotPosVec.emplace_back<std::vector<double>>({ 492.00, 16.15, 693.90, 46.32, 171.64, 8.66 });
		robotPosVec.emplace_back<std::vector<double>>({ 475.72, -86.14, 709.64, 24.89, 160.99, -7.40 });
		robotPosVec.emplace_back<std::vector<double>>({ 470.69, 92.61, 697.99, 60.61, -174.38, 17.21 });
		robotPosVec.emplace_back<std::vector<double>>({ 473.37, 195.01, 585.63, 80.25, -155.65, 22.65 });
		robotPosVec.emplace_back<std::vector<double>>({ 326.58, 34.31, 556.53, 57.80, 158.15, 30.94 });
		std::vector<std::string> fileNames{
			".\\calib_data\\492.000000,16.150000,693.900000,46.320000,171.640000,8.660000,.jpg",
			".\\calib_data\\475.720000,-86.140000,709.640000,24.890000,160.990000,-7.400000,.jpg",
			".\\calib_data\\470.690000,92.610000,697.990000,60.610000,-174.380000,17.210000,.jpg",
			".\\calib_data\\473.370000,195.010000,585.630000,80.250000,-155.650000,22.650000,.jpg",
			".\\calib_data\\326.580000,34.310000,556.530000,57.800000,158.150000,30.940000,.jpg"
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