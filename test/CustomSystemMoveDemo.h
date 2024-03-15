#include "CalibrateAllDemo.h"

void RTToPos(const cv::Mat R_end2robot, const cv::Mat t_end2robot, std::vector<double>& pos)
{
	if (pos.size() < 6)
		pos.resize(6, 0);
	for (int i = 0; i < 3; i++)
	{
		pos[i] = t_end2robot.at<double>(i, 0);
	}

	double rx{ 0 }, ry{ 0 }, rz{ 0 };
	ry = atan2(-R_end2robot.at<double>(2, 0), sqrt(R_end2robot.at<double>(0, 0) * R_end2robot.at<double>(0, 0) + R_end2robot.at<double>(1, 0) * R_end2robot.at<double>(1, 0)));
	if (abs(2 * ry - CV_PI) < 0.01)
	{
		rx = atan2(R_end2robot.at<double>(0, 1), R_end2robot.at<double>(1, 1));
	}
	else if (abs(2 * ry + CV_PI) < 0.01)
	{
		rx = -atan2(R_end2robot.at<double>(0, 1), R_end2robot.at<double>(1, 1));
	}
	else
	{
		rx = atan2(R_end2robot.at<double>(2, 1) / cos(ry), R_end2robot.at<double>(2, 2) / cos(ry));
		rz = atan2(R_end2robot.at<double>(1, 0) / cos(ry), R_end2robot.at<double>(0, 0) / cos(ry));
	}
	pos[3] = rz * 180 / CV_PI;
	pos[5] = rx * 180 / CV_PI;
	pos[4] = ry * 180 / CV_PI;
}

void CustomSystemMove(const std::vector<double> cameraPos_custom, const cv::Mat R_custom2robot, const cv::Mat t_custom2robot, const cv::Mat R_end2camera, const cv::Mat t_end2camera,
	std::vector<double>& endPos_robot)
{
	cv::Mat R_camera2custom, t_camera2custom;
	PosToRT(cameraPos_custom, R_camera2custom, t_camera2custom);

	cv::Mat R_end2robot;
	R_end2robot = R_custom2robot * R_camera2custom * R_end2camera;


	cv::Mat t_end2robot;
	cv::Mat t_end2custom;
	t_end2custom = R_camera2custom * t_end2camera + t_camera2custom;
	t_end2robot = R_custom2robot * t_end2custom + t_custom2robot;

	RTToPos(R_end2robot, t_end2robot, endPos_robot);
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
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,50.000000,750.000000,0.000000,0.000000,180.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,300.000000,650.000000,0.000000,0.000000,150.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 650.000000,-300.000000,700.000000,0.000000,0.000000,210.000000 });
		robotPosVec.emplace_back<std::vector<double>>({ 350.000000,0.000000,700.000000,0.000000,-30.000000,180.000000 });
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
	cv::Mat cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
	int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
	
	std::vector<double> robotPos;
	for (double y = 0; y <= BOARD_SIZE.width * SQUARE_SIZE; y += SQUARE_SIZE)
	{
		CustomSystemMove(std::vector<double>({ 0,y,300,0,0,180 }), R_obj2base, t_obj2base, R_gripper2cam, t_gripper2cam, robotPos);
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