#include "EyeInHandCalibrationDemo.h"

void RTToPos(const cv::Mat R, const cv::Mat t, std::vector<double>& pos)
{
	if (pos.size() < 6) pos.resize(6, 0);
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

void CustomSystemMove(RobotMoveSubSystem* robotMovement, const std::vector<double> pos, const cv::Mat R_robot2custom, const cv::Mat t_robot2custom, const cv::Mat R_camera2end, const cv::Mat t_camera2end)
{
	cv::Mat R_custom2camera, t_custom2camera;
	PosToRT(pos, R_custom2camera, t_custom2camera);

	cv::Mat R_robot2camera, t_robot2camera;
	R_robot2camera = R_custom2camera * R_robot2custom;
	t_robot2camera = R_custom2camera * t_robot2custom + t_custom2camera;

	cv::Mat R_robot2end, t_robot2end;
	R_robot2end = R_camera2end * R_robot2camera;
	t_robot2end = R_camera2end * t_robot2camera + t_camera2end;

	std::vector<double> robotPos{};
	cv::Mat R_robot2end64F, t_robot2end64F;
	R_robot2end.convertTo(R_robot2end64F, CV_64F);
	t_robot2end.convertTo(t_robot2end64F, CV_64F);
	RTToPos(R_robot2end64F, t_robot2end64F, robotPos);
	std::cout << "robotPos:"
		<< robotPos[0] << ","
		<< robotPos[1] << ","
		<< robotPos[2] << ","
		<< robotPos[3] << ","
		<< robotPos[4] << ","
		<< robotPos[5] << std::endl;
	if(robotMovement != nullptr) robotMovement->MoveToPos(robotPos[0], robotPos[1], robotPos[2], robotPos[3], robotPos[4], robotPos[5]);
}

int TestCustomSystemMove(bool useRobot = false)
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
		std::vector<std::string> fileNames{ ".\\calib_data\\2\\629.11,-254.28,920.70,106.05,-82.14,52.11.jpg",
			".\\calib_data\\2\\664.75,-136.22,920.71,93.02,-72.07,74.15.jpg",
			".\\calib_data\\2\\676.66,49.76,921.16,101.06,-58.81,80.09.jpg",
			".\\calib_data\\2\\547.73,49.47,905.58,122.46,-54.92,62.04.jpg",
			".\\calib_data\\2\\806.85,-131.20,855.97,69.33,-74.51,101.29.jpg" };
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base;
	int ret = EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_gripper2cam, t_gripper2cam, R_obj2base, t_obj2base);
	cv::Mat R_robot2custom, t_robot2custom;
	cv::transpose(R_obj2base, R_robot2custom);
	t_robot2custom = -R_robot2custom * t_obj2base;
	cv::Mat R_camera2end, t_camera2end;
	cv::transpose(R_gripper2cam, R_camera2end);
	t_camera2end = -R_camera2end * t_gripper2cam;
	RobotMoveSubSystem* pRobotMovement{ nullptr };
	if (useRobot)
	{
		pRobotMovement = new RobotMoveSubSystem();
	}
	CustomSystemMove(pRobotMovement, { 0,0,300,0,180,0 }, R_robot2custom, t_robot2custom, R_camera2end, t_camera2end);
	CustomSystemMove(pRobotMovement, { 0,20,300,0,180,0 }, R_robot2custom, t_robot2custom, R_camera2end, t_camera2end);
	CustomSystemMove(pRobotMovement, { 0,40,300,0,180,0 }, R_robot2custom, t_robot2custom, R_camera2end, t_camera2end);
	CustomSystemMove(pRobotMovement, { 0,60,300,0,180,0 }, R_robot2custom, t_robot2custom, R_camera2end, t_camera2end);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}