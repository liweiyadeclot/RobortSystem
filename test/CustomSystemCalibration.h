#pragma once
#include "ceres/cost_function.h"
#include "ceres/loss_function.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
#include "EyeInHandCalibrationDemo.h"

template <typename T>
std::vector<T> RotationMatrixRotate(const double* R, const T* P)
{
	std::vector<T> ret{ T(0),T(0),T(0) };
	for (int i = 0; i < 3; i++)
	{
		ret[i] = R[3 * i] * P[0] + R[3 * i + 1] * P[1] + R[3 * i + 2] * P[2];
	}
	return ret;
}

template <typename T>
std::vector<T> RotationMatrixRotate1(const T* R, const double* P)
{
	std::vector<T> ret{ T(0),T(0),T(0) };
	for (int i = 0; i < 3; i++)
	{
		ret[i] = R[3 * i] * P[0] + R[3 * i + 1] * P[1] + R[3 * i + 2] * P[2];
	}
	return ret;
}

struct LeastSquareCostFunctor
{
public:
	LeastSquareCostFunctor(cv::Mat R_robot2end, cv::Mat t_robot2end, cv::Mat R_end2camera, cv::Mat t_end2camera, cv::Mat cameraMatrix, cv::Mat P_custom, cv::Mat P_pixel)
		:m_R_robot2end((double*)R_robot2end.data),
		m_t_robot2end((double*)t_robot2end.data),
		m_R_end2camera((double*)R_end2camera.data),
		m_t_end2camera((double*)t_end2camera.data),
		m_cameraMatrix((double*)cameraMatrix.data),
		m_P_custom((double*)P_custom.data),
		m_P_pixel((double*)P_pixel.data)
	{
	}

	template <typename T>
	bool operator()(const T* const R_custom2robot, const T* const t_custom2robot, T* residual) const
	{
		std::vector<T> P_robot{ RotationMatrixRotate1(R_custom2robot, m_P_custom) };
		for (int i = 0; i < 3; i++) P_robot[i] += t_custom2robot[i];
		std::vector<T> P_end{ RotationMatrixRotate(m_R_robot2end, P_robot.data()) };
		for (int i = 0; i < 3; i++) P_end[i] += m_t_robot2end[i];
		std::vector<T> P_camera{ RotationMatrixRotate(m_R_end2camera, P_end.data()) };
		for (int i = 0; i < 3; i++) P_camera[i] += m_t_end2camera[i];
		std::vector<T> P_pixel_esti{ RotationMatrixRotate(m_cameraMatrix, P_camera.data()) };
		for (int i = 0; i < 3; i++) P_pixel_esti[i] /= P_camera[2];

		residual[0] = T(m_P_pixel[0]) - P_pixel_esti[0];
		residual[1] = T(m_P_pixel[1]) - P_pixel_esti[1];
		return true;
	}
private:
	double* m_R_robot2end;
	double* m_t_robot2end;
	double* m_R_end2camera;
	double* m_t_end2camera;
	double* m_cameraMatrix;
	double* m_P_custom;
	double* m_P_pixel;
};

int CustomSystemCalibration(const std::vector<cv::Mat> images, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, const std::vector<std::vector<double>> robotPosVec)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	cv::Mat R_cam2gripper, t_cam2gripper;

	EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_cam2gripper, t_cam2gripper);

	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector< std::vector<cv::Point2f>> imagePoints{};
	for (cv::Mat origin : images)
	{
		std::vector<cv::Point2f> corners{};
		if (FindChessboradCorners(origin, BOARD_SIZE, corners))
		{
			imagePoints.push_back(corners);
		}

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

		cv::Mat R_gripper2base, t_gripper2base;

		RobotPosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		R_gripper2baseVec.push_back(R_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);
	}
	std::vector<std::vector<std::pair<cv::Point3f, cv::Point2f>>> pointPairs_custom2pixel{};
	for (int i = 0; i < imagePoints.size(); i++)
	{
		std::vector<std::pair<cv::Point3f, cv::Point2f>> pairs{};
		for (int j = 0; j < imagePoints[i].size(); j++)
		{
			pairs.push_back(std::make_pair(objPoints[i][j], imagePoints[i][j]));
		}
		pointPairs_custom2pixel.push_back(pairs);
	}
	double R_custom2robot[9] = { 1,0,0,0,1,0,0,0,1 };
	double t_custom2robot[3] = { 0,0,0 };
	// Build the problem.
	ceres::Problem problem;

	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j++)
		{

			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<LeastSquareCostFunctor, 2, 9, 3>(new LeastSquareCostFunctor(R_gripper2baseVec[i], t_gripper2baseVec[i], R_cam2gripper, t_cam2gripper, cameraMatrix, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]))),
				new ceres::CauchyLoss(1),
				R_custom2robot, t_custom2robot);
		}
	}


	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);

	std::cout << summary.BriefReport() << std::endl
		<< "R_custom2robot:" << cv::Mat(3, 3, CV_64F, R_custom2robot) << std::endl
		<< "t_custom2robot:" << cv::Mat(3, 1, CV_64F, t_custom2robot) << std::endl;
	return 0;
}

int CustomSystemCalibration1(const std::vector<cv::Mat> images, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, const std::vector<std::vector<double>> robotPosVec)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	cv::Mat R_cam2gripper, t_cam2gripper;

	EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_cam2gripper, t_cam2gripper);

	cv::Mat T_cam2gripper = Rt2T(R_cam2gripper, t_cam2gripper);

	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector< std::vector<cv::Point2f>> imagePoints{};
	for (cv::Mat origin : images)
	{
		std::vector<cv::Point2f> corners{};
		if (FindChessboradCorners(origin, BOARD_SIZE, corners))
		{
			imagePoints.push_back(corners);
		}
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

		cv::Mat R_gripper2base, t_gripper2base;

		RobotPosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		R_gripper2baseVec.push_back(R_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);
	}
	std::vector<cv::Mat> AVec; // 线性方程组的A
	for (int i = 0; i < rvec_obj2camVec.size(); i++)
	{
		AVec.push_back((T_cam2gripper * RtVec2T(rvec_obj2camVec[i], t_obj2camVec[i])).inv());
	}

	std::vector<cv::Mat> bVec; // 线性方程组的b
	for (int i = 0; i < R_gripper2baseVec.size(); i++)
	{
		bVec.push_back(Rt2T(R_gripper2baseVec[i], t_gripper2baseVec[i]));
	}

	// 使用 cv::vconcat 将齐次变换矩阵拼接成增广矩阵
	cv::Mat A;
	cv::vconcat(AVec, A);
	cv::Mat b;
	cv::vconcat(bVec, b);
	cv::Mat X;
	cv::solve(AVec[1], bVec[1], X, cv::DECOMP_NORMAL);

	std::cout << "T1:" << X << std::endl;

	cv::solve(A, b, X, cv::DECOMP_NORMAL);
	std::cout << "T:" << X << std::endl;

	AVec.clear();
	bVec.clear();

	for (int i = 0; i < rvec_obj2camVec.size(); i++)
	{
		cv::Mat R_obj2cam;
		cv::Rodrigues(rvec_obj2camVec[i], R_obj2cam);
		AVec.push_back((R_obj2cam * R_cam2gripper).inv());
	}

	for (int i = 0; i < R_gripper2baseVec.size(); i++)
	{
		bVec.push_back(R_gripper2baseVec[i]);
	}
	cv::solve(AVec[1], bVec[1], X, cv::DECOMP_NORMAL);

	std::cout << "R1:" << X << std::endl;

	cv::vconcat(AVec, A);
	cv::vconcat(bVec, b);
	cv::solve(A, b, X, cv::DECOMP_NORMAL);
	std::cout << "R:" << X << std::endl;

	cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
	for (int i = 0; i < t_obj2camVec.size(); i++)
	{
		t = t + t_obj2camVec[i] + t_cam2gripper + t_gripper2baseVec[i];
	}
	t = t / t_obj2camVec.size();
	std::cout << "t:" << t << std::endl;
	return 0;
}

int TestCustomSystemCalib(bool useRobot = false)
{
	const cv::Size BOARD_SIZE(9, 7);
	const uint32_t SQUARE_SIZE{ 190 };

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
		robotPosVec.emplace_back<std::vector<double>>({ 501.59, 205.26, 724.08, 159.99, -52.94, 40.79 });
		//robotPosVec.emplace_back<std::vector<double>>({ 583.17, 54.59, 671.00, -171.09, -75.18, -19.67 });
		robotPosVec.emplace_back<std::vector<double>>({ 719.01, 294.18, 798.64, 113.76, -63.60, 88.46 });
		//robotPosVec.emplace_back<std::vector<double>>({ 742.58, 225.29, 807.35, 114.94,-78.37, 81.74 });
		robotPosVec.emplace_back<std::vector<double>>({ 767.45, 120.58, 798.61, 103.99, -82.84, 84.64 });
		std::vector<std::string> fileNames{ "C:\\Users\\zsh\\MVS\\Data\\501.59-205.26-724.08-159.99--52.94-40.79.jpg",
			//"C:\\Users\\zsh\\MVS\\Data\\583.17-54.59-671.00--171.09--75.18--19.67.jpg",
			"C:\\Users\\zsh\\MVS\\Data\\719.01-294.18-798.64-113.76--63.60-88.46.jpg",
			//"C:\\Users\\zsh\\MVS\\Data\\742.58-225.29-807.35-114.94--78.37-81.74.jpg",
			"C:\\Users\\zsh\\MVS\\Data\\767.45-120.58-798.61-103.99--82.84-84.64.jpg" };
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_cam2gripper, t_cam2gripper;
	return CustomSystemCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec);
}