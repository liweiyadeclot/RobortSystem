#pragma once
#include "ceres/cost_function.h"
#include "ceres/loss_function.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
#include "ceres/rotation.h"
#include "EyeInHandCalibrationDemo.h"

template <typename T1, typename T2, typename T3>
void DistortImagePoint(const T1 cameraMatrix[9], const T1 distCoeff[5], const T2 src[2], T3 dst[2])
{
	float k1{ distCoeff[0] }, k2{ distCoeff[1] }, p1{ distCoeff[2] }, p2{ distCoeff[3] }, k3{ distCoeff[4] };
	//unaccomplished
	float u{ src[0] }, v{ src[1] }, x{}, y{}, & dstU{ dst[0] }, & dstV{ dst[1] };
	float r_sqr{ x * x + y * y };
	dstU = (1 ) * x;
}

template <typename T1, typename T2, typename T3>
void Matrix3x3Mul3x1(const T1 m3x3[9], const T2 m3x1[3], T3 result[3])
{
	for (int i = 0; i < 3; i++)
	{
		result[i] = T3(m3x3[3 * i]) * T3(m3x1[0]) + T3(m3x3[3 * i + 1]) * T3(m3x1[1]) + T3(m3x3[3 * i + 2]) * T3(m3x1[2]);
	}
}

template <typename T1, typename T2>
void ArrayTypeConvert(const T1* inArray, T2* outArray, int ArrayLen)
{
	for (int i = 0; i < ArrayLen; i++)
	{
		outArray[i] = T2(inArray[i]);
	}
}

template <typename T>
void DeepCopy(const T* src, T* dst, int copyLen)
{
	for (int i = 0; i < copyLen; i++)
	{
		dst[i] = src[i];
	}
}

class ReprojectionCostFunctor
{
public:
	ReprojectionCostFunctor(cv::Mat R_robot2end, cv::Mat t_robot2end, cv::Mat R_end2camera, cv::Mat t_end2camera, cv::Mat cameraMatrix, cv::Mat distCoeff, cv::Mat P_custom, cv::Mat P_pixel, bool debug = false)
		:m_q_robot2end(),
		m_t_robot2end(),
		m_q_end2camera(),
		m_t_end2camera(),
		m_cameraMatrix(),
		m_distCoeff(),
		m_P_custom(),
		m_P_pixel(),
		m_debug(debug)
	{
		ceres::RotationMatrixToQuaternion((double*)R_robot2end.data, m_q_robot2end);
		DeepCopy((double*)t_robot2end.data, m_t_robot2end, 3);
		ceres::RotationMatrixToQuaternion((double*)R_end2camera.data, m_q_end2camera);
		DeepCopy((double*)t_end2camera.data, m_t_end2camera, 3);
		DeepCopy((double*)cameraMatrix.data, m_cameraMatrix, 9);
		DeepCopy((double*)distCoeff.data, m_distCoeff, 5);
		ArrayTypeConvert((float*)P_custom.data, m_P_custom, 3);
		ArrayTypeConvert((float*)P_pixel.data, m_P_pixel, 2);
	}

	~ReprojectionCostFunctor()
	{
	}

	template <typename T>
	bool operator()(const T const q_custom2robot[4], const T const t_custom2robot[3], T residual[1]) const
	{
		T P_custom[3]{};
		ArrayTypeConvert(m_P_custom, P_custom, 3);

		T P_robot[3]{};
		ceres::QuaternionRotatePoint(q_custom2robot, P_custom, P_robot);
		for (int i = 0; i < 3; i++) P_robot[i] += t_custom2robot[i];
		T q_robot2end[4]{};
		ArrayTypeConvert(m_q_robot2end, q_robot2end, 4);

		T P_end[3]{};
		ceres::QuaternionRotatePoint(q_robot2end, P_robot, P_end);
		for (int i = 0; i < 3; i++) P_end[i] += m_t_robot2end[i];
		T q_end2camera[4]{};
		ArrayTypeConvert(m_q_end2camera, q_end2camera, 4);

		T P_camera[3]{};
		ceres::QuaternionRotatePoint(q_end2camera, P_end, P_camera);
		for (int i = 0; i < 3; i++) P_camera[i] += m_t_end2camera[i];

		T P_pixel_esti[3]{};
		Matrix3x3Mul3x1(m_cameraMatrix, P_camera, P_pixel_esti);
		for (int i = 0; i < 3; i++) P_pixel_esti[i] /= P_camera[2];
		residual[0] = (T(m_P_pixel[0]) - P_pixel_esti[0]) * (T(m_P_pixel[0]) - P_pixel_esti[0])
			+ (T(m_P_pixel[1]) - P_pixel_esti[1]) * (T(m_P_pixel[1]) - P_pixel_esti[1]);

		if (m_debug)
		{
			std::cout << "m_P_pixel:" << cv::Mat(2, 1, CV_64F, (double*)m_P_pixel) << ", P_pixel_esti:" << cv::Mat(3, 1, CV_64F, (double*)P_pixel_esti) << std::endl;
		}

		return true;
	}
private:
	double m_q_robot2end[4];
	double m_t_robot2end[3];
	double m_q_end2camera[4];
	double m_t_end2camera[3];
	double m_cameraMatrix[9];
	double m_distCoeff[5];
	double m_P_custom[3];
	double m_P_pixel[2];
	bool m_debug;
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
			std::vector<cv::Point2f> undistortCorners{};
			cv::undistortImagePoints(corners, undistortCorners, cameraMatrix, distCoeff);
			imagePoints.push_back(undistortCorners);
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
	double q_custom2robot[4] = { 1,0,0,0 };
	double t_custom2robot[3] = { 0,0,0 };
	// Build the problem.
	ceres::Problem problem;

	cv::Mat R_end2camera{};
	cv::transpose(R_cam2gripper, R_end2camera);
	cv::Mat t_end2camera{ -R_end2camera * t_cam2gripper };
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j++)
		{
			cv::Mat R_robot2end{};
			cv::transpose(R_gripper2baseVec[i], R_robot2end);
			cv::Mat t_robot2end{ -(R_robot2end * t_gripper2baseVec[i]) };
			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 1, 4, 3>(new ReprojectionCostFunctor(R_robot2end, t_robot2end, R_end2camera, t_end2camera, cameraMatrix, distCoeff, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]))),
				new ceres::CauchyLoss(1.0),
				//nullptr,
				q_custom2robot, t_custom2robot);
		}
	}


	// Run the solver!
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 200;
	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);

	double R_custom2robot[9] = { 1,0,0,0,1,0,0,0,1 };
	ceres::QuaternionToRotation(q_custom2robot, R_custom2robot);
	std::cout << summary.BriefReport() << std::endl
		<< "R_custom2robot:" << cv::Mat(3, 3, CV_64F, R_custom2robot) << std::endl
		<< "t_custom2robot:" << cv::Mat(3, 1, CV_64F, t_custom2robot) << std::endl;
	cv::Mat obj{ cv::Mat(3,1,CV_64F) };
	cv::Mat estimateP_robot{};
	cv::Mat R_robot2end{};
	cv::Mat t_robot2end{};
	cv::Mat estimateP_end{};
	cv::Mat estimateP_camera{};

	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j += 15)
		{
			obj.at<double>(0, 0) = objPoints[i][j].x;
			obj.at<double>(1, 0) = objPoints[i][j].y;
			obj.at<double>(2, 0) = objPoints[i][j].z;
			estimateP_robot = cv::Mat(3, 3, CV_64F, R_custom2robot) * obj + cv::Mat(3, 1, CV_64F, t_custom2robot);
			cv::transpose(R_gripper2baseVec[i], R_robot2end);
			cv::Mat t_robot2end{ -(R_robot2end * t_gripper2baseVec[i]) };
			estimateP_end = R_robot2end * estimateP_robot + t_robot2end;
			estimateP_camera = R_end2camera * estimateP_end + t_end2camera;
			//std::cout << "imagePoints[" << i << "][" << j << "]:" << imagePoints[i][j] << ", reprojection:" << (cameraMatrix * estimateP_camera) / estimateP_camera.at<double>(2, 0) << std::endl;
			double reprojectionError{ 0 };
			ReprojectionCostFunctor functor{ ReprojectionCostFunctor(R_robot2end, t_robot2end, R_end2camera, t_end2camera, cameraMatrix, distCoeff, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]), true) };
			functor(q_custom2robot, t_custom2robot, &reprojectionError);
			std::cout << "reprojectionError:" << reprojectionError << std::endl;
		}
	}
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

	cv::Mat obj{ cv::Mat(3,1,CV_64F) };
	cv::Mat estimateP_robot{};
	cv::Mat R_robot2end{};
	cv::Mat t_robot2end{};
	cv::Mat estimateP_end{};
	cv::Mat estimateP_camera{};

	//for (int i = 0; i < imagePoints.size(); i++)
	//{
	//	for (int j = 0; j < imagePoints[i].size(); j += 15)
	//	{
	//		obj.at<double>(0, 0) = objPoints[i][j].x;
	//		obj.at<double>(1, 0) = objPoints[i][j].y;
	//		obj.at<double>(2, 0) = objPoints[i][j].z;
	//		estimateP_robot = cv::Mat(3, 3, CV_64F, R_custom2robot) * obj + cv::Mat(3, 1, CV_64F, t_custom2robot);
	//		cv::transpose(R_gripper2baseVec[i], R_robot2end);
	//		cv::Mat t_robot2end{ -(R_robot2end * t_gripper2baseVec[i]) };
	//		estimateP_end = R_robot2end * estimateP_robot + t_robot2end;
	//		estimateP_camera = R_end2camera * estimateP_end + t_end2camera;
	//		//std::cout << "imagePoints[" << i << "][" << j << "]:" << imagePoints[i][j] << ", reprojection:" << (cameraMatrix * estimateP_camera) / estimateP_camera.at<double>(2, 0) << std::endl;
	//		double reprojectionError{ 0 };
	//		ReprojectionCostFunctor functor{ ReprojectionCostFunctor(R_robot2end, t_robot2end, R_end2camera, t_end2camera, cameraMatrix, distCoeff, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]), true) };
	//		functor(q_custom2robot, t_custom2robot, &reprojectionError);
	//		std::cout << "reprojectionError:" << reprojectionError << std::endl;
	//	}
	//}
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