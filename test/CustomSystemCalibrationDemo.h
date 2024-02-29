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
	dstU = (1) * x;
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

void CVRotationMatrixToQuaternion(const cv::Mat R, double q[4])
{
	cv::Mat R_colMajor;
	cv::transpose(R, R_colMajor);
	ceres::RotationMatrixToQuaternion((double*)R_colMajor.data, q);
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

		CVRotationMatrixToQuaternion(R_robot2end, m_q_robot2end);
		DeepCopy((double*)t_robot2end.data, m_t_robot2end, 3);
		CVRotationMatrixToQuaternion(R_end2camera, m_q_end2camera);
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
	static bool projection(
		const T P_custom[3],
		const T q_custom2robot[4],
		const T t_custom2robot[3],
		const T q_robot2end[4],
		const T t_robot2end[3],
		const T q_end2camera[4],
		const T t_end2camera[3],
		const T cameraMatrix[9],
		T P_pixel_esti[3],
		bool debug = false)
	{
		T P_robot[3]{};
		ceres::QuaternionRotatePoint(q_custom2robot, P_custom, P_robot);
		for (int i = 0; i < 3; i++) P_robot[i] += t_custom2robot[i];

		T P_end[3]{};
		ceres::QuaternionRotatePoint(q_robot2end, P_robot, P_end);
		for (int i = 0; i < 3; i++) P_end[i] += t_robot2end[i];

		T P_camera[3]{};
		ceres::QuaternionRotatePoint(q_end2camera, P_end, P_camera);
		for (int i = 0; i < 3; i++) P_camera[i] += t_end2camera[i];

		Matrix3x3Mul3x1(cameraMatrix, P_camera, P_pixel_esti);
		for (int i = 0; i < 3; i++) P_pixel_esti[i] /= P_camera[2];
		if (debug)std::cout << "t_custom2robot:" << cv::Mat(3, 1, CV_64F, (void*)t_custom2robot) << ",t_robot2end:" << cv::Mat(3, 1, CV_64F, (void*)t_robot2end) << ",t_end2camera:" << cv::Mat(3, 1, CV_64F, (void*)t_end2camera);
		if (debug)std::cout << "P_custom:" << cv::Mat(3, 1, CV_64F, (void*)P_custom) << ", P_robot:" << cv::Mat(3, 1, CV_64F, (void*)P_robot) << ", P_end:" << cv::Mat(3, 1, CV_64F, (void*)P_end) << ", P_camera:" << cv::Mat(3, 1, CV_64F, (void*)P_camera) << std::endl;
		return true;
	}

	template <typename T>
	bool operator()(const T const q_custom2robot[4], const T const t_custom2robot[3], T residual[1]) const
	{
		T P_custom[3]{};
		ArrayTypeConvert(m_P_custom, P_custom, 3);
		T q_robot2end[4]{};
		ArrayTypeConvert(m_q_robot2end, q_robot2end, 4);
		T t_robot2end[3]{};
		ArrayTypeConvert(m_t_robot2end, t_robot2end, 3);
		T q_end2camera[4]{};
		ArrayTypeConvert(m_q_end2camera, q_end2camera, 4);
		T t_end2camera[3]{};
		ArrayTypeConvert(m_t_end2camera, t_end2camera, 3);
		T cameraMatrix[9]{};
		ArrayTypeConvert(m_cameraMatrix, cameraMatrix, 9);

		T P_pixel_esti[3]{};
		projection(P_custom, q_custom2robot, t_custom2robot, q_robot2end, t_robot2end, q_end2camera, t_end2camera, cameraMatrix, P_pixel_esti);
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

int CustomSystemCalibration(const std::vector<cv::Mat> images, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, const std::vector<std::vector<double>> robotPosVec,
	cv::Mat& R_camera2end, cv::Mat& t_camera2end, cv::Mat& R_custom2robot, cv::Mat& t_custom2robot)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_camera2end, t_camera2end);

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
	std::vector<cv::Mat> R_base2gripperVec, t_base2gripperVec, rvec_obj2camVec, t_obj2camVec;
	for (int i = 0; i < objPoints.size(); i++) {
		cv::Mat rvec, tvec;
		if (!cv::solvePnP(objPoints[i], imagePoints[i], cameraMatrix, distCoeff, rvec, tvec))
		{
			std::cout << "Falied to solvePnP" << i << std::endl;
		}
		rvec_obj2camVec.push_back(rvec);
		t_obj2camVec.push_back(tvec);

		cv::Mat R_base2gripper, t_base2gripper;
		PosToRT(robotPosVec[i], R_base2gripper, t_base2gripper);
		R_base2gripperVec.push_back(R_base2gripper);
		t_base2gripperVec.push_back(t_base2gripper);
	}
	double q_custom2robotArray[4] = { 1,0,0,0 };
	double t_custom2robotArray[3] = { 0,0,0 };
	cv::Mat obj{ cv::Mat(3,1,CV_64F) };
	// Build the problem.
	ceres::Problem problem;

	cv::Mat R_end2camera;
	cv::transpose(R_camera2end, R_end2camera);
	cv::Mat t_end2camera{ -R_end2camera * t_camera2end };
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j++)
		{
			cv::Mat R_robot2end{ R_base2gripperVec[i] };
			cv::Mat t_robot2end{ t_base2gripperVec[i] };
			problem.AddResidualBlock(
				new ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 1, 4, 3>(new ReprojectionCostFunctor(R_robot2end, t_robot2end, R_end2camera, t_end2camera, cameraMatrix, distCoeff, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]))),
				new ceres::HuberLoss(1.0),
				//nullptr,
				q_custom2robotArray, t_custom2robotArray);
		}
	}


	// Run the solver!
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);

	double R_custom2robotArray[9] = { 1,0,0,0,1,0,0,0,1 };
	ceres::QuaternionToRotation(q_custom2robotArray, R_custom2robotArray);
	R_custom2robot = cv::Mat(3, 3, CV_64F, R_custom2robotArray);
	t_custom2robot = cv::Mat(3, 1, CV_64F, t_custom2robotArray);
	std::cout << summary.BriefReport() << std::endl
		<< "R_custom2robot:" << R_custom2robot << std::endl
		<< "t_custom2robot:" << t_custom2robot << std::endl;

	for (int i = 0; i < imagePoints.size(); i++)
	{
		std::vector<cv::Point2f> reprojPoints{};
		std::vector<cv::Point3f> estimateP_cameraVec{};
		for (int j = 0; j < imagePoints[i].size(); j += 1)
		{
			cv::Mat estimateP_base{ }, estimateP_gripper{ }, estimateP_camera{ };
			obj.at<double>(0, 0) = objPoints[i][j].x;
			obj.at<double>(1, 0) = objPoints[i][j].y;
			obj.at<double>(2, 0) = objPoints[i][j].z;
			estimateP_base = R_custom2robot * obj + t_custom2robot;
			estimateP_gripper = R_base2gripperVec[i] * estimateP_base + t_base2gripperVec[i];
			estimateP_camera = R_end2camera * estimateP_gripper + t_end2camera;
			estimateP_cameraVec.emplace_back(estimateP_camera);
		}
		cv::projectPoints(estimateP_cameraVec, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeff, reprojPoints);
		cv::Mat view{ images[i].clone() };
		drawChessboardCorners(view, BOARD_SIZE, reprojPoints, true);
		cv::resize(view.clone(), view, cv::Size(640, 480));
		std::string windowName{ std::string("reprojection0") };
		windowName[windowName.length() - 1] += i;
		cv::imshow(windowName, view);
	}
	return 0;
}
/*
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
*/
int TestCustomSystemCalib(bool useRobot = false)
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
			images.push_back(cam->GetFrame());
		}
		cv::destroyAllWindows();
	}
	else
	{
		robotPosVec.emplace_back<std::vector<double>>({ 492.00, 16.15, 693.90, 46.32, 171.64, 8.66 });
		robotPosVec.emplace_back<std::vector<double>>({ 475.72, -86.14, 709.64, 24.89, 160.99, -7.40 });
		robotPosVec.emplace_back<std::vector<double>>({ 470.69, 92.61, 697.99, 60.61, -174.38, 17.21 });
		std::vector<std::string> fileNames{
			".\\calib_data\\3\\492.000000,16.150000,693.900000,46.320000,171.640000,8.660000,.jpg",
			".\\calib_data\\3\\475.720000,-86.140000,709.640000,24.890000,160.990000,-7.400000,.jpg",
			".\\calib_data\\3\\470.690000,92.610000,697.990000,60.610000,-174.380000,17.210000,.jpg"
		};
		for (std::string file : fileNames)
		{
			images.push_back(cv::imread(file));
		}

	}
	cv::Mat R_camera2end, t_camera2end, R_custom2camera, t_custom2camera;
	int ret = CustomSystemCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_camera2end, t_camera2end, R_custom2camera, t_custom2camera);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}