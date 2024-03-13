#pragma once

#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

#include "opencv2/opencv.hpp"
#include "ceres/autodiff_cost_function.h"

#include "CameraManager.h"
#include "RobotMoveSubSystem.h"
#include "CameraCalibrationDemo.h"

void PosToRT(const std::vector<double> pos, cv::Mat& R_end2robot, cv::Mat& t_end2robot)
{
	double rz{ pos[3] * CV_PI / 180 }, ry{ pos[4] * CV_PI / 180 }, rx{ pos[5] * CV_PI / 180 };
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
	R_end2robot = R_x * R_y * R_z;

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
	transformationMat.at<double>(3, 3) = 1.0; // 添加常量1
	return transformationMat;
}

cv::Mat RtVec2T(cv::Mat rvec, cv::Mat tvec)
{
	// 提取 R_gripper2base 的旋转矩阵和平移向量
	cv::Mat R_gripper2base;
	cv::Rodrigues(rvec, R_gripper2base);

	return Rt2T(R_gripper2base, tvec);
}

cv::Mat skew(const cv::Mat& v)
{
	double vx = v.at<double>(0, 0);
	double vy = v.at<double>(1, 0);
	double vz = v.at<double>(2, 0);
	return (cv::Mat_<double>(3, 3) << 0, -vz, vy,
		vz, 0, -vx,
		-vy, vx, 0);
}

cv::Mat homogeneousInverse(const cv::Mat& T)
{
	CV_Assert(T.rows == 4 && T.cols == 4);

	cv::Mat R = T(cv::Rect(0, 0, 3, 3));
	cv::Mat t = T(cv::Rect(3, 0, 1, 3));
	cv::Mat Rt = R.t();
	cv::Mat tinv = -Rt * t;
	cv::Mat Tinv = cv::Mat::eye(4, 4, T.type());
	Rt.copyTo(Tinv(cv::Rect(0, 0, 3, 3)));
	tinv.copyTo(Tinv(cv::Rect(3, 0, 1, 3)));

	return Tinv;
}

cv::Mat rot2quatMinimal(const cv::Mat& R)
{

	double m00 = R.at<double>(0, 0), m01 = R.at<double>(0, 1), m02 = R.at<double>(0, 2);
	double m10 = R.at<double>(1, 0), m11 = R.at<double>(1, 1), m12 = R.at<double>(1, 2);
	double m20 = R.at<double>(2, 0), m21 = R.at<double>(2, 1), m22 = R.at<double>(2, 2);
	double trace = m00 + m11 + m22;

	double qx, qy, qz;
	if (trace > 0) {
		double S = sqrt(trace + 1.0) * 2; // S=4*qw
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S;
		qz = (m10 - m01) / S;
	}
	else if (m00 > m11 && m00 > m22) {
		double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
		qx = 0.25 * S;
		qy = (m01 + m10) / S;
		qz = (m02 + m20) / S;
	}
	else if (m11 > m22) {
		double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		qx = (m01 + m10) / S;
		qy = 0.25 * S;
		qz = (m12 + m21) / S;
	}
	else {
		double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}

	return (cv::Mat_<double>(3, 1) << qx, qy, qz);
}

cv::Mat quatMinimal2rot(const cv::Mat& q)
{

	cv::Mat p = q.t() * q;
	double w = sqrt(1 - p.at<double>(0, 0));

	cv::Mat diag_p = cv::Mat::eye(3, 3, CV_64FC1) * p.at<double>(0, 0);
	return 2 * q * q.t() + 2 * w * skew(q) + cv::Mat::eye(3, 3, CV_64FC1) - 2 * diag_p;
}

template <typename T>
void DeepCopy(const T* src, T* dst, int copyLen)
{
	for (int i = 0; i < copyLen; i++)
	{
		dst[i] = src[i];
	}
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

template<typename T1, typename T2, typename T3>
void DistanceSqr(const T1 const* pa, const T2 const* pb, int dimN, T3& distanceSqr)
{
	distanceSqr = (T3(pa[0]) - T3(pb[0])) * (T3(pa[0]) - T3(pb[0]));
	for (int i = 1; i < dimN; i++)
	{
		distanceSqr += (T3(pa[i]) - T3(pb[i])) * (T3(pa[i]) - T3(pb[i]));
	}
}

class A9dXeqB3dDistanceCostFunctor
{
public:

	A9dXeqB3dDistanceCostFunctor(const cv::Mat A, const cv::Mat B, double trustXNormSqr) :
		m_A(std::vector<double>((size_t)9, 0)),
		m_B(std::vector<double>((size_t)3, 0)),
		m_trustXNormSqr(trustXNormSqr)
	{
		DeepCopy((double*)A.data, m_A.data(), 9);
		DeepCopy((double*)B.data, m_B.data(), 3);
	}

	~A9dXeqB3dDistanceCostFunctor() = default;

	template<typename T>
	bool operator()(const T const X[3], T residual[1]) const
	{
		/*T zeros[3]{T(0),T(0),T(0)};
		T XNorm{T(0)};
		DistanceSqr(X, zeros, 3, XNorm);
		if (XNorm > T(m_trustXNormSqr))
		{
			return false;
		}*/

		T A[9]{};
		ArrayTypeConvert(m_A.data(), A, 9);
		T B[3]{};
		ArrayTypeConvert(m_B.data(), B, 3);

		T B_estimate[3]{};
		Matrix3x3Mul3x1(A, X, B_estimate);
		//LeastSquare
		DistanceSqr(B, B_estimate, 3, residual[0]);
		return true;
	}
private:
	std::vector<double> m_A;
	std::vector<double> m_B;
	double m_trustXNormSqr;
};

int EyeInHandCalibration(const std::vector<cv::Mat> images, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, const std::vector<std::vector<double>> robotPosVec,
	cv::Mat& R_cam2gripper, cv::Mat& t_cam2gripper)
{
	cv::Mat cameraMatrix;
	cv::Mat distCoeff;
	CameraCalibrationDemoMain(images, BOARD_SIZE.width, BOARD_SIZE.height, SQUARE_SIZE, cameraMatrix, distCoeff);

	std::vector<std::vector<cv::Point3f>> objPoints{};
	std::vector< std::vector<cv::Point2f>> imagePoints{};
	for (cv::Mat origin : images)
	{
		//cv::Mat origin = cv::imread(fileName);
		std::vector<cv::Point2f> corners{};
		if (FindChessboradCorners(origin, BOARD_SIZE, corners))
		{
			//drawChessboardCorners(origin, BOARD_SIZE, corners, true);
			imagePoints.push_back(corners);
		}
		//cv::Mat view;
		// //缩小，不然我电脑显示不完全
		//resize(origin, view, cv::Size(640, 480));
		//cv::imshow("result", view);
		//cv::waitKey();

	}

	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	std::vector<cv::Mat> Hg, Hc;
	for (int i = 0; i < objPoints.size(); i++) {
		cv::Mat rvec, tvec;
		if (!cv::solvePnP(objPoints[i], imagePoints[i], cameraMatrix, distCoeff, rvec, tvec))
		{
			std::cout << "Falied to solvePnP" << i << std::endl;
		}

		Hc.push_back(RtVec2T(rvec, tvec));



		cv::Mat R_gripper2base, t_gripper2base;
		PosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);

		Hg.push_back(Rt2T(R_gripper2base, t_gripper2base));
	}

	//Number of unique camera position pairs
	int K = static_cast<int>((Hg.size() * Hg.size() - Hg.size()) / 2.0);

	std::vector<cv::Mat> vec_Hgij, vec_Hcij;
	vec_Hgij.reserve(static_cast<size_t>(K));
	vec_Hcij.reserve(static_cast<size_t>(K));

	cv::Mat Pcg_{ cv::Mat::eye(3,1, CV_64F) };
	//Will store: skew(Pgij+Pcij)
	cv::Mat A(3 * K, 3, CV_64FC1);
	//Will store: Pcij - Pgij
	cv::Mat B(3 * K, 1, CV_64FC1);
	ceres::Problem problem1;
	int idx = 0;
	for (size_t i = 0; i < Hg.size(); i++)
	{
		for (size_t j = i + 1; j < Hg.size(); j++, idx++)
		{
			//Defines coordinate transformation from Gi to Gj
			//Hgi is from Gi (gripper) to RW (robot base)
			//Hgj is from Gj (gripper) to RW (robot base)
			cv::Mat Hgij = homogeneousInverse(Hg[j]) * Hg[i]; //eq 6
			vec_Hgij.push_back(Hgij);
			//Rotation axis for Rgij which is the 3D rotation from gripper coordinate frame Gi to Gj
			cv::Mat Pgij = 2 * rot2quatMinimal(Hgij);

			//Defines coordinate transformation from Ci to Cj
			//Hci is from CW (calibration target) to Ci (camera)
			//Hcj is from CW (calibration target) to Cj (camera)
			cv::Mat Hcij = Hc[j] * homogeneousInverse(Hc[i]); //eq 7
			vec_Hcij.push_back(Hcij);
			//Rotation axis for Rcij
			cv::Mat Pcij = 2 * rot2quatMinimal(Hcij);

			//Left-hand side: skew(Pgij+Pcij)
			//Right-hand side: Pcij - Pgij
			cv::Mat diff = Pcij - Pgij;
			//Rotation from camera to gripper is obtained from the set of equations:
			//    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij    (eq 12)
			problem1.AddResidualBlock(
				new ceres::AutoDiffCostFunction<A9dXeqB3dDistanceCostFunctor, 1, 3>(
					new A9dXeqB3dDistanceCostFunctor(skew(Pgij + Pcij), diff, 1)
				),
				nullptr,
				(double*)Pcg_.data);
			//Left-hand side: skew(Pgij+Pcij)
			skew(Pgij + Pcij).copyTo(A(cv::Rect(0, idx * 3, 3, 3)));
			//Right-hand side: Pcij - Pgij
			diff.copyTo(B(cv::Rect(0, idx * 3, 1, 3)));
		}
	}

	//use ceres solver
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem1, &summary);
	std::cout << "Rcg solve summary:" << summary.BriefReport() << std::endl;

	//use OpenCV
	//Rotation from camera to gripper is obtained from the set of equations:
	//    skew(Pgij+Pcij) * Pcg_ = Pcij - Pgij    (eq 12)
	//cv::solve(A, B, Pcg_, cv::DECOMP_SVD);

	cv::Mat Pcg_norm = Pcg_.t() * Pcg_;
	//Obtained non-unit quaternion is scaled back to unit value that
	//designates camera-gripper rotation
	cv::Mat Pcg = 2 * Pcg_ / sqrt(1 + Pcg_norm.at<double>(0, 0)); //eq 14

	cv::Mat Rcg = quatMinimal2rot(Pcg / 2.0);

	cv::Mat Tcg{ cv::Mat::zeros(3,1,CV_64F) };
	ceres::Problem problem2;
	idx = 0;
	for (size_t i = 0; i < Hg.size(); i++)
	{
		for (size_t j = i + 1; j < Hg.size(); j++, idx++)
		{
			//Defines coordinate transformation from Gi to Gj
			//Hgi is from Gi (gripper) to RW (robot base)
			//Hgj is from Gj (gripper) to RW (robot base)
			cv::Mat Hgij = vec_Hgij[static_cast<size_t>(idx)];
			//Defines coordinate transformation from Ci to Cj
			//Hci is from CW (calibration target) to Ci (camera)
			//Hcj is from CW (calibration target) to Cj (camera)
			cv::Mat Hcij = vec_Hcij[static_cast<size_t>(idx)];

			//Left-hand side: (Rgij - I)
			cv::Mat diffLeft = Hgij(cv::Rect(0, 0, 3, 3)) - cv::Mat::eye(3, 3, CV_64FC1);

			//Right-hand side: Rcg*Tcij - Tgij
			cv::Mat diffRight = Rcg * Hcij(cv::Rect(3, 0, 1, 3)) - Hgij(cv::Rect(3, 0, 1, 3));
			//Translation from camera to gripper is obtained from the set of equations:
			//    (Rgij - I) * Tcg = Rcg*Tcij - Tgij    (eq 15)
			problem2.AddResidualBlock(
				new ceres::AutoDiffCostFunction<A9dXeqB3dDistanceCostFunctor, 1, 3>(
					new A9dXeqB3dDistanceCostFunctor(diffLeft, diffRight, 10000)
				),
				nullptr,
				(double*)Tcg.data);
			//Left-hand side: (Rgij - I)
			diffLeft.copyTo(A(cv::Rect(0, idx * 3, 3, 3)));

			//Right-hand side: Rcg*Tcij - Tgij
			diffRight.copyTo(B(cv::Rect(0, idx * 3, 1, 3)));
		}
	}

	//use ceres solver
	ceres::Solve(options, &problem2, &summary);
	std::cout << "Tcg solve summary:" << summary.BriefReport() << std::endl;

	//use OpenCV
	//Translation from camera to gripper is obtained from the set of equations:
	//    (Rgij - I) * Tcg = Rcg*Tcij - Tgij    (eq 15)
	//cv::solve(A, B, Tcg, cv::DECOMP_SVD);

	R_cam2gripper = Rcg;
	t_cam2gripper = Tcg;

	std::cout << "R_cam2gripper:" << R_cam2gripper << std::endl;
	std::cout << "t_cam2gripper:" << t_cam2gripper << std::endl;

	return 0;
}

int TestEyeInHandCalib(bool useRobot = false)
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
	cv::Mat R_gripper2cam, t_gripper2cam;
	int ret = EyeInHandCalibration(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, R_gripper2cam, t_gripper2cam);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}