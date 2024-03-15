#pragma once
#include <vector>

#include "opencv2/opencv.hpp"
#include "ceres/autodiff_cost_function.h"
#include "ceres/loss_function.h"
#include "ceres/problem.h"
#include "ceres/solver.h"
#include "ceres/rotation.h"

#include "RobotMoveSubSystem.h"
#include "CameraManager.h"

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
		/*cv::Mat view{ image.getMat().clone() };
		cv::drawChessboardCorners(view, patternSize, outCorners, true);
		cv::resize(view.clone(), view, cv::Size(640, 480));
		cv::imshow("Chessboard", view);
		cv::waitKey(3*1000);*/
	}
	return found;
}

void genChessBoardObjectPoints(const cv::Size BOARD_SIZE, const uint32_t SQUARE_SIZE, const std::vector<std::vector<cv::Point2f> >& imagePoints, std::vector<std::vector<cv::Point3f>>& objPoints)
{
	std::vector<cv::Point3f> corners;
	for (int i = 0; i < BOARD_SIZE.height; ++i) {
		for (int j = 0; j < BOARD_SIZE.width; ++j) {
			corners.emplace_back(i * SQUARE_SIZE, j * SQUARE_SIZE, 0);
		}
	}
	objPoints.resize(imagePoints.size(), corners);
}

void PosToRT(const std::vector<double> pos, cv::Mat& R_end2robot, cv::Mat& t_end2robot)
{
	double rz{ pos[3] * CV_PI / 180.0f }, ry{ pos[4] * CV_PI / 180.0f }, rx{ pos[5] * CV_PI / 180.0f };
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
	/*double S{ 2 * sin(acos((trace - 1) / 2)) };
	double qx{ (m21 - m12) / S };
	double qy{ (m02 - m20) / S };
	double qz{ (m10 - m01) / S };*/
	return /*sin((acos((trace - 1) / 2)) / 2) * */(cv::Mat_<double>(3, 1) << qx, qy, qz);
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

	A9dXeqB3dDistanceCostFunctor(const cv::Mat A, const cv::Mat B, double trustXNormSqr, bool debug = false) :
		m_A(std::vector<double>((size_t)9, 0)),
		m_B(std::vector<double>((size_t)3, 0)),
		m_trustXNormSqr(trustXNormSqr),
		m_debug(debug)
	{
		DeepCopy((double*)A.data, m_A.data(), 9);
		DeepCopy((double*)B.data, m_B.data(), 3);
	}

	~A9dXeqB3dDistanceCostFunctor() = default;

	template<typename T>
	bool operator()(const T const X[3], T residual[1]) const
	{
		T zeros[3]{ T(0),T(0),T(0) };
		T XNorm{ T(0) };
		DistanceSqr(X, zeros, 3, XNorm);
		if (XNorm > T(m_trustXNormSqr))
		{
			return false;
		}

		T A[9]{};
		ArrayTypeConvert(m_A.data(), A, 9);
		T B[3]{};
		ArrayTypeConvert(m_B.data(), B, 3);

		T B_estimate[3]{};
		Matrix3x3Mul3x1(A, X, B_estimate);
		//LeastSquare
		DistanceSqr(B, B_estimate, 3, residual[0]);
		if (m_debug)
		{
			std::cout << "B:" << cv::Mat(3, 1, CV_64F, B) << ", B_esti:" << cv::Mat(3, 1, CV_64F, B_estimate) << std::endl;
		}
		return true;
	}
private:
	std::vector<double> m_A;
	std::vector<double> m_B;
	double m_trustXNormSqr;
	bool m_debug;
};

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
	bool operator()(const T const q_end2camera[4], const T const t_end2camera[3], const T const q_custom2robot[4], const T const t_custom2robot[3], T residual[1]) const
	{
		T P_custom[3]{};
		ArrayTypeConvert(m_P_custom, P_custom, 3);
		T q_robot2end[4]{};
		ArrayTypeConvert(m_q_robot2end, q_robot2end, 4);
		T t_robot2end[3]{};
		ArrayTypeConvert(m_t_robot2end, t_robot2end, 3);
		/*T q_end2camera[4]{};
		ArrayTypeConvert(m_q_end2camera, q_end2camera, 4);
		T t_end2camera[3]{};
		ArrayTypeConvert(m_t_end2camera, t_end2camera, 3);*/
		T cameraMatrix[9]{};
		ArrayTypeConvert(m_cameraMatrix, cameraMatrix, 9);

		T P_pixel_esti[3]{};
		projection(P_custom, q_custom2robot, t_custom2robot, q_robot2end, t_robot2end, q_end2camera, t_end2camera, cameraMatrix, P_pixel_esti);
		DistanceSqr(m_P_pixel, P_pixel_esti, 2, residual[0]);

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

int CalibrateAll(std::vector<cv::Mat> inImageVec, const cv::Size& BOARD_SIZE, const uint32_t& SQUARE_SIZE, std::vector<std::vector<double>> inRobotPosVec,
	cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& R_camera2end, cv::Mat& t_camera2end, cv::Mat& R_custom2robot, cv::Mat& t_custom2robot)
{

	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<cv::Mat> imageVec;
	std::vector<std::vector<double>> robotPosVec;
	for (int i = 0; i < inImageVec.size(); i++)
	{
		std::vector<cv::Point2f> corners;
		if (true == FindChessboradCorners(inImageVec[i], BOARD_SIZE, corners))
		{
			imagePoints.push_back(corners);
			imageVec.push_back(inImageVec[i]);
			robotPosVec.push_back(inRobotPosVec[i]);
		}
	}
	std::vector<std::vector<cv::Point3f>> objPoints{};
	genChessBoardObjectPoints(BOARD_SIZE, SQUARE_SIZE, imagePoints, objPoints);
	// start camera calibration
	std::vector<cv::Mat> rvec_custom2cameraVecs, tvec_custom2cameraVecs;
	cv::calibrateCamera(objPoints, imagePoints, imageVec[0].size(), cameraMatrix, distCoeffs, rvec_custom2cameraVecs, tvec_custom2cameraVecs);
	// end of camera calibration
	//start eye-in-hand calibration (partly copy from OpenCV source)
	//计算和收集所需数据
	std::vector<cv::Mat> Hg, Hc, R_robot2endVec, t_robot2endVec, R_gripper2baseVec, t_gripper2baseVec;
	for (int i = 0; i < imagePoints.size(); i++) {
		cv::Mat rvec{ rvec_custom2cameraVecs[i] }, tvec{ tvec_custom2cameraVecs[i] };
		Hc.push_back(RtVec2T(rvec, tvec));

		cv::Mat R_gripper2base, t_gripper2base;
		PosToRT(robotPosVec[i], R_gripper2base, t_gripper2base);
		Hg.push_back(Rt2T(R_gripper2base, t_gripper2base));
		cv::Mat rvec_gripper2base;
		cv::Rodrigues(R_gripper2base, rvec_gripper2base);
		R_gripper2baseVec.push_back(rvec_gripper2base);
		t_gripper2baseVec.push_back(t_gripper2base);

		cv::Mat R_robot2end;
		cv::transpose(R_gripper2base, R_robot2end);
		R_robot2endVec.push_back(R_robot2end);
		t_robot2endVec.push_back(-R_robot2end * t_gripper2base);
	}
	//cv::calibrateHandEye(R_gripper2baseVec, t_gripper2baseVec, rvec_custom2cameraVecs, tvec_custom2cameraVecs, R_camera2end, t_camera2end);
	//std::cout << "R_camera2end:" << R_camera2end << std::endl
	//	<< "t_camera2end:" << t_camera2end << std::endl;
	//Number of unique camera position pairs
	int K = static_cast<int>((Hg.size() * Hg.size() - Hg.size()) / 2.0);

	std::vector<cv::Mat> vec_Hgij, vec_Hcij;
	vec_Hgij.reserve(static_cast<size_t>(K));
	vec_Hcij.reserve(static_cast<size_t>(K));

	cv::Mat Pcg_{ cv::Mat::eye(3,1, CV_64F) };
	ceres::Problem problem1;
	ceres::Solver::Options options;
	options.minimizer_progress_to_stdout = false;
	//options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
	//options.trust_region_strategy_type = ceres::DOGLEG;
	options.max_num_iterations = 10000;
	ceres::Solver::Summary summary;
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
			cv::Mat Hcit = homogeneousInverse(Hc[i]);
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
					new A9dXeqB3dDistanceCostFunctor(skew(Pgij + Pcij), diff, DBL_MAX)
				),
				/*new ceres::HuberLoss(1.0),*/
				nullptr,
				(double*)Pcg_.data);
			// compute initial value
			if (i == 0 && j == 1)
			{
				ceres::Solve(options, &problem1, &summary);
			}

		}
	}

	// Run the solver!
	ceres::Solve(options, &problem1, &summary);
	std::cout << "Pcg " << summary.BriefReport() << std::endl;

	cv::Mat Pcg_norm = Pcg_.t() * Pcg_;
	//Obtained non-unit quaternion is scaled back to unit value that
	//designates camera-gripper rotation
	cv::Mat Pcg = 2 * Pcg_ / sqrt(1 + Pcg_norm.at<double>(0, 0)); //eq 14

	cv::Mat Rcg = quatMinimal2rot(Pcg / 2.0);
	//Lemma I
	std::cout << "Assert \n" << Hg[0](cv::Rect(0, 0, 3, 3)) * Rcg * Hc[0](cv::Rect(0, 0, 3, 3)) << " == \n" << Hg[1](cv::Rect(0, 0, 3, 3)) * Rcg * Hc[1](cv::Rect(0, 0, 3, 3)) << std::endl;
	std::cout << "Assert \n" << Rcg * Rcg.t() << " == \n" << cv::Mat::eye(3, 3, CV_64F) << std::endl;
	std::cout << "Assert " << cv::determinant(Rcg) << "== 1" << std::endl;
	//Lemma II
	std::cout << "Assert \n" << Rcg * rot2quatMinimal(vec_Hcij[0](cv::Rect(0, 0, 3, 3))) << "==\n" << rot2quatMinimal(vec_Hgij[0](cv::Rect(0, 0, 3, 3))) << std::endl;
	//Lemma III
	std::cout << "Assert \n" << Pcg.t()*(rot2quatMinimal(vec_Hgij[0](cv::Rect(0, 0, 3, 3))) - rot2quatMinimal(vec_Hcij[0](cv::Rect(0, 0, 3, 3)))) << "==0\n" << std::endl;
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
			std::cout << "diffLeft:\n" << diffLeft << std::endl;
			std::cout << "diffRight:\n" << diffRight << std::endl;
			//Translation from camera to gripper is obtained from the set of equations:
			//    (Rgij - I) * Tcg = Rcg*Tcij - Tgij    (eq 15)
			problem2.AddResidualBlock(
				new ceres::AutoDiffCostFunction<A9dXeqB3dDistanceCostFunctor, 1, 3>(
					new A9dXeqB3dDistanceCostFunctor(diffLeft, diffRight, DBL_MAX)
				),
				/*new ceres::HuberLoss(1.0),*/
				nullptr,
				(double*)Tcg.data);
			// compute initial value
			if (i == 0 && j == 1)
			{
				cv::solve(diffLeft, diffRight, Tcg, cv::DECOMP_SVD);
				std::cout << "Tcg init:" << Tcg << std::endl;
			}
		}
	}

	ceres::Solve(options, &problem2, &summary);
	std::cout << "Tcg " << summary.BriefReport() << std::endl;
	std::cout << "Assert " << vec_Hgij[0](cv::Rect(0, 0, 3, 3)) * Tcg + vec_Hgij[0](cv::Rect(3, 0, 1, 3)) << std::endl
		<< " == \n" << Rcg * vec_Hcij[0](cv::Rect(3, 0, 1, 3)) + Tcg << std::endl;
	R_camera2end = Rcg;
	t_camera2end = Tcg;
	std::cout << "R_camera2end:" << R_camera2end << std::endl
		<< "t_camera2end:" << t_camera2end << std::endl;
	//end of eye-in-hand calibration
	//start custom system calibration

	double* q_custom2robotArray = (double*)malloc(4 * sizeof(double));
	double* t_custom2robotArray = (double*)malloc(3 * sizeof(double));
	memset(q_custom2robotArray, 0, 4 * sizeof(double));
	q_custom2robotArray[0] = 1;
	memset(t_custom2robotArray, 0, 3 * sizeof(double));


	cv::Mat obj{ cv::Mat(3,1,CV_64F) };

	cv::Mat R_end2camera;
	cv::transpose(R_camera2end, R_end2camera);
	cv::Mat t_end2camera{ -R_end2camera * t_camera2end };
	double* q_end2cameraArray = (double*)malloc(4 * sizeof(double));
	double* t_end2cameraArray = (double*)t_end2camera.data;
	CVRotationMatrixToQuaternion(R_end2camera, q_end2cameraArray);

	ceres::Problem problem3;
	// Set up the only cost function (also known as residual). This uses
	// auto-differentiation to obtain the derivative (jacobian).
	for (int i = 0; i < imagePoints.size(); i++)
	{
		for (int j = 0; j < imagePoints[i].size(); j++)
		{
			cv::Mat R_robot2end{ R_robot2endVec[i] };
			cv::Mat t_robot2end{ t_robot2endVec[i] };
			problem3.AddResidualBlock(
				new ceres::AutoDiffCostFunction<ReprojectionCostFunctor, 1, 4, 3, 4, 3>(new ReprojectionCostFunctor(R_robot2end, t_robot2end, R_end2camera, t_end2camera, cameraMatrix, distCoeffs, cv::Mat(objPoints[i][j]), cv::Mat(imagePoints[i][j]))),
				new ceres::HuberLoss(1.0),
				//nullptr,
				q_end2cameraArray, t_end2cameraArray, q_custom2robotArray, t_custom2robotArray);
		}
	}

	ceres::Solve(options, &problem3, &summary);
	double* R_custom2robotArray = (double*)malloc(9 * sizeof(double));
	ceres::QuaternionToRotation(q_custom2robotArray, R_custom2robotArray);
	R_custom2robot = cv::Mat(3, 3, CV_64F, R_custom2robotArray);
	t_custom2robot = cv::Mat(3, 1, CV_64F, t_custom2robotArray);
	ceres::QuaternionToRotation(q_end2cameraArray, (double*)R_end2camera.data);
	cv::transpose(R_end2camera, R_camera2end);
	t_camera2end = -R_camera2end * t_end2camera;
	std::cout << summary.BriefReport() << std::endl
		<< "R_camera2end:" << R_camera2end << std::endl
		<< "t_camera2end:" << t_camera2end << std::endl
		<< "R_custom2robot:" << R_custom2robot << std::endl
		<< "t_custom2robot:" << t_custom2robot << std::endl;
	// end of custom system calibration
	// start reprojection validation
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
			estimateP_gripper = R_robot2endVec[i] * estimateP_base + t_robot2endVec[i];
			estimateP_camera = R_end2camera * estimateP_gripper + t_end2camera;
			estimateP_cameraVec.emplace_back(estimateP_camera);
			/*cv::Mat estimateHomoP_image = cameraMatrix * estimateP_camera / estimateP_camera.at<double>(2, 0);
			reprojPoints.emplace_back(estimateHomoP_image.at<double>(0, 0), estimateHomoP_image.at<double>(1, 0));*/
		}
		cv::projectPoints(estimateP_cameraVec, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, reprojPoints);
		cv::Mat view;
		cv::undistort(imageVec[i].clone(), view, cameraMatrix, distCoeffs);
		drawChessboardCorners(view, BOARD_SIZE, reprojPoints, true);
		cv::resize(view.clone(), view, cv::Size(640, 480));
		std::string windowName{ std::string("reprojection0") };
		windowName[windowName.length() - 1] += i;
		cv::imshow(windowName, view);
	}
	return 0;
}

int TestCalibrateAllDemo(bool useRobot = false)
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
	cv::Mat cameraMatrix, distCoeffs, R_camera2end, t_camera2end, R_custom2camera, t_custom2camera;
	int ret = CalibrateAll(images, BOARD_SIZE, SQUARE_SIZE, robotPosVec, cameraMatrix, distCoeffs, R_camera2end, t_camera2end, R_custom2camera, t_custom2camera);
	cv::waitKey();
	cv::destroyAllWindows();
	return ret;
}
