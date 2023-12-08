#include "RobotKinematicsImpl.h"

#include <cmath>

using namespace Computing;

RobotKinematicsImpl::RobotKinematicsImpl(double dh1, double dh2, double dh3, double dh4, double dh5, double dh6)
{
	m_DH = DHParams();
	m_DH(0, 0) = dh1;
	m_DH(0, 1) = dh2;
	m_DH(0, 2) = dh3;
	m_DH(0, 3) = dh4;
	m_DH(0, 4) = dh5;
	m_DH(0, 5) = dh6;
}

RobotKinematicsImpl::JointAngles8d RobotKinematicsImpl::InverseResolve(double x, double y, double z, double rX, double rY, double rZ)
{
	JointAngles8d theta;
	Vector3d v(rX, rY, rZ);
	double t_alpha = v.norm();//��ģ
	v.normalize();//��׼��
	AngleAxisd rv(t_alpha, v);//��ת����
	Matrix3d rm;
	rm = rv.matrix();

	//2.���
	double A, B, C, D, E, F, G, M, N;//�ô�д��ĸ�������

	//ע�⣬���������±��0��ʼ�����⣬�����һ�е�һ�е�Ԫ����(0,0)
	//theta1
	A = rm(0, 2) * m_DH(0, 5) - x;
	B = rm(1, 2) * m_DH(0, 4) - y;
	C = m_DH(0, 3);
	//��һ���⣬����һ������
	theta(0, 0) = atan2(B, A) - atan2(C, sqrt(A * A + B * B - C * C));
	theta(1, 0) = theta(0, 0);
	theta(2, 0) = theta(0, 0);
	theta(3, 0) = theta(0, 0);
	//�ڶ����⣬�����嵽����
	theta(4, 0) = atan2(B, A) - atan2(C, -sqrt(A * A + B * B - C * C));
	theta(5, 0) = theta(4, 0);
	theta(6, 0) = theta(4, 0);
	theta(7, 0) = theta(4, 0);

	//theta5
	//��theta(0, 0)�����ĵ�һ���⣬����һ������
	A = sin(theta(0, 0)) * rm(0, 2) - cos(theta(0, 0)) * rm(1, 2);
	theta(0, 4) = atan2(sqrt(1 - A * A), A);
	theta(1, 4) = theta(0, 4);
	//��theta(0, 0)�����ĵڶ����⣬������������
	theta(2, 4) = atan2(-sqrt(1 - A * A), A);
	theta(3, 4) = theta(2, 4);
	//��theta(4, 0)�����ĵ�һ���⣬�����嵽����
	A = sin(theta(4, 0)) * rm(0, 2) - cos(theta(4, 0)) * rm(1, 2);
	theta(4, 4) = atan2(sqrt(1 - A * A), A);
	theta(5, 4) = theta(4, 4);
	//��theta[5][1]�����ĵڶ����⣬�����ߵ�����
	theta(6, 4) = atan2(-sqrt(1 - A * A), A);
	theta(7, 4) = theta(6, 4);

	//theta6
	for (int i = 0; i < 8; i++)
	{
		A = (-sin(theta(i, 0)) * rm(0, 1) + cos(theta(i, 0)) * rm(1, 1)) / theta[i][5];
		B = (sin(theta(i, 0)) * rm(0, 0) - cos(theta(i, 0)) * rm(1, 0)) / theta[i][5];
		theta[i][6] = atan2(A, B);
	}

	//theta2��theta3��theta4
	for (int i = 0; i < 8; i = i + 2)
	{
		//����theta2+theta3+theta4
		double theta234[8];
		A = rm(2, 2) / sin(theta[i][5]);
		B = (cos(theta(i, 0)) * rm(0, 2) + sin(theta(i, 0)) * rm(1, 2)) / sin(theta[i][5]);
		theta234[i] = atan2(-A, -B) - EIGEN_PI;
		theta234[i + 1] = theta234[i];

		//��ȥtheta2+theta3������theta2
		A = -cos(theta234[i]) * sin(theta[i][5]) * m_DH[6] + sin(theta234[i]) * m_DH[5];
		B = -sin(theta234[i]) * sin(theta[i][5]) * m_DH[6] - cos(theta234[i]) * m_DH[5];
		C = cos(theta(i, 0)) * x + sin(theta(i, 0)) * y;
		D = z - m_DH[1];
		M = C - A;
		N = D - B;
		E = -2 * N * a[2];
		F = 2 * M * a[2];
		G = M * M + N * N + a[2] * a[2] - a[3] * a[3];
		theta[i][2] = atan2(F, E) - atan2(G, sqrt(E * E + F * F - G * G));
		theta[i + 1][2] = atan2(F, E) - atan2(G, -sqrt(E * E + F * F - G * G));

		//��theta2����theta2+theta3
		double theta23[8 + 1];
		theta23[i] = atan2((N - sin(theta[i][2]) * a[2]) / a[3], (M - cos(theta[i][2]) * a[2]) / a[3]);
		theta23[i + 1] = atan2((N - sin(theta[i + 1][2]) * a[2]) / a[3], (M - cos(theta[i + 1][2]) * a[2]) / a[3]);

		//theta3
		theta[i][3] = theta23[i] - theta[i][2];
		theta[i + 1][3] = theta23[i + 1] - theta[i + 1][2];

		//theta4
		theta[i][4] = theta234[i] - theta23[i];
		theta[i + 1][4] = theta234[i + 1] - theta23[i + 1];
	}

}