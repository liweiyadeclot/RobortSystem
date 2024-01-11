#include <cstdio>
#include <cstring>
#include <fstream>
#include "CameraManager.h"
#include "opencv2/opencv.hpp"

using std::ofstream;

bool FindChessboradCorners(cv::InputArray image, cv::Size patternSize, std::vector<cv::Point2f> outCorners,
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
	}
	return found;
}

int main(int argc, char** argv)
{
	CameraManager* s_CameraManager = CameraManager::GetInstance();
	std::shared_ptr<Camera> cam = s_CameraManager->GetOrOpenCamera();
	std::vector<std::vector<cv::Point2f> > imagePoints;


	// ѭ����ȡ����ͷ��׽����֡���������̸��λ
	while (true)
	{
		// ��ȡһ֡ͼ��
		CameraFrame frame{ cam->GetFrame() };

		cv::Mat view;
		// ��С����Ȼ�ҵ�����ʾ����ȫ
		cv::resize(frame, view, cv::Size(640, 480));
		// ��ʾͼ��
		cv::imshow("Camera", view);
		// �ȴ��س�����������ã�����������������������q���˳�ѭ����ʼ�궨
		int key = cv::waitKey(1);
		if (key == 'q')
		{
			break;
		}
		else if (key != '\r')
		{
			continue;
		}


		std::vector<cv::Point2f> corners;
		if (true == FindChessboradCorners(frame, cv::Size(9, 6), corners))
		{
			imagePoints.push_back(corners);
			cv::drawChessboardCorners(frame, cv::Size(9, 6), corners, true);
		}
		else
		{
			cv::putText(frame, "Corners not found", cv::Point(0,0), 1, 1, cv::Scalar::all(255));
		}
		// ��С����Ȼ�ҵ�����ʾ����ȫ
		cv::resize(frame.clone(), frame, cv::Size(640, 480));
		// ��ʾ�ҵ��Ľǵ�
		cv::imshow("Camera", frame);

		// �ȴ���������£�����q���˳�ѭ����ʼ�궨
		if (cv::waitKey() == 'q')
			break;
	}

	// �ͷ����д���
	cv::destroyAllWindows();

	return 0;
}