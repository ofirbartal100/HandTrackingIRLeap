#pragma once
#include <opencv2/opencv.hpp>
#include "LeapMotion.h"


class LeapToImageMapper
{
	LeapMotion* _leap;
	vector<cv::Point2f> _pattern;

	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs, tvecs;
	const cv::Size frameSize = cv::Size(720, 540);
	double reprojection_error;

public:
	std::vector<cv::Point2f> projections;
	bool isCalibrated;

	LeapToImageMapper(LeapMotion* leap)
	{
		_leap = leap;
		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
		distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	}

	void SetPattern(vector<cv::Point2f> pattern)
	{
		_pattern = pattern;
	}

	vector<cv::Point2f> getPattern()
	{
		return _pattern;
	}

	void Calibrate()
	{
		std::vector<cv::Point3f> res = _leap->GetJoints();

		if (res.size() == 21)
		{
			auto corresponding_points = std::vector<cv::Point3f>();
			int c = 0;

			//corresponding to the palm and fingertips
			for (auto r : res)
			{
				if (c == 0 || c == 4 || c == 8 || c == 12 || c == 16 || c == 20)
				{
					corresponding_points.push_back(r);
				}
				c++;
			}

			auto object_points = std::vector<std::vector<cv::Point3f>>();
			object_points.push_back(corresponding_points);

			auto image_points = std::vector<std::vector<cv::Point2f>>();
			image_points.push_back(_pattern);

			cameraMatrix = cv::initCameraMatrix2D(object_points, image_points, frameSize, 0);

			std::cout << cameraMatrix << std::endl;
			reprojection_error = cv::calibrateCamera(object_points, image_points, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_USE_INTRINSIC_GUESS);

			std::cout << reprojection_error << std::endl;
			std::cout << distCoeffs << std::endl;

			isCalibrated = true;
		}
	}

	void StartMapping()
	{
		std::vector<cv::Point3f> res = _leap->GetJoints();
		cv::projectPoints(res, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, projections);
	};
};