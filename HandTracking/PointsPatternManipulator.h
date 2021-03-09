#pragma once
#include "ImageManipulator.h"

class PointsPatternManipulator : public ImageManipulator
{
private:
	vector<cv::Point2f> _pattern;
    int _circle_radius;
    cv::Scalar _mapperPointColor;

public:
	PointsPatternManipulator(vector<cv::Point2f> pattern)
	{
		_pattern = pattern;
        _circle_radius = 3;
        _mapperPointColor = cv::Scalar(255, 255, 255);

	}
    PointsPatternManipulator(vector<cv::Point2f> pattern, cv::Scalar mapperPointColor)
    {
        _pattern = pattern;
        _circle_radius = 3;
        _mapperPointColor = mapperPointColor;
    }

	virtual void Manipulate(cv::Mat& m)
	{
		for (auto p : _pattern)
		{
			auto c = cv::Point(int(p.x), int(p.y));
			circle(m, c, _circle_radius, _mapperPointColor, -1);
		}
	}

	void ChangePoints(vector<cv::Point2f> pattern)
	{
		_pattern = pattern;
	}
};

