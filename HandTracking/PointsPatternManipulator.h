#pragma once
#include "ImageManipulator.h"

class PointsPatternManipulator : public ImageManipulator
{
private:
	vector<cv::Point2f> _pattern;
public:
	PointsPatternManipulator(vector<cv::Point2f> pattern)
	{
		_pattern = pattern;
	}

	virtual void Manipulate(cv::Mat& m)
	{
		for (auto p : _pattern)
		{
			auto c = cv::Point(int(p.x), int(p.y));
			circle(m, c, 3, cv::Scalar(255, 250, 0), -1);
		}
	}

	void ChangePoints(vector<cv::Point2f> pattern)
	{
		_pattern = pattern;
	}
};

