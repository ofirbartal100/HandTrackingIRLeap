#pragma once
#include "ImageManipulator.h"

class PointsPatternManipulator : public ImageManipulator
{
private:
	vector<cv::Vec2i> _pattern;
public:
	PointsPatternManipulator(vector<cv::Vec2i> pattern)
	{
		_pattern = pattern;
	}

	virtual void Manipulate(cv::Mat& m)
	{
		for (auto p : _pattern)
		{
			auto c = cv::Point(p[0], p[1]);
			circle(m, c, 3, cv::Scalar(0, 0, 0), -1);
		}
	}
};

