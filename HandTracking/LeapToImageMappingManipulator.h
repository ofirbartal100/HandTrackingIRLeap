#pragma once
#include "ImageManipulator.h"
#include <opencv2/tracking/tracker.hpp>
#include "LeapToImageMapper.h"

class LeapToImageMappingManipulator : public ImageManipulator
{
private:
	LeapToImageMapper * _mapper;
public:
	LeapToImageMappingManipulator(LeapToImageMapper * mapper)
	{
		_mapper = mapper;
	}

	virtual void Manipulate(cv::Mat& m)
	{
		if(_mapper->isCalibrated)
		{
			for (auto p : _mapper->projections)
			{
				auto c = cv::Point(int(p.x), int(p.y));
				circle(m, c, 3, cv::Scalar(255, 250, 0), -1);
			}
		}
		else
		{
			for (auto p : _mapper->getPattern())
			{
				auto c = cv::Point(int(p.x), int(p.y));
				circle(m, c, 3, cv::Scalar(255, 250, 0), -1);
			}
		}
	}

};
