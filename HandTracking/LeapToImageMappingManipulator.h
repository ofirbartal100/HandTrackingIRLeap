#pragma once
#include "ImageManipulator.h"
#include <opencv2/tracking/tracker.hpp>
#include "LeapToImageMapper.h"

class LeapToImageMappingManipulator : public ImageManipulator
{
private:
	LeapToImageMapper * _mapper;
	int circle_radius;
    cv::Scalar _mapperPointColor;
public:
	LeapToImageMappingManipulator(LeapToImageMapper * mapper)
	{
		_mapper = mapper;
		circle_radius = 3;
        _mapperPointColor = cv::Scalar(255, 255, 255);
	}

    LeapToImageMappingManipulator(LeapToImageMapper * mapper, cv::Scalar mapperPointColor)
    {
        _mapper = mapper;
        circle_radius = 3;
        _mapperPointColor = mapperPointColor;
    }

	virtual void Manipulate(cv::Mat& m)
	{
		if (_mapper->isCalibrated)
		{
			cv::Point c;
			if (_mapper->Map())
			{
				for (auto p : _mapper->projections)
				{
					c = cv::Point(int(p.x), int(p.y));
					if (m.rows <= c.y || c.y < 0 || m.cols <= c.x || c.x < 0) continue;
					circle(m, c, circle_radius, _mapperPointColor, -1);
				}
			}
			//cout << _mapper->projections.size() << endl;
		}
		else
		{
			for (auto p : _mapper->getPattern())
			{
				auto c = cv::Point(int(p.x), int(p.y));
				circle(m, c, 3, cv::Scalar(255, 255, 255), -1);
			}
		}
	}

};
