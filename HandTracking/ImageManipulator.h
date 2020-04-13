#pragma once
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"

class ImageManipulator
{
public:
	virtual void Manipulate(cv::Mat&)
	{
		return;
	}

	virtual ~ImageManipulator()
	{

	}
};

