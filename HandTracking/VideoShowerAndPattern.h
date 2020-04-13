#pragma once
#include "VideoShower.h"
#include "ImageManipulator.h"

class VideoShowerAndPattern : public VideoShower
{

public:
	vector<cv::Vec2i> _pattern;

	void ApplyImageManipulation(ImageManipulator* mani)
	{
		manipulator = mani;
		is_image_manipulation = true;
	}

	void StopImageManipulation()
	{
		is_image_manipulation = false;
	}
};

