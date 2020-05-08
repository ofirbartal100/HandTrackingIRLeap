#pragma once
#include "LeapMotion.h"

#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>
#include <opencv2/calib3d.hpp>
#include "LeapToImageMapper.h"

class AlgorithmHook
{
public:
	virtual void Run(DWORD key_dword) {};
};


class CalibrationAlgorithmHook : public AlgorithmHook
{
	LeapToImageMapper* _mapper;
public:
	CalibrationAlgorithmHook(LeapToImageMapper* mapper)
	{
		_mapper = mapper;
	}

	virtual void Run(DWORD key_dword)
	{
		cout << key_dword << endl;
		if (key_dword == VK_SPACE)
		{
			_mapper->Calibrate();
		}
		if (key_dword == 0x15)
		{
			//_mapper->Map();
		}
	}
};
